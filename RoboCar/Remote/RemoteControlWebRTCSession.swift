//
//  RemoteControlWebRTCSession.swift
//  RoboCar
//

import Foundation
import UIKit
import WebRTC

final class RemoteControlWebRTCSession: NSObject {
    enum Role {
        case host
        case client
    }

    var onSignal: ((RemoteMessage) -> Void)?
    var onVideoTrack: ((RTCVideoTrack) -> Void)?
    var onStatusChanged: ((String) -> Void)?
    var onLocalFrameSent: ((Int, Int, UInt64) -> Void)?
    var onRemoteFrameRendered: ((CGSize, UInt64) -> Void)?
    var onRemoteFrameImage: ((UIImage) -> Void)?

    private static let factory: RTCPeerConnectionFactory = {
        RTCInitializeSSL()
        return RTCPeerConnectionFactory(
            encoderFactory: RTCDefaultVideoEncoderFactory(),
            decoderFactory: RTCDefaultVideoDecoderFactory()
        )
    }()

    private let role: Role
    private let queue = DispatchQueue(label: "com.robocar.remote.webrtc", qos: .userInitiated)
    private var peerConnection: RTCPeerConnection?
    private var videoTrack: RTCVideoTrack?
    private var viewStreamer: RemoteRenderedViewStreamer?
    private var remoteVideoTrack: RTCVideoTrack?
    private var remoteFrameObserver: RemoteVideoFrameObserver?
    private var hasVideoReceiver = false
    private var pendingCandidates: [RTCIceCandidate] = []

    init(role: Role) {
        self.role = role
        super.init()
    }

    var canSendVideo: Bool {
        guard let peerConnection,
              viewStreamer != nil else { return false }
        return peerConnection.iceConnectionState == .connected || peerConnection.iceConnectionState == .completed
    }
    
    var canAcceptVideoFrame: Bool {
        viewStreamer != nil
    }

    var hasPeerConnection: Bool {
        peerConnection != nil
    }

    func start() {
        queue.async { [weak self] in
            guard let self else { return }
            self.createPeerConnectionIfNeeded()
            if self.role == .host {
                self.configureLocalVideoTrackIfNeeded()
                self.createOffer()
            } else {
                self.configureRemoteVideoReceiverIfNeeded()
            }
        }
    }

    func stop() {
        queue.async { [weak self] in
            guard let self else { return }
            self.peerConnection?.close()
            self.peerConnection = nil
            self.videoTrack = nil
            self.viewStreamer = nil
            self.remoteVideoTrack = nil
            self.remoteFrameObserver = nil
            self.pendingCandidates.removeAll()
            self.publishStatus("WebRTC disconnected")
        }
    }

    func handleSignal(_ message: RemoteMessage) {
        queue.async { [weak self] in
            guard let self else { return }
            self.createPeerConnectionIfNeeded()
            if self.role == .host {
                self.configureLocalVideoTrackIfNeeded()
            }

            switch message.signalType {
            case "offer", "answer":
                guard let sdp = message.sdp, let type = message.signalType.flatMap(Self.sdpType(from:)) else { return }
                self.log("received \(message.signalType ?? "sdp")")
                let description = RTCSessionDescription(type: type, sdp: sdp)
                self.peerConnection?.setRemoteDescription(description) { [weak self] error in
                    guard let self else { return }
                    if let error {
                        self.publishStatus("WebRTC remote SDP failed: \(error.localizedDescription)")
                        return
                    }
                    self.flushPendingCandidates()
                    self.attachRemoteVideoTrackIfAvailable(reason: "remote SDP")
                    if type == .offer {
                        self.createAnswer()
                    }
                }
            case "candidate":
                guard let candidate = message.candidate,
                      let sdpMLineIndex = message.sdpMLineIndex else { return }
                let ice = RTCIceCandidate(sdp: candidate, sdpMLineIndex: sdpMLineIndex, sdpMid: message.sdpMid)
                if self.peerConnection?.remoteDescription == nil {
                    self.pendingCandidates.append(ice)
                    self.log("queued ICE candidate")
                } else {
                    self.peerConnection?.add(ice)
                    self.log("added ICE candidate")
                }
            default:
                break
            }
        }
    }

    func configureForLocalConnection(_ isLocal: Bool) {
        queue.async { [weak self] in
            guard let self else { return }
            let maxBps = isLocal ? 10_000_000 : 1_000_000
            let fps: Int32 = isLocal ? 20 : 10
            self.viewStreamer?.targetFps = fps
            guard let sender = self.peerConnection?.senders.first(where: { $0.track?.kind == "video" }) else { return }
            let params = sender.parameters
            for encoding in params.encodings {
                encoding.maxBitrateBps = NSNumber(value: maxBps)
            }
            sender.parameters = params
            self.log("configured for \(isLocal ? "local" : "remote"): \(maxBps / 1000)kbps max, \(fps)fps")
        }
    }

    func sendVideoFrame(_ image: UIImage) {
        queue.async { [weak self] in
            guard let self, let streamer = self.viewStreamer else {
                self?.log("dropped view frame before local video track was ready")
                return
            }
            streamer.send(image)
        }
    }

    private func createPeerConnectionIfNeeded() {
        guard peerConnection == nil else { return }
        let configuration = RTCConfiguration()
        configuration.sdpSemantics = .unifiedPlan
        configuration.continualGatheringPolicy = .gatherContinually
        configuration.iceServers = [
            RTCIceServer(urlStrings: ["stun:stun.l.google.com:19302"]),
            RTCIceServer(urlStrings: ["stun:stun1.l.google.com:19302"]),
            RTCIceServer(urlStrings: ["stun:stun.cloudflare.com:3478"])
        ]

        let constraints = RTCMediaConstraints(
            mandatoryConstraints: nil,
            optionalConstraints: ["DtlsSrtpKeyAgreement": "true"]
        )
        peerConnection = Self.factory.peerConnection(with: configuration, constraints: constraints, delegate: self)
        publishStatus(role == .host ? "WebRTC signaling ready" : "WebRTC connecting")
    }

    private func configureLocalVideoTrackIfNeeded() {
        guard videoTrack == nil, let peerConnection else { return }
        let source = Self.factory.videoSource()
        let streamer = RemoteRenderedViewStreamer(videoSource: source)
        streamer.onFrameSent = { [weak self] width, height, count in
            self?.onLocalFrameSent?(width, height, count)
        }
        let track = Self.factory.videoTrack(with: source, trackId: "robocar-view")
        track.isEnabled = true
        peerConnection.add(track, streamIds: ["robocar"])
        viewStreamer = streamer
        videoTrack = track
        log("configured local rendered-view video track")
    }

    private func configureRemoteVideoReceiverIfNeeded() {
        guard !hasVideoReceiver, let peerConnection else { return }
        let initOptions = RTCRtpTransceiverInit()
        initOptions.direction = .recvOnly
        peerConnection.addTransceiver(of: .video, init: initOptions)
        hasVideoReceiver = true
        log("configured remote video receiver")
    }

    private func attachRemoteVideoTrackIfAvailable(reason: String) {
        guard remoteVideoTrack == nil, let peerConnection else { return }
        for transceiver in peerConnection.transceivers {
            guard let track = transceiver.receiver.track as? RTCVideoTrack else { continue }
            attachRemoteVideoTrack(track, reason: reason)
            return
        }
        log("no remote video track available after \(reason)")
    }

    private func attachRemoteVideoTrack(_ track: RTCVideoTrack, reason: String) {
        guard remoteVideoTrack !== track else { return }
        track.isEnabled = true
        let renderer = RemoteVideoFrameObserver(onFrame: { [weak self] size, count in
            self?.onRemoteFrameRendered?(size, count)
        }, onImage: { [weak self] image in
            self?.onRemoteFrameImage?(image)
        })
        remoteFrameObserver = renderer
        remoteVideoTrack = track
        track.add(renderer)
        log("attached remote video track via \(reason)")
        DispatchQueue.main.async { [weak self] in
            self?.onVideoTrack?(track)
        }
    }

    private func createOffer() {
        let constraints = RTCMediaConstraints(
            mandatoryConstraints: ["OfferToReceiveVideo": "true"],
            optionalConstraints: nil
        )
        peerConnection?.offer(for: constraints) { [weak self] sdp, error in
            guard let self else { return }
            if let error {
                self.publishStatus("WebRTC offer failed: \(error.localizedDescription)")
                return
            }
            guard let sdp else { return }
            self.log("created offer")
            self.setLocalDescriptionAndSignal(sdp.preferH264VideoCodec())
        }
    }

    private func createAnswer() {
        let constraints = RTCMediaConstraints(mandatoryConstraints: nil, optionalConstraints: nil)
        peerConnection?.answer(for: constraints) { [weak self] sdp, error in
            guard let self else { return }
            if let error {
                self.publishStatus("WebRTC answer failed: \(error.localizedDescription)")
                return
            }
            guard let sdp else { return }
            self.log("created answer")
            self.setLocalDescriptionAndSignal(sdp.preferH264VideoCodec())
        }
    }

    private func flushPendingCandidates() {
        guard peerConnection?.remoteDescription != nil else { return }
        pendingCandidates.forEach { peerConnection?.add($0) }
        pendingCandidates.removeAll()
    }

    private func setLocalDescriptionAndSignal(_ sdp: RTCSessionDescription) {
        peerConnection?.setLocalDescription(sdp) { [weak self] error in
            guard let self else { return }
            if let error {
                self.publishStatus("WebRTC local SDP failed: \(error.localizedDescription)")
                return
            }
            var message = RemoteMessage(type: "webrtcSignal")
            message.signalType = Self.signalType(from: sdp.type)
            message.sdp = sdp.sdp
            self.log("sending \(message.signalType ?? "sdp")")
            self.signal(message)
        }
    }

    private func signal(_ message: RemoteMessage) {
        DispatchQueue.main.async { [weak self] in
            self?.onSignal?(message)
        }
    }

    private func publishStatus(_ status: String) {
        print("[RemoteWebRTC] \(status)")
        log(status)
        DispatchQueue.main.async { [weak self] in
            self?.onStatusChanged?(status)
        }
    }

    private func log(_ message: String) {
        print("[RemoteWebRTC] \(role == .host ? "host" : "client"): \(message)")
    }

    private static func signalType(from type: RTCSdpType) -> String {
        switch type {
        case .offer: return "offer"
        case .prAnswer: return "prAnswer"
        case .answer: return "answer"
        @unknown default: return "answer"
        }
    }

    private static func sdpType(from string: String) -> RTCSdpType? {
        switch string {
        case "offer": return .offer
        case "prAnswer": return .prAnswer
        case "answer": return .answer
        default: return nil
        }
    }
}

extension RemoteControlWebRTCSession: RTCPeerConnectionDelegate {
    func peerConnection(_ peerConnection: RTCPeerConnection, didChange stateChanged: RTCSignalingState) {}

    func peerConnection(_ peerConnection: RTCPeerConnection, didAdd stream: RTCMediaStream) {
        if let track = stream.videoTracks.first {
            attachRemoteVideoTrack(track, reason: "media stream callback")
        }
    }

    func peerConnection(_ peerConnection: RTCPeerConnection, didAdd receiver: RTCRtpReceiver, streams: [RTCMediaStream]) {
        guard let track = receiver.track as? RTCVideoTrack else { return }
        attachRemoteVideoTrack(track, reason: "receiver callback")
    }

    func peerConnection(_ peerConnection: RTCPeerConnection, didRemove stream: RTCMediaStream) {}

    func peerConnectionShouldNegotiate(_ peerConnection: RTCPeerConnection) {}

    func peerConnection(_ peerConnection: RTCPeerConnection, didChange newState: RTCIceConnectionState) {
        publishStatus("WebRTC ICE: \(newState.description)")
        if newState == .connected || newState == .completed {
            attachRemoteVideoTrackIfAvailable(reason: "ICE \(newState.description)")
        }
    }

    func peerConnection(_ peerConnection: RTCPeerConnection, didChange newState: RTCIceGatheringState) {}

    func peerConnection(_ peerConnection: RTCPeerConnection, didGenerate candidate: RTCIceCandidate) {
        log("generated ICE candidate type=\(candidate.candidateTypeDescription) mid=\(candidate.sdpMid ?? "nil")")
        var message = RemoteMessage(type: "webrtcSignal")
        message.signalType = "candidate"
        message.candidate = candidate.sdp
        message.sdpMid = candidate.sdpMid
        message.sdpMLineIndex = candidate.sdpMLineIndex
        log("sending ICE candidate")
        signal(message)
    }

    func peerConnection(_ peerConnection: RTCPeerConnection, didRemove candidates: [RTCIceCandidate]) {}

    func peerConnection(_ peerConnection: RTCPeerConnection, didOpen dataChannel: RTCDataChannel) {}
}

private final class RemoteRenderedViewStreamer {
    var onFrameSent: ((Int, Int, UInt64) -> Void)?
    var targetFps: Int32 = 10

    private let videoSource: RTCVideoSource
    private let capturer: RTCVideoCapturer
    private var pixelBufferPool: CVPixelBufferPool?
    private var poolSize = CGSize.zero
    private var frameCount: UInt64 = 0

    init(videoSource: RTCVideoSource) {
        self.videoSource = videoSource
        self.capturer = RTCVideoCapturer(delegate: videoSource)
    }

    func send(_ image: UIImage) {
        guard let pixelBuffer = makePixelBuffer(for: image) else { return }
        let width = CVPixelBufferGetWidth(pixelBuffer)
        let height = CVPixelBufferGetHeight(pixelBuffer)
        videoSource.adaptOutputFormat(toWidth: Int32(width), height: Int32(height), fps: targetFps)

        let buffer = RTCCVPixelBuffer(pixelBuffer: pixelBuffer)
        let timestampNs = Int64(Date().timeIntervalSince1970 * 1_000_000_000)
        let frame = RTCVideoFrame(buffer: buffer, rotation: ._0, timeStampNs: timestampNs)
        videoSource.capturer(capturer, didCapture: frame)
        frameCount += 1
        onFrameSent?(width, height, frameCount)
    }

    private func makePixelBuffer(for image: UIImage) -> CVPixelBuffer? {
        guard let cgImage = image.cgImage else { return nil }
        let width = cgImage.width
        let height = cgImage.height
        let size = CGSize(width: width, height: height)
        if pixelBufferPool == nil || poolSize != size {
            pixelBufferPool = Self.makePool(width: width, height: height)
            poolSize = size
        }

        var pixelBuffer: CVPixelBuffer?
        guard let pixelBufferPool else { return nil }
        let status = CVPixelBufferPoolCreatePixelBuffer(nil, pixelBufferPool, &pixelBuffer)
        guard status == kCVReturnSuccess, let pixelBuffer else { return nil }

        CVPixelBufferLockBaseAddress(pixelBuffer, [])
        defer { CVPixelBufferUnlockBaseAddress(pixelBuffer, []) }

        guard let context = CGContext(
            data: CVPixelBufferGetBaseAddress(pixelBuffer),
            width: width,
            height: height,
            bitsPerComponent: 8,
            bytesPerRow: CVPixelBufferGetBytesPerRow(pixelBuffer),
            space: CGColorSpaceCreateDeviceRGB(),
            bitmapInfo: CGImageAlphaInfo.premultipliedFirst.rawValue | CGBitmapInfo.byteOrder32Little.rawValue
        ) else { return nil }

        context.draw(cgImage, in: CGRect(x: 0, y: 0, width: width, height: height))
        return pixelBuffer
    }

    private static func makePool(width: Int, height: Int) -> CVPixelBufferPool? {
        let poolAttributes = [kCVPixelBufferPoolMinimumBufferCountKey: 3] as CFDictionary
        let bufferAttributes: [String: Any] = [
            kCVPixelBufferPixelFormatTypeKey as String: kCVPixelFormatType_32BGRA,
            kCVPixelBufferWidthKey as String: width,
            kCVPixelBufferHeightKey as String: height,
            kCVPixelBufferIOSurfacePropertiesKey as String: [:],
            kCVPixelBufferMetalCompatibilityKey as String: true,
            kCVPixelBufferCGImageCompatibilityKey as String: true,
            kCVPixelBufferCGBitmapContextCompatibilityKey as String: true
        ]
        var pool: CVPixelBufferPool?
        CVPixelBufferPoolCreate(kCFAllocatorDefault, poolAttributes, bufferAttributes as CFDictionary, &pool)
        return pool
    }
}

private final class RemoteVideoFrameObserver: NSObject, RTCVideoRenderer {
    private let onFrame: (CGSize, UInt64) -> Void
    private let onImage: (UIImage) -> Void
    private let ciContext = CIContext()
    private var frameCount: UInt64 = 0

    init(onFrame: @escaping (CGSize, UInt64) -> Void, onImage: @escaping (UIImage) -> Void) {
        self.onFrame = onFrame
        self.onImage = onImage
        super.init()
    }

    func setSize(_ size: CGSize) {}

    func renderFrame(_ frame: RTCVideoFrame?) {
        guard let frame else { return }
        frameCount += 1
        let size = CGSize(width: Int(frame.width), height: Int(frame.height))
        let image = makeImage(from: frame)
        DispatchQueue.main.async { [onFrame, frameCount] in
            onFrame(size, frameCount)
        }
        if let image {
            DispatchQueue.main.async { [onImage] in
                onImage(image)
            }
        }
    }

    private func makeImage(from frame: RTCVideoFrame) -> UIImage? {
        guard let cvBuffer = frame.buffer as? RTCCVPixelBuffer else { return nil }
        let ciImage = CIImage(cvPixelBuffer: cvBuffer.pixelBuffer)
        guard let cgImage = ciContext.createCGImage(ciImage, from: ciImage.extent) else { return nil }
        return UIImage(cgImage: cgImage)
    }
}

private extension RTCSessionDescription {
    func preferH264VideoCodec() -> RTCSessionDescription {
        RTCSessionDescription(type: type, sdp: sdp.preferH264VideoCodecInSDP())
    }
}

private extension String {
    func preferH264VideoCodecInSDP() -> String {
        let lines = components(separatedBy: "\r\n")
        guard let videoMLineIndex = lines.firstIndex(where: { $0.hasPrefix("m=video") }) else { return self }

        let h264Payloads = lines.compactMap { line -> String? in
            guard line.hasPrefix("a=rtpmap:"), line.localizedCaseInsensitiveContains("H264/") else { return nil }
            let afterPrefix = line.dropFirst("a=rtpmap:".count)
            return afterPrefix.split(separator: " ").first.map(String.init)
        }
        guard !h264Payloads.isEmpty else { return self }

        var parts = lines[videoMLineIndex].split(separator: " ").map(String.init)
        guard parts.count > 3 else { return self }
        let header = Array(parts.prefix(3))
        let payloads = Array(parts.dropFirst(3))
        let preferred = h264Payloads + payloads.filter { !h264Payloads.contains($0) }
        parts = header + preferred

        var updated = lines
        updated[videoMLineIndex] = parts.joined(separator: " ")
        return updated.joined(separator: "\r\n")
    }
}

private extension RTCIceCandidate {
    var candidateTypeDescription: String {
        let parts = sdp.split(separator: " ").map(String.init)
        if let typeIndex = parts.firstIndex(of: "typ"), parts.indices.contains(typeIndex + 1) {
            return parts[typeIndex + 1]
        }
        return "unknown"
    }
}



private extension RTCIceConnectionState {
    var description: String {
        switch self {
        case .new: return "new"
        case .checking: return "checking"
        case .connected: return "connected"
        case .completed: return "completed"
        case .failed: return "failed"
        case .disconnected: return "disconnected"
        case .closed: return "closed"
        case .count: return "count"
        @unknown default: return "unknown"
        }
    }
}
