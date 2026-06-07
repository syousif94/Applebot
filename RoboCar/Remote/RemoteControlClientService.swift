//
//  RemoteControlClientService.swift
//  RoboCar
//

import Foundation
import Network
import WebRTC

final class RemoteControlClientService {
    var onStatusChanged: ((String) -> Void)?
    var onMessage: ((RemoteMessage) -> Void)?
    var onVideoTrack: ((RTCVideoTrack) -> Void)?
    var onVideoFrameImage: ((UIImage) -> Void)?

    private let queue = DispatchQueue(label: "com.robocar.remote.client", qos: .userInitiated)
    private let encoder = JSONEncoder()
    private let decoder = JSONDecoder()
    private var connection: NWConnection?
    private var receiveBuffer = ""
    private var seq: UInt64 = 0
    private var webRTCSession: RemoteControlWebRTCSession?
    private var isTCPConnected = false

    var isConnected: Bool { isTCPConnected }

    func connect(to endpoint: NWEndpoint) {
        disconnect(sendStop: false)
        let parameters = NWParameters.tcp
        parameters.includePeerToPeer = true
        let connection = NWConnection(to: endpoint, using: parameters)
        self.connection = connection
        connection.stateUpdateHandler = { [weak self] state in
            switch state {
            case .ready:
                self?.isTCPConnected = true
                self?.publishStatus("WebRTC signaling connected")
                self?.startWebRTC()
                self?.receive()
            case .waiting(let error):
                self?.publishStatus("Waiting: \(error.localizedDescription)")
            case .failed(let error):
                self?.publishStatus("Connection failed: \(error.localizedDescription)")
                self?.disconnect(sendStop: false)
            case .cancelled:
                self?.isTCPConnected = false
                self?.publishStatus("Disconnected")
            default:
                break
            }
        }
        connection.start(queue: queue)
    }

    func connect(host: String, port: UInt16 = RemoteControlProtocol.defaultPort) {
        guard let nwPort = NWEndpoint.Port(rawValue: port) else { return }
        connect(to: .hostPort(host: NWEndpoint.Host(host), port: nwPort))
    }

    func disconnect(sendStop: Bool = true) {
        if sendStop {
            send(RemoteMessage(type: "stopDrive"))
        }
        connection?.cancel()
        connection = nil
        isTCPConnected = false
        receiveBuffer = ""
        webRTCSession?.stop()
        webRTCSession = nil
    }

    func sendDrive(x: Float, y: Float) {
        var message = RemoteMessage(type: "drive")
        message.x = x
        message.y = y
        send(message)
    }

    func sendStopDrive() {
        send(RemoteMessage(type: "stopDrive"))
    }

    func send(_ message: RemoteMessage) {
        guard let connection else { return }
        queue.async { [weak self] in
            guard let self else { return }
            var outbound = message
            outbound.seq = self.seq
            outbound.ts = Date().timeIntervalSince1970
            self.seq += 1
            guard let data = try? self.encoder.encode(outbound) else { return }
            var framed = data
            framed.append(0x0A)
            connection.send(content: framed, completion: .contentProcessed { _ in })
        }
    }

    private func startWebRTC() {
        let session = RemoteControlWebRTCSession(role: .client)
        session.onSignal = { [weak self] message in
            self?.sendSignal(message)
        }
        session.onVideoTrack = { [weak self] track in
            self?.onVideoTrack?(track)
        }
        session.onRemoteFrameRendered = { [weak self] size, count in
            guard count == 1 || count % 30 == 0 else { return }
            self?.publishStatus("WebRTC received view frame #\(count) (\(Int(size.width))x\(Int(size.height)))")
        }
        session.onRemoteFrameImage = { [weak self] image in
            self?.onVideoFrameImage?(image)
        }
        session.onStatusChanged = { [weak self] status in
            self?.publishStatus(status)
        }
        webRTCSession = session
        session.start()
    }

    private func sendSignal(_ message: RemoteMessage) {
        queue.async { [weak self] in
            guard let self, let connection else { return }
            guard let data = try? self.encoder.encode(message) else { return }
            var framed = data
            framed.append(0x0A)
            connection.send(content: framed, completion: .contentProcessed { _ in })
        }
    }

    private func receive() {
        connection?.receive(minimumIncompleteLength: 1, maximumLength: 256 * 1024) { [weak self] data, _, isComplete, error in
            guard let self else { return }
            if let data, !data.isEmpty, let chunk = String(data: data, encoding: .utf8) {
                self.handle(chunk: chunk)
            }
            if isComplete || error != nil {
                self.disconnect(sendStop: false)
                return
            }
            self.receive()
        }
    }

    private func handle(chunk: String) {
        receiveBuffer += chunk
        while let newline = receiveBuffer.firstIndex(of: "\n") {
            let line = String(receiveBuffer[..<newline])
            receiveBuffer.removeSubrange(...newline)
            guard let data = line.data(using: .utf8), let message = try? decoder.decode(RemoteMessage.self, from: data) else { continue }
            if message.type == "webrtcSignal" {
                webRTCSession?.handleSignal(message)
                continue
            }
            DispatchQueue.main.async { [weak self] in
                self?.onMessage?(message)
            }
        }
    }

    private func publishStatus(_ status: String) {
        print("[RemoteClient] \(status)")
        DispatchQueue.main.async { [weak self] in
            self?.onStatusChanged?(status)
        }
    }
}
