//
//  RemoteControlClientService.swift
//  RoboCar
//

import Foundation
import UIKit

final class RemoteControlClientService: NSObject {
    var onStatusChanged: ((String) -> Void)?
    var onMessage: ((RemoteMessage) -> Void)?
    var onVideoFrameImage: ((UIImage) -> Void)?
    var onVideoFrameSize: ((CGSize) -> Void)?
    var onConnected: (() -> Void)?
    var onDisconnected: (() -> Void)?

        private let transport = RemoteControlIrohSession.shared
        private let video = RemoteVideoCodec()
        private var sequence: UInt64 = 0
        var isConnected: Bool { transport.role == .controller && transport.isConnected }

    override init() {
        super.init()
        NotificationCenter.default.addObserver(self, selector: #selector(statusChanged), name: .remotePeersChanged, object: nil)
    }

    func start(videoView: RemoteVideoView) {
        transport.start(role: .controller)
        video.view = videoView
        video.onFrame = { [weak self] size in self?.onVideoFrameSize?(size) }
        transport.onMessage = { [weak self] data in
            guard let message = try? JSONDecoder().decode(RemoteMessage.self, from: data) else { return }
            self?.onMessage?(message)
        }
        transport.onVideo = { [weak self] data in self?.video.display(data) }
        transport.onConnected = { [weak self] in self?.onConnected?() }
        transport.onDisconnected = { [weak self] in self?.video.reset(); self?.onDisconnected?() }
        transport.onNeedsKeyframe = nil
    }

    func connect(to peer: RemotePeer) { transport.connectToPeer(peer.id) }

    func connectSelectedHost() {
        Task {
            do {
                _ = try await transport.ready()
                if let id = transport.store?.selectedHostID, !transport.isConnected { transport.connectToPeer(id) }
            } catch { onStatusChanged?(error.localizedDescription) }
        }
    }

    func disconnect(sendStop: Bool = true) {
        transport.disconnect()
        video.reset()
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

    func sendNLCommand(_ text: String) {
        var message = RemoteMessage(type: "nlCommand")
        message.nlCommand = text
        send(message)
    }

    func sendStopNLCommand() {
        send(RemoteMessage(type: "stopNLCommand"))
    }

    func restartVideo() {
        guard isConnected else { return }
        video.reset()
        send(RemoteMessage(type: "restartVideo"))
    }

    func send(_ message: RemoteMessage) {
        guard isConnected else { return }
        var outbound = message
        outbound.seq = sequence
        outbound.ts = Date().timeIntervalSince1970
        sequence += 1
        guard let data = try? JSONEncoder().encode(outbound) else { return }
        transport.sendMessage(data)
    }

    @objc private func statusChanged() { onStatusChanged?(transport.status) }
}
