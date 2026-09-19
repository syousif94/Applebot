import Foundation
import UIKit
import IrohLib
import OSLog

extension Notification.Name {
    static let remotePeersChanged = Notification.Name("remotePeersChanged")
}

@MainActor
final class RemoteControlIrohSession {
    static let shared = RemoteControlIrohSession()
    static let alpn = Data("robocar-remote/1".utf8)
    private static let logger = Logger(subsystem: "com.robocar.iroh", category: "pairing")

    struct Packet: Codable {
        var kind: String
        var payload: Data?
        var ticket: String?
        var name: String?
        var role: RemotePeerRole?
        var token: String?
        var lease: UInt64?
    }

    private(set) var store: RemotePeerStore?
    private(set) var role: RemotePeerRole = .controller
    private(set) var connectedPeer: RemotePeer?
    private(set) var status = "Not connected"
    private(set) var isRelay = false
    private(set) var invitation: RemotePairingInvitation?
    private(set) var sessionID = UUID()
    var onMessage: ((Data) -> Void)?
    var onVideo: ((Data) -> Void)?
    var onConnected: (() -> Void)?
    var onDisconnected: (() -> Void)?
    var onNeedsKeyframe: (() -> Void)?

    private var endpoint: Endpoint?
    private var starting: Task<Endpoint, Error>?
    private var accepting: Task<Void, Never>?
    private var connecting: Task<Void, Never>?
    private var connection: Connection?
    private var control: RemoteIrohWriter?
    private var telemetry: RemoteIrohWriter?
    private var video: RemoteIrohWriter?
    private var sessionTasks: [Task<Void, Never>] = []
    private var pending: [UUID: Connection] = [:]
    private var generation = UUID()
    private var invitationDeadline: TimeInterval = 0
    private var pairingAttempt = UUID()
    private var lastReceived = ProcessInfo.processInfo.systemUptime
    private var lease: UInt64 = 0
    private var waitingForKeyframe = true
    private var retryAllowed = false
    private var reconnectPeerID: String?
    private var revoked = Set<String>()
    private var handshakes = 0
    private var connectionAttempt = UUID()

    var isRunning: Bool { endpoint != nil || starting != nil }
    var isConnected: Bool { connectedPeer != nil }

    func start(role: RemotePeerRole) {
        if self.role != role { stop() }
        self.role = role
        Task {
            do { _ = try await ready() }
            catch { publish(error.localizedDescription) }
        }
    }

    func ready() async throws -> Endpoint {
        if let endpoint { return endpoint }
        if let starting { return try await starting.value }
        if store == nil { store = try RemotePeerStore() }
        guard let store else { throw RemotePairingError.notPaired }
        let epoch = generation
        let secret = store.secret
        let task = Task {
            try await Endpoint.bind(options: EndpointOptions(
                preset: presetN0(), secretKey: secret, alpns: [Self.alpn]))
        }
        starting = task
        publish("Starting Iroh")
        do {
            let bound = try await task.value
            guard epoch == generation else {
                try? await bound.close()
                throw RemotePairingError.cancelled
            }
            endpoint = bound
            starting = nil
            publish(role == .robot ? "No controller connected" : "Choose a paired robot")
            accepting = Task { [weak self] in
                while !Task.isCancelled, let incoming = await bound.acceptNext() {
                    guard let self, self.generation == epoch else { break }
                    Self.logger.info("Incoming Iroh connection received")
                    guard self.handshakes < 4 else { try? await incoming.refuse(); continue }
                    self.handshakes += 1
                    let identifier = UUID()
                    Task { [weak self] in
                        defer { self?.handshakes -= 1 }
                        do {
                            let accepted = try await incoming.accept().connect()
                            guard let self, self.generation == epoch else {
                                try? accepted.close(errorCode: 1, reason: Data())
                                return
                            }
                            guard self.pending.count < 4 else {
                                try? accepted.close(errorCode: 1, reason: Data("Busy".utf8))
                                return
                            }
                            self.pending[identifier] = accepted
                            let timeout = self.deadline(accepted, seconds: 45)
                            defer {
                                timeout.cancel()
                                self.pending[identifier] = nil
                                if self.connection !== accepted {
                                    try? accepted.close(errorCode: 0, reason: Data())
                                }
                            }
                            do {
                                try await self.accept(accepted, epoch: epoch)
                            } catch {
                                Self.logger.error("Incoming handshake failed: \(error.localizedDescription, privacy: .public)")
                                if self.generation == epoch {
                                    self.publish("Incoming connection failed: \(error.localizedDescription)")
                                }
                                try? accepted.close(errorCode: 1, reason: Data(error.localizedDescription.prefix(256).utf8))
                            }
                        } catch {
                            Self.logger.error("Accepting Iroh connection failed: \(error.localizedDescription, privacy: .public)")
                            if let failed = self?.pending.removeValue(forKey: identifier) {
                                try? failed.close(errorCode: 1, reason: Data("Handshake rejected".utf8))
                            }
                        }
                    }
                }
            }
            return bound
        } catch {
            if epoch == generation { starting = nil }
            throw error
        }
    }

    func stop() {
        Self.logger.info("Stopping Iroh endpoint")
        generation = UUID()
        cancelPairing()
        disconnect()
        accepting?.cancel()
        accepting = nil
        starting?.cancel()
        starting = nil
        let oldEndpoint = endpoint
        endpoint = nil
        Task { try? await oldEndpoint?.close() }
        publish("Remote connection stopped")
    }

    func showInvitation() async throws -> String {
        cancelPairing()
        let attempt = pairingAttempt
        let epoch = generation
        let endpoint = try await ready()
        publish("Preparing pairing connection")
        try await Self.withPairingTimeout {
            await endpoint.online()
        }
        guard epoch == generation, attempt == pairingAttempt, !Task.isCancelled else {
            throw RemotePairingError.cancelled
        }
        let value = try RemotePairingInvitation(
            ticket: EndpointTicket.fromAddr(addr: endpoint.addr()).description,
            name: UIDevice.current.name, role: role)
        invitation = value
        invitationDeadline = ProcessInfo.processInfo.systemUptime + 120
        return try value.encoded
    }

    func cancelPairing() {
        invitation = nil
        invitationDeadline = 0
        pairingAttempt = UUID()
        for pendingConnection in pending.values where pendingConnection !== connection {
            Self.logger.info("Closing pending connection: pairing cancelled")
            try? pendingConnection.close(errorCode: 1, reason: Data("Cancelled".utf8))
        }
        pending.removeAll()
    }

    func pair(_ text: String) async throws {
        let endpoint = try await ready()
        let invite = try RemotePairingInvitation.decode(text, localID: endpoint.id().description, role: role)
        cancelPairing()
        let attempt = pairingAttempt
        let epoch = generation
        let identifier = UUID()
        Self.logger.info("Dialing scanned pairing invitation")
        publish("Connecting to \(invite.name)")
        let address = try EndpointTicket.fromString(str: invite.ticket).endpointAddr()
        let peerConnection = try await Self.withPairingTimeout {
            let connected = try await endpoint.connect(addr: address, alpn: Self.alpn)
            guard !Task.isCancelled else {
                try? connected.close(errorCode: 1, reason: Data("Cancelled".utf8))
                throw CancellationError()
            }
            return connected
        }
        guard epoch == generation, attempt == pairingAttempt, !Task.isCancelled else {
            try? peerConnection.close(errorCode: 1, reason: Data())
            throw RemotePairingError.cancelled
        }
        pending[identifier] = peerConnection
        let timeout = deadline(peerConnection, seconds: 45)
        defer {
            timeout.cancel()
            pending[identifier] = nil
            try? peerConnection.close(errorCode: 0, reason: Data())
        }
        let stream = try await peerConnection.openBi()
        try await Self.write(hello("pair", token: invite.token), to: stream.send())
        Self.logger.info("Pairing request sent; waiting for QR authorization")
        publish("Pairing with \(invite.name)")
        let response = try await Self.read(from: stream.recv())
        guard response.kind == "approved", attempt == pairingAttempt, epoch == generation else {
            throw RemotePairingError.cancelled
        }
        var peer = try validatedPeer(response, connection: peerConnection)
        guard peer.id == (try EndpointTicket.fromString(str: invite.ticket)).endpointAddr().id().description else {
            throw RemotePairingError.invalidInvitation
        }
        peer.confirmed = false
        try store?.remember(peer)
        try await Self.write(Packet(kind: "commit"), to: stream.send())
        let committed = try await Self.read(from: stream.recv())
        guard committed.kind == "paired", attempt == pairingAttempt, epoch == generation else {
            throw RemotePairingError.cancelled
        }
        try await Self.write(Packet(kind: "saved"), to: stream.send())
        let complete = try await Self.read(from: stream.recv())
        guard complete.kind == "complete", attempt == pairingAttempt, epoch == generation else {
            throw RemotePairingError.cancelled
        }
        try await Self.acknowledgePairingCompletion(send: stream.send(), receive: stream.recv())
        guard attempt == pairingAttempt, epoch == generation, !Task.isCancelled else {
            throw RemotePairingError.cancelled
        }
        peer.confirmed = true
        try store?.remember(peer)
        revoked.remove(peer.id)
        publish("Paired with \(peer.name)")
        if role == .controller { connectToPeer(peer.id) }
    }

    func removePeer(_ id: String) throws {
        try store?.remove(id)
        revoked.insert(id)
        cancelPairing()
        if connectedPeer?.id == id || reconnectPeerID == id { disconnect(reason: "Pairing removed") }
        publish("Pairing removed")
    }

    func connectToPeer(_ id: String) {
        guard !(retryAllowed && reconnectPeerID == id) else { return }
        disconnect()
        retryAllowed = true
        reconnectPeerID = id
        let attempt = connectionAttempt
        connecting = Task { [weak self] in
            var delay: UInt64 = 1
            while let self, !Task.isCancelled, self.retryAllowed {
                do {
                    let endpoint = try await self.ready()
                      guard !Task.isCancelled, attempt == self.connectionAttempt,
                          self.role == .controller, let peer = self.store?.peer(id),
                          peer.role == .robot, !self.revoked.contains(id) else { throw RemotePairingError.notPaired }
                    let epoch = self.generation
                    self.publish("Connecting to \(peer.name)")
                    let address = try EndpointId.fromString(s: id)
                    let peerConnection = try await Self.withPairingTimeout {
                        let connected = try await endpoint.connect(addr: EndpointAddr(id: address, relayUrl: nil, addresses: []), alpn: Self.alpn)
                        guard !Task.isCancelled else {
                            try? connected.close(errorCode: 1, reason: Data("Cancelled".utf8))
                            throw CancellationError()
                        }
                        return connected
                    }
                    guard epoch == self.generation, attempt == self.connectionAttempt, self.retryAllowed, !Task.isCancelled else {
                        try? peerConnection.close(errorCode: 1, reason: Data())
                        return
                    }
                    let identifier = UUID()
                    self.pending[identifier] = peerConnection
                    let timeout = self.deadline(peerConnection, seconds: 10)
                    defer { timeout.cancel(); self.pending[identifier] = nil }
                    do {
                        let stream = try await peerConnection.openBi()
                        try await Self.write(self.hello("resume"), to: stream.send())
                        let response = try await Self.read(from: stream.recv())
                        if response.kind == "busy" { throw RemotePairingError.busy }
                        if response.kind == "notPaired" { throw RemotePairingError.notPaired }
                        let verified = try self.validatedPeer(response, connection: peerConnection)
                        guard response.kind == "welcome", peerConnection.remoteId().description == id,
                            verified.role == .robot,
                              self.store?.peer(id) != nil, !self.revoked.contains(id), epoch == self.generation,
                            attempt == self.connectionAttempt, self.retryAllowed, !Task.isCancelled else { throw RemotePairingError.notPaired }
                        try await Self.write(Packet(kind: "ready"), to: stream.send())
                        guard attempt == self.connectionAttempt, epoch == self.generation,
                            self.store?.peer(id) != nil, !self.revoked.contains(id), !Task.isCancelled else {
                            throw RemotePairingError.cancelled
                        }
                        var refreshed = peer
                        refreshed.name = verified.name
                        refreshed.ticket = verified.ticket
                        try self.store?.remember(refreshed)
                        try self.store?.selectHost(id)
                        self.activate(peerConnection, stream: stream, peer: refreshed)
                        return
                    } catch {
                        try? peerConnection.close(errorCode: 1, reason: Data())
                        throw error
                    }
                } catch let error as RemotePairingError {
                    guard attempt == self.connectionAttempt else { return }
                    if case .busy = error {
                        self.publish("Robot busy. Reconnecting...")
                        try? await Task.sleep(nanoseconds: delay * 1_000_000_000)
                        delay = min(delay * 2, 15)
                    } else {
                        self.retryAllowed = false
                        self.publish(error.localizedDescription)
                        return
                    }
                } catch {
                    guard !Task.isCancelled, attempt == self.connectionAttempt, self.retryAllowed else { return }
                    self.publish("Robot unavailable. Reconnecting...")
                    try? await Task.sleep(nanoseconds: delay * 1_000_000_000)
                    delay = min(delay * 2, 15)
                }
            }
        }
    }

    func disconnect(reason: String = "Disconnected") {
        sessionID = UUID()
        connectionAttempt = UUID()
        retryAllowed = false
        reconnectPeerID = nil
        connecting?.cancel()
        connecting = nil
        for pendingConnection in pending.values {
            Self.logger.info("Closing pending connection: \(reason, privacy: .public)")
            try? pendingConnection.close(errorCode: 0, reason: Data(reason.utf8))
        }
        pending.removeAll()
        let wasConnected = connection != nil
        let old = connection
        connection = nil
        connectedPeer = nil
        sessionTasks.forEach { $0.cancel() }
        sessionTasks.removeAll()
        control?.cancel(); telemetry?.cancel(); video?.cancel()
        control = nil; telemetry = nil; video = nil
        try? old?.close(errorCode: 0, reason: Data(reason.utf8))
        lease = 0
        waitingForKeyframe = true
        if wasConnected { onDisconnected?() }
        publish(reason)
    }

    func sendMessage(_ data: Data) {
        guard isConnected else { return }
        let packet = Packet(kind: "message", payload: data, lease: lease)
        if role == .robot { telemetry?.enqueue(packet) }
        else { control?.enqueue(packet) }
    }

    func sendVideo(_ data: Data, keyframe: Bool) {
        guard isConnected, let video else { return }
        if waitingForKeyframe && !keyframe { onNeedsKeyframe?(); return }
        if video.queuedBytes > 512 * 1024 {
            video.dropQueued()
            waitingForKeyframe = true
            onNeedsKeyframe?()
            if !keyframe { return }
        }
        if keyframe { waitingForKeyframe = false }
        video.enqueue(Packet(kind: "video", payload: data))
    }

    private func accept(_ incoming: Connection, epoch: UUID) async throws {
        Self.logger.info("Incoming transport handshake established; waiting for request stream")
        let stream = try await incoming.acceptBi()
        let request = try await Self.read(from: stream.recv())
        Self.logger.info("Incoming request decoded: \(request.kind, privacy: .public)")
        var peer = try validatedPeer(request, connection: incoming)
        if request.kind == "pair" {
            let attempt = pairingAttempt
            guard let invitation, request.token == invitation.token,
                  ProcessInfo.processInfo.systemUptime < invitationDeadline else {
                throw RemotePairingError.expiredInvitation
            }
            self.invitation = nil
            invitationDeadline = 0
            publish("Pairing with \(peer.name)")
            Self.logger.info("Pairing authorized by active QR invitation")
            guard attempt == pairingAttempt, epoch == generation else { throw RemotePairingError.cancelled }
            try await Self.write(hello("approved"), to: stream.send())
            let commit = try await Self.read(from: stream.recv())
            guard commit.kind == "commit", attempt == pairingAttempt, epoch == generation else {
                throw RemotePairingError.cancelled
            }
            peer.confirmed = false
            try store?.remember(peer)
            try await Self.write(Packet(kind: "paired"), to: stream.send())
            let saved = try await Self.read(from: stream.recv())
            guard saved.kind == "saved", attempt == pairingAttempt, epoch == generation else {
                throw RemotePairingError.cancelled
            }
            peer.confirmed = true
            try store?.remember(peer)
            revoked.remove(peer.id)
            try await Self.sendPairingCompletion(send: stream.send(), receive: stream.recv())
            guard attempt == pairingAttempt, epoch == generation else { throw RemotePairingError.cancelled }
            publish("Paired with \(peer.name)")
            _ = await incoming.closed()
            if role == .controller, epoch == generation, attempt == pairingAttempt,
               store?.peer(peer.id) != nil, !revoked.contains(peer.id) {
                connectToPeer(peer.id)
            }
            return
        }
        guard request.kind == "resume", role == .robot, peer.role == .controller,
              let trusted = store?.peer(peer.id), trusted.role == .controller,
              !revoked.contains(peer.id), epoch == generation else {
            try await Self.write(Packet(kind: "notPaired"), to: stream.send())
            throw RemotePairingError.notPaired
        }
        guard connection == nil else {
            try await Self.write(Packet(kind: "busy"), to: stream.send())
            throw RemotePairingError.busy
        }
        connection = incoming
        do {
            try await Self.write(hello("welcome"), to: stream.send())
            let ready = try await Self.read(from: stream.recv())
            guard ready.kind == "ready", connection === incoming, store?.peer(peer.id) != nil,
                  !revoked.contains(peer.id), epoch == generation else { throw RemotePairingError.notPaired }
            var refreshed = trusted
            refreshed.name = peer.name
            refreshed.ticket = peer.ticket
            try store?.remember(refreshed)
            activate(incoming, stream: stream, peer: refreshed)
        } catch {
            if connection === incoming { disconnect(reason: "Connection failed") }
            throw error
        }
    }

    private func hello(_ kind: String, token: String? = nil) throws -> Packet {
        guard let endpoint else { throw RemotePairingError.cancelled }
        return Packet(kind: kind, ticket: try EndpointTicket.fromAddr(addr: endpoint.addr()).description,
                      name: String(UIDevice.current.name.prefix(32)), role: role, token: token)
    }

    private func validatedPeer(_ packet: Packet, connection: Connection) throws -> RemotePeer {
        guard let ticket = packet.ticket, let name = packet.name, let peerRole = packet.role,
              peerRole != role, !name.isEmpty, name.utf8.count <= 128 else {
            throw RemotePairingError.invalidInvitation
        }
        let identifier = connection.remoteId().description
        guard try EndpointTicket.fromString(str: ticket).endpointAddr().id().description == identifier else {
            throw RemotePairingError.invalidInvitation
        }
        return RemotePeer(id: identifier, name: name, role: peerRole, ticket: ticket, pairedAt: Date())
    }

    private func activate(_ active: Connection, stream: BiStream, peer: RemotePeer) {
        sessionID = UUID()
        connection = active
        connectedPeer = peer
        lastReceived = ProcessInfo.processInfo.systemUptime
        let fail: () -> Void = { [weak self] in
            guard let self, self.connection === active else { return }
            let reason = active.closeReason() ?? "Connection lost"
            let removed = reason.contains("Pairing removed")
            self.lost(active, reason: removed ? "Pairing removed. Scan again." : "Connection lost", reconnect: !removed)
        }
        control = RemoteIrohWriter(stream: stream.send(), limit: 128 * 1024, failed: fail)
        sessionTasks.append(Task { [weak self] in
            do { try await self?.readSession(stream.recv(), connection: active, channel: "control") }
            catch { fail() }
        })
        sessionTasks.append(Task { [weak self] in
            guard let self else { return }
            do {
                if self.role == .robot {
                    let stateStream = try await active.openUni()
                    try await Self.write(Packet(kind: "telemetry"), to: stateStream)
                    let videoStream = try await active.openUni()
                    try await Self.write(Packet(kind: "video"), to: videoStream)
                    guard self.connection === active else { return }
                    self.telemetry = RemoteIrohWriter(stream: stateStream, limit: 8 * 1024 * 1024, failed: fail)
                    self.video = RemoteIrohWriter(stream: videoStream, limit: 4 * 1024 * 1024, failed: fail)
                    self.onConnected?()
                } else {
                    self.onConnected?()
                    var channels = Set<String>()
                    for _ in 0..<2 {
                        let incoming = try await active.acceptUni()
                        let header = try await Self.read(from: incoming)
                        guard ["telemetry", "video"].contains(header.kind), channels.insert(header.kind).inserted,
                              self.connection === active else { throw RemotePairingError.cancelled }
                        self.sessionTasks.append(Task {
                            do { try await self.readSession(incoming, connection: active, channel: header.kind) }
                            catch { fail() }
                        })
                    }
                }
            } catch { fail() }
        })
        sessionTasks.append(Task { [weak self] in
            while !Task.isCancelled {
                guard let self, self.connection === active else { return }
                let now = ProcessInfo.processInfo.systemUptime
                if now - self.lastReceived > 3 { fail(); return }
                if self.role == .robot { self.lease = UInt64(now * 1000) }
                self.control?.enqueue(Packet(kind: "heartbeat", lease: self.lease))
                let relay = active.paths().first { $0.isSelected }?.isRelay ?? false
                if self.isRelay != relay { self.isRelay = relay; self.notify() }
                do { try await Task.sleep(nanoseconds: 250_000_000) } catch { return }
            }
        })
        sessionTasks.append(Task { [weak self] in
            let reason = await active.closed()
            guard let self, self.connection === active else { return }
            let removed = reason.contains("Pairing removed")
            let explicit = reason.contains("Disconnected") || removed || reason.contains("Remote connection stopped")
            self.lost(active, reason: removed ? "Pairing removed. Scan again." : "Disconnected", reconnect: !explicit)
        })
        publish(role == .robot ? "Controller connected: \(peer.name)" : "Connected to \(peer.name)")
    }

    private func readSession(_ stream: RecvStream, connection active: Connection, channel: String) async throws {
        while !Task.isCancelled {
            let packet = try await Self.read(from: stream, limit: channel == "control" ? 128 * 1024 : 8 * 1024 * 1024)
            guard connection === active, let connectedPeer, store?.peer(connectedPeer.id) != nil,
                  !revoked.contains(connectedPeer.id) else { throw RemotePairingError.notPaired }
            if channel == "control" { lastReceived = ProcessInfo.processInfo.systemUptime }
            switch (channel, packet.kind) {
            case ("control", "heartbeat"):
                if role == .controller { lease = packet.lease ?? 0 }
            case ("control", "message") where role == .robot:
                guard let issued = packet.lease else { continue }
                let now = UInt64(ProcessInfo.processInfo.systemUptime * 1000)
                guard issued <= now, now - issued < 750 else { continue }
                if let data = packet.payload { onMessage?(data) }
            case ("telemetry", "message") where role == .controller:
                if let data = packet.payload { onMessage?(data) }
            case ("video", "video") where role == .controller:
                if let data = packet.payload { onVideo?(data) }
            default: throw RemotePairingError.invalidInvitation
            }
        }
    }

    private func lost(_ active: Connection, reason: String, reconnect: Bool) {
        guard connection === active else { return }
        let peer = connectedPeer
        let canRetry = reconnect && role == .controller && retryAllowed
        disconnect(reason: reason)
        if canRetry, let peer, store?.peer(peer.id) != nil, !revoked.contains(peer.id) {
            let attempt = connectionAttempt
            connecting = Task { [weak self] in
                do { try await Task.sleep(nanoseconds: 1_000_000_000) } catch { return }
                guard let self, self.connectionAttempt == attempt else { return }
                self.connectToPeer(peer.id)
            }
        }
    }

    enum PairingTimeout: LocalizedError {
        case elapsed

        var errorDescription: String? {
            "Pairing connection timed out. Keep both devices online and scan a fresh QR code."
        }
    }

    static func withPairingTimeout<Value: Sendable>(
        nanoseconds: UInt64 = 15_000_000_000,
        operation: @escaping @Sendable () async throws -> Value
    ) async throws -> Value {
        let waiter = PairingWaiter<Value>()
        return try await withTaskCancellationHandler {
            try Task.checkCancellation()
            return try await withCheckedThrowingContinuation { continuation in
                waiter.continuation = continuation
                waiter.operation = Task {
                    do { waiter.finish(.success(try await operation())) }
                    catch { waiter.finish(.failure(error)) }
                }
                waiter.timeout = Task {
                    do { try await Task.sleep(nanoseconds: nanoseconds) }
                    catch { return }
                    waiter.finish(.failure(PairingTimeout.elapsed))
                }
            }
        } onCancel: {
            Task { @MainActor in waiter.finish(.failure(CancellationError())) }
        }
    }

    private func deadline(_ connection: Connection, seconds: UInt64) -> Task<Void, Never> {
        Task {
            do { try await Task.sleep(nanoseconds: seconds * 1_000_000_000) }
            catch { return }
            Self.logger.error("Closing connection: handshake timed out after \(seconds) seconds")
            try? connection.close(errorCode: 1, reason: Data("Handshake timed out".utf8))
        }
    }

    private func publish(_ text: String) { status = text; notify() }
    private func notify() { NotificationCenter.default.post(name: .remotePeersChanged, object: self) }

    static func frame(_ packet: Packet) throws -> Data {
        let payload = try JSONEncoder().encode(packet)
        guard payload.count <= 8 * 1024 * 1024 else { throw RemotePairingError.invalidInvitation }
        var length = UInt32(payload.count).bigEndian
        var data = withUnsafeBytes(of: &length) { Data($0) }
        data.append(payload)
        return data
    }

    static func write(_ packet: Packet, to stream: SendStream) async throws {
        try await stream.writeAll(buf: frame(packet))
    }

    static func sendPairingCompletion(send: SendStream, receive: RecvStream) async throws {
        try await write(Packet(kind: "complete"), to: send)
        let receipt = try await read(from: receive)
        guard receipt.kind == "receipt" else { throw RemotePairingError.invalidInvitation }
        try await send.finish()
    }

    static func acknowledgePairingCompletion(send: SendStream, receive: RecvStream) async throws {
        try await write(Packet(kind: "receipt"), to: send)
        let remaining = try await receive.read(sizeLimit: 1)
        guard remaining.isEmpty else { throw RemotePairingError.invalidInvitation }
    }

    static func read(from stream: RecvStream, limit: Int = 16384) async throws -> Packet {
        let header = try await stream.readExact(size: 4)
        let length = header.reduce(UInt32(0)) { ($0 << 8) | UInt32($1) }
        guard length > 0, length <= limit else { throw RemotePairingError.invalidInvitation }
        return try JSONDecoder().decode(Packet.self, from: await stream.readExact(size: length))
    }
}

@MainActor
private final class PairingWaiter<Value: Sendable> {
    var continuation: CheckedContinuation<Value, Error>?
    var operation: Task<Void, Never>?
    var timeout: Task<Void, Never>?

    func finish(_ result: Result<Value, Error>) {
        guard let continuation else { return }
        self.continuation = nil
        operation?.cancel()
        timeout?.cancel()
        operation = nil
        timeout = nil
        continuation.resume(with: result)
    }
}

@MainActor
private final class RemoteIrohWriter {
    private let stream: SendStream
    private let limit: Int
    private let failed: () -> Void
    private var queue: [Data] = []
    private var task: Task<Void, Never>?
    private(set) var queuedBytes = 0

    init(stream: SendStream, limit: Int, failed: @escaping () -> Void) {
        self.stream = stream
        self.limit = limit
        self.failed = failed
    }

    func enqueue(_ packet: RemoteControlIrohSession.Packet) {
        guard let bytes = try? RemoteControlIrohSession.frame(packet), queuedBytes + bytes.count <= limit else {
            failed()
            return
        }
        queue.append(bytes)
        queuedBytes += bytes.count
        guard task == nil else { return }
        task = Task { [weak self] in
            guard let self else { return }
            defer { self.task = nil }
            while !self.queue.isEmpty, !Task.isCancelled {
                let data = self.queue.removeFirst()
                self.queuedBytes -= data.count
                let timeout = Task {
                    do { try await Task.sleep(nanoseconds: 2_000_000_000) } catch { return }
                    self.failed()
                }
                do { try await self.stream.writeAll(buf: data); timeout.cancel() }
                catch { timeout.cancel(); self.failed(); return }
            }
        }
    }

    func dropQueued() { queue.removeAll(); queuedBytes = 0 }
    func cancel() { task?.cancel(); task = nil; dropQueued() }
}