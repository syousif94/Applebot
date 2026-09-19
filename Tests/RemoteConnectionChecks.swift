import Foundation
import Security
import IrohLib

@main
struct RemoteConnectionChecks {
    @MainActor
    static func main() async throws {
        let relayOnly = CommandLine.arguments.contains("--relay-only")
        let secret = SecretKey.generate().toBytes()
        let host = try await Endpoint.bind(options: EndpointOptions(preset: relayOnly ? presetN0() : presetMinimal(), secretKey: secret,
                                                                    alpns: [RemoteControlIrohSession.alpn]))
        let controller = try await Endpoint.bind(options: EndpointOptions(preset: relayOnly ? presetN0() : presetMinimal()))
        if relayOnly {
            try await RemoteControlIrohSession.withPairingTimeout { await host.online() }
            print("PASS: host relay ready")
        }
        let timeoutStarted = ProcessInfo.processInfo.systemUptime
        do {
            try await RemoteControlIrohSession.withPairingTimeout(nanoseconds: 20_000_000) {
                await withCheckedContinuation { continuation in
                    DispatchQueue.main.asyncAfter(deadline: .now() + 0.5) { continuation.resume() }
                }
            }
            fatalError("Uncancellable operation did not time out")
        } catch RemoteControlIrohSession.PairingTimeout.elapsed {}
        precondition(ProcessInfo.processInfo.systemUptime - timeoutStarted < 0.4)
        let success = try await RemoteControlIrohSession.withPairingTimeout { 42 }
        precondition(success == 42)
        print("PASS: pairing deadline returns without waiting for uncancellable work")
        let ticket = try EndpointTicket.fromAddr(addr: host.addr()).description
        let invitation = try RemotePairingInvitation(ticket: ticket, name: "Test robot", role: .robot)
        let parsed = try RemotePairingInvitation.decode(invitation.encoded, localID: controller.id().description, role: .controller)
        precondition(parsed.ticket == ticket && parsed.token == invitation.token)
        do {
            _ = try RemotePairingInvitation.decode(invitation.encoded, localID: host.id().description, role: .controller)
            fatalError("Self pairing accepted")
        } catch {}
        do {
            _ = try RemotePairingInvitation.decode(invitation.encoded, localID: controller.id().description,
                                                   role: .controller, now: Date().addingTimeInterval(200))
            fatalError("Expired invitation accepted")
        } catch {}
        do {
            _ = try RemotePairingInvitation.decode(invitation.encoded, localID: controller.id().description, role: .robot)
            fatalError("Same-role pairing accepted")
        } catch {}

        let service = "com.robocar.iroh.test.\(UUID().uuidString)"
        defer {
            SecItemDelete([kSecClass as String: kSecClassGenericPassword,
                           kSecAttrService as String: service] as CFDictionary)
        }
        do {
            let store = try RemotePeerStore(service: service)
            let peer = RemotePeer(id: host.id().description, name: "Test robot", role: .robot, ticket: ticket, pairedAt: Date())
            try store.remember(peer)
            precondition(store.selectedHostID == nil)
            try store.selectHost(peer.id)
            let restored = try RemotePeerStore(service: service)
            precondition(restored.secret == store.secret && restored.peer(peer.id) == peer)
            precondition(restored.selectedHostID == peer.id)
            let otherPeer = RemotePeer(id: controller.id().description, name: "Other robot", role: .robot,
                                       ticket: try EndpointTicket.fromAddr(addr: controller.addr()).description, pairedAt: Date())
            try restored.remember(otherPeer)
            precondition(restored.selectedHostID == peer.id)
            try restored.remove(otherPeer.id)
            precondition(restored.selectedHostID == peer.id)
            var renamed = peer
            renamed.name = "Renamed robot"
            try restored.remember(renamed)
            precondition(restored.peer(peer.id)?.name == renamed.name && restored.selectedHostID == peer.id)
            try restored.remove(peer.id)
            let removed = try RemotePeerStore(service: service)
            precondition(removed.secret == store.secret && removed.peers.isEmpty && removed.selectedHostID == nil)
            print("PASS: Keychain last-connected selection, name refresh, persistence and removal")
        } catch RemotePairingError.storage(errSecMissingEntitlement) {
            print("SKIP: Keychain checks require a provisioned Catalyst app (-34018)")
        }

        let server = Task {
            guard let incoming = await host.acceptNext() else { fatalError("Missing incoming connection") }
            let connection = try await incoming.accept().connect()
            precondition(connection.remoteId() == controller.id())
            let stream = try await connection.acceptBi()
            let packet = try await RemoteControlIrohSession.read(from: stream.recv())
            precondition(packet.kind == "test" && packet.payload == Data("hello".utf8))
            try await RemoteControlIrohSession.write(packet, to: stream.send())
            try await RemoteControlIrohSession.sendPairingCompletion(send: stream.send(), receive: stream.recv())
            _ = await connection.closed()
        }
        let dialAddress = relayOnly
            ? EndpointAddr(id: host.id(), relayUrl: host.addr().relayUrl(), addresses: [])
            : host.addr()
        if relayOnly { precondition(dialAddress.relayUrl() != nil) }
        let connection = try await RemoteControlIrohSession.withPairingTimeout {
            try await controller.connect(addr: dialAddress, alpn: RemoteControlIrohSession.alpn)
        }
        if relayOnly { print("PASS: relay-only connection established") }
        precondition(connection.remoteId() == host.id())
        let stream = try await connection.openBi()
        try await RemoteControlIrohSession.write(.init(kind: "test", payload: Data("hello".utf8)), to: stream.send())
        let echoed = try await RemoteControlIrohSession.read(from: stream.recv())
        precondition(echoed.kind == "test" && echoed.payload == Data("hello".utf8))
        let complete = try await RemoteControlIrohSession.read(from: stream.recv())
        precondition(complete.kind == "complete")
        try await RemoteControlIrohSession.acknowledgePairingCompletion(send: stream.send(), receive: stream.recv())
        print("PASS: pairing completion is acknowledged before connection closes")
        try connection.close(errorCode: 0, reason: Data())
        try await server.value
        let originalID = host.id()
        try await host.close()
        let restarted = try await Endpoint.bind(options: EndpointOptions(preset: presetMinimal(), secretKey: secret))
        precondition(restarted.id() == originalID)
        try await restarted.close()
        try await controller.close()
        print("PASS: QR validation, stable identity, authenticated Iroh framing round trip")
    }
}