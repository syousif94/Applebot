import Foundation
import Security
import IrohLib

enum RemotePeerRole: String, Codable {
    case robot, controller
}

struct RemotePeer: Codable, Equatable, Identifiable {
    let id: String
    var name: String
    let role: RemotePeerRole
    var ticket: String
    let pairedAt: Date
    var confirmed = true
}

enum RemotePairingError: LocalizedError {
    case invalidInvitation, expiredInvitation, notPaired, busy, cancelled, storage(OSStatus)

    var errorDescription: String? {
        switch self {
        case .invalidInvitation: return "Invalid device invitation."
        case .expiredInvitation: return "Pairing invitation expired. Show a new QR code."
        case .notPaired: return "Device is not paired. Scan a new QR code."
        case .busy: return "A controller is already connected."
        case .cancelled: return "Connection cancelled."
        case .storage: return "Could not access secure device storage."
        }
    }
}

@MainActor
final class RemotePeerStore {
    private struct Record: Codable {
        var secret: Data
        var peers: [RemotePeer]
        var selectedHostID: String?
    }

    private let service: String
    private var record: Record
    var peers: [RemotePeer] { record.peers.filter(\.confirmed) }
    var selectedHostID: String? { record.selectedHostID }
    var secret: Data { record.secret }

    init(service: String = "com.robocar.iroh.identity") throws {
        self.service = service
        var query = Self.query(service: service)
        query[kSecReturnData as String] = true
        query[kSecMatchLimit as String] = kSecMatchLimitOne
        var result: CFTypeRef?
        let status = SecItemCopyMatching(query as CFDictionary, &result)
        if status == errSecSuccess, let data = result as? Data {
            record = try JSONDecoder().decode(Record.self, from: data)
            _ = try SecretKey.fromBytes(bytes: record.secret)
        } else if status == errSecItemNotFound {
            record = Record(secret: SecretKey.generate().toBytes(), peers: [])
            var attributes = Self.query(service: service)
            attributes[kSecValueData as String] = try JSONEncoder().encode(record)
            attributes[kSecAttrAccessible as String] = kSecAttrAccessibleAfterFirstUnlockThisDeviceOnly
            let added = SecItemAdd(attributes as CFDictionary, nil)
            guard added == errSecSuccess else { throw RemotePairingError.storage(added) }
        } else {
            throw RemotePairingError.storage(status)
        }
    }

    func peer(_ id: String) -> RemotePeer? { peers.first { $0.id == id } }

    func remember(_ peer: RemotePeer) throws {
        let ticket = try EndpointTicket.fromString(str: peer.ticket)
        guard ticket.endpointAddr().id().description == peer.id,
              !peer.name.isEmpty, peer.name.utf8.count <= 128 else {
            throw RemotePairingError.invalidInvitation
        }
        var updated = record
        updated.peers.removeAll { $0.id == peer.id }
        updated.peers.append(peer)
        try save(updated)
    }

    func remove(_ id: String) throws {
        var updated = record
        updated.peers.removeAll { $0.id == id }
        if updated.selectedHostID == id { updated.selectedHostID = nil }
        try save(updated)
    }

    func selectHost(_ id: String) throws {
        guard peer(id)?.role == .robot else { throw RemotePairingError.notPaired }
        var updated = record
        updated.selectedHostID = id
        try save(updated)
    }

    private func save(_ updated: Record) throws {
        let attributes = [kSecValueData as String: try JSONEncoder().encode(updated)]
        let status = SecItemUpdate(Self.query(service: service) as CFDictionary, attributes as CFDictionary)
        guard status == errSecSuccess else { throw RemotePairingError.storage(status) }
        record = updated
    }

    private static func query(service: String) -> [String: Any] {
        [kSecClass as String: kSecClassGenericPassword,
         kSecAttrService as String: service,
         kSecAttrAccount as String: "identity-and-peers",
         kSecAttrSynchronizable as String: false]
    }
}

struct RemotePairingInvitation: Codable {
    let version: Int
    let ticket: String
    let name: String
    let role: RemotePeerRole
    let token: String
    let expiresAt: Date

    static let prefix = "robocar-pair:"

    init(ticket: String, name: String, role: RemotePeerRole, now: Date = Date()) throws {
        var bytes = [UInt8](repeating: 0, count: 32)
        let status = SecRandomCopyBytes(kSecRandomDefault, bytes.count, &bytes)
        guard status == errSecSuccess else { throw RemotePairingError.storage(status) }
        version = 1
        self.ticket = ticket
        self.name = String(name.prefix(32))
        self.role = role
        token = Data(bytes).base64EncodedString()
        expiresAt = now.addingTimeInterval(120)
    }

    var encoded: String {
        get throws { Self.prefix + (try JSONEncoder().encode(self)).base64EncodedString() }
    }

    static func decode(_ text: String, localID: String, role: RemotePeerRole, now: Date = Date()) throws -> Self {
        guard text.utf8.count <= 8192, text.hasPrefix(prefix),
              let data = Data(base64Encoded: String(text.dropFirst(prefix.count))) else {
            throw RemotePairingError.invalidInvitation
        }
        let invitation = try JSONDecoder().decode(Self.self, from: data)
        guard invitation.version == 1, invitation.role != role,
              !invitation.name.isEmpty, invitation.name.utf8.count <= 128,
              Data(base64Encoded: invitation.token)?.count == 32 else {
            throw RemotePairingError.invalidInvitation
        }
        let address = try EndpointTicket.fromString(str: invitation.ticket).endpointAddr()
        guard address.id().description != localID else { throw RemotePairingError.invalidInvitation }
        guard invitation.expiresAt > now,
              invitation.expiresAt.timeIntervalSince(now) <= 180 else {
            throw RemotePairingError.expiredInvitation
        }
        return invitation
    }
}