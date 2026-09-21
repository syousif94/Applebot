import Foundation
import simd

struct RobotRigDocument: Codable, Equatable {
    var version = 1
    var assetHash: String
    var groups: [RobotRigGroup] = []

    func validate(partIDs: Set<String>) throws {
        guard version == 1 else { throw RobotRigError.invalid("Unsupported rig version") }
        guard Set(groups.map(\.id)).count == groups.count else { throw RobotRigError.invalid("Duplicate group IDs") }
        var assigned = Set<String>()
        var names = Set<String>()
        var motors = Set<Int>()
        for group in groups {
            let name = group.name.trimmingCharacters(in: .whitespacesAndNewlines)
            guard !name.isEmpty, names.insert(name.lowercased()).inserted else {
                throw RobotRigError.invalid("Group names must be unique and nonempty")
            }
            for part in group.parts {
                guard partIDs.contains(part), assigned.insert(part).inserted else {
                    throw RobotRigError.invalid("Missing part or part assigned to multiple groups")
                }
            }
            if let axis = group.axis { try axis.validate() }
            if let binding = group.motor {
                try binding.validate()
                guard motors.insert(binding.servoID).inserted else { throw RobotRigError.invalid("A servo can drive only one group") }
            }
            var visited: Set<UUID> = [group.id]
            var parent = group.parentID
            while let parentID = parent {
                guard visited.insert(parentID).inserted,
                      let ancestor = groups.first(where: { $0.id == parentID }) else {
                    throw RobotRigError.invalid("Invalid or cyclic group hierarchy")
                }
                parent = ancestor.parentID
            }
        }
    }

    func transforms(angles: [UUID: Float]) -> [UUID: simd_float4x4] {
        var result: [UUID: simd_float4x4] = [:]
        func resolve(_ group: RobotRigGroup) -> simd_float4x4 {
            if let cached = result[group.id] { return cached }
            let parent = groups.first { $0.id == group.parentID }
            let inherited = parent.map(resolve) ?? matrix_identity_float4x4
            let local = group.axis.map { $0.rotation(degrees: angles[group.id] ?? 0) } ?? matrix_identity_float4x4
            let transform = inherited * local
            result[group.id] = transform
            return transform
        }
        for group in groups { _ = resolve(group) }
        return result
    }
}

struct RobotRigGroup: Codable, Equatable, Identifiable {
    var id = UUID()
    var name: String
    var parts: Set<String>
    var parentID: UUID?
    var axis: RobotRigAxis?
    var motor: RobotRigMotorBinding?
}

struct RobotRigAxis: Codable, Equatable {
    var origin: SIMD3<Float>
    var direction: SIMD3<Float>

    func validate() throws {
        guard origin.isFinite, direction.isFinite, abs(simd_length(direction) - 1) < 0.001 else {
            throw RobotRigError.invalid("Axis requires a finite origin and a unit direction")
        }
    }

    func rotation(degrees: Float) -> simd_float4x4 {
        var forward = matrix_identity_float4x4
        forward.columns.3 = SIMD4(origin, 1)
        var backward = matrix_identity_float4x4
        backward.columns.3 = SIMD4(-origin, 1)
        return forward * simd_float4x4(simd_quatf(angle: degrees * .pi / 180, axis: direction)) * backward
    }
}

struct RobotRigMotorBinding: Codable, Equatable {
    enum Mode: String, Codable, CaseIterable { case position, multiTurn }
    var servoID: Int
    var mode: Mode = .multiTurn
    var reversed = false
    var ratio: Double = 1
    var offset: Double = 0
    var minimum: Double = -90
    var maximum: Double = 90
    var speed: UInt16 = 200
    var robotID: String?

    func validate() throws {
        guard (1...253).contains(servoID), ratio.isFinite, ratio > 0,
              offset.isFinite, minimum.isFinite, maximum.isFinite, minimum < maximum,
              speed > 0, speed <= 3000 else { throw RobotRigError.invalid("Invalid motor calibration or limits") }
        for angle in [minimum, maximum] {
            let motor = motorAngle(for: angle)
            guard motor.isFinite, abs(motor * 10) <= Double(Int32.max),
                  mode != .position || (motor >= 0 && motor <= 4095 * 360.0 / 4096) else {
                throw RobotRigError.invalid("Joint limits exceed the motor coordinate range")
            }
        }
    }

    func motorAngle(for jointAngle: Double) -> Double { offset + (reversed ? -1 : 1) * ratio * jointAngle }
    func jointAngle(for motorAngle: Double) -> Double { (reversed ? -1 : 1) * (motorAngle - offset) / ratio }
}

enum RobotRigError: LocalizedError {
    case invalid(String)
    var errorDescription: String? { if case let .invalid(message) = self { return message }; return nil }
}

extension SIMD3 where Scalar == Float {
    var isFinite: Bool { x.isFinite && y.isFinite && z.isFinite }
}