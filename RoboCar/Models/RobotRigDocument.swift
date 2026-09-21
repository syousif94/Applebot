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
            if let helper = group.ikHelper {
                guard helper.point.isFinite else { throw RobotRigError.invalid("IK helper must be finite") }
                _ = try ikChain(for: group.id)
            }
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
    var ikHelper: RobotRigIKHelper?
}

struct RobotRigIKHelper: Codable, Equatable {
    var point: SIMD3<Float>
    var rootID: UUID
}

struct RobotRigIKResult {
    var angles: [UUID: Float]
    var endpoint: SIMD3<Float>
    var error: Float
    var reached: Bool
}

extension RobotRigDocument {
    func ikChain(for groupID: UUID) throws -> [RobotRigGroup] {
        guard let group = groups.first(where: { $0.id == groupID }), let helper = group.ikHelper else {
            throw RobotRigError.invalid("Add an IK helper first")
        }
        var chain: [RobotRigGroup] = []
        var visited = Set<UUID>()
        var current: RobotRigGroup? = group
        while let joint = current, visited.insert(joint.id).inserted {
            chain.append(joint)
            guard chain.count <= 32 else { throw RobotRigError.invalid("IK chains support up to 32 groups") }
            if joint.id == helper.rootID { return chain }
            current = groups.first { $0.id == joint.parentID }
        }
        throw RobotRigError.invalid("IK chain root must be the child or one of its ancestors")
    }

    func ikEndpoint(for groupID: UUID, angles: [UUID: Float]) -> SIMD3<Float>? {
        guard let helper = groups.first(where: { $0.id == groupID })?.ikHelper,
              let transform = transforms(angles: angles)[groupID] else { return nil }
        let point = transform * SIMD4(helper.point, 1)
        return SIMD3(point.x, point.y, point.z)
    }

    func solveIK(for groupID: UUID, target: SIMD3<Float>, angles initial: [UUID: Float]) throws -> RobotRigIKResult {
        guard target.isFinite, initial.values.allSatisfy(\.isFinite) else { throw RobotRigError.invalid("IK target and angles must be finite") }
        let chain = try ikChain(for: groupID)
        let joints = chain.filter { $0.axis != nil }
        guard !joints.isEmpty, let helper = chain.first?.ikHelper else { throw RobotRigError.invalid("Set a rotation axis on at least one group in the IK chain") }
        let lookup = Dictionary(uniqueKeysWithValues: groups.map { ($0.id, $0) })
        var ancestry = chain
        var visited = Set(chain.map(\.id))
        var parentID = chain.last?.parentID
        while let id = parentID {
            guard visited.insert(id).inserted, let parent = lookup[id] else { throw RobotRigError.invalid("Invalid IK ancestry") }
            ancestry.append(parent)
            parentID = parent.parentID
        }
        func chainTransforms(_ angles: [UUID: Float]) -> [UUID: simd_float4x4] {
            var result: [UUID: simd_float4x4] = [:]
            var inherited = matrix_identity_float4x4
            for joint in ancestry.reversed() {
                inherited *= joint.axis.map { $0.rotation(degrees: angles[joint.id] ?? 0) } ?? matrix_identity_float4x4
                result[joint.id] = inherited
            }
            return result
        }
        let scale = max(joints.reduce(Float(0)) { max($0, simd_distance($1.axis!.origin, helper.point)) }, 0.0001)
        let tolerance = scale * 0.001
        var best = initial
        for joint in joints {
            best[joint.id] = min(max(best[joint.id] ?? 0, Float(joint.motor?.minimum ?? -180)), Float(joint.motor?.maximum ?? 180))
        }
        let initialPoint = chainTransforms(best)[groupID]! * SIMD4(helper.point, 1)
        var bestPoint = SIMD3(initialPoint.x, initialPoint.y, initialPoint.z)
        var bestError = simd_distance(bestPoint, target)
        let seed = best
        for attempt in 0..<3 {
            var angles = best
            if attempt > 0 {
                for (index, joint) in joints.enumerated() {
                    let perturbation: Float = (index % 2 == 0 ? 12 : -12) * (attempt == 1 ? 1 : -1)
                    angles[joint.id] = min(max((seed[joint.id] ?? 0) + perturbation, Float(joint.motor?.minimum ?? -180)), Float(joint.motor?.maximum ?? 180))
                }
            }
            for _ in 0..<100 {
                let transforms = chainTransforms(angles)
                let endpoint4 = transforms[groupID]! * SIMD4(helper.point, 1)
                let endpoint = SIMD3(endpoint4.x, endpoint4.y, endpoint4.z)
                let error = simd_distance(endpoint, target)
                if error < bestError {
                    best = angles
                    bestPoint = endpoint
                    bestError = error
                }
                if bestError <= tolerance { break }
                let columns: [SIMD3<Float>] = joints.map { joint in
                    let inherited = joint.parentID.flatMap { transforms[$0] } ?? matrix_identity_float4x4
                    let origin = inherited * SIMD4(joint.axis!.origin, 1)
                    let direction = inherited * SIMD4(joint.axis!.direction, 0)
                    return simd_cross(SIMD3(direction.x, direction.y, direction.z), (endpoint - SIMD3(origin.x, origin.y, origin.z)) / scale)
                }
                var normal = matrix_identity_float3x3 * Float(0.0025)
                for column in columns {
                    normal += simd_float3x3(columns: (column * column.x, column * column.y, column * column.z))
                }
                let correction = simd_inverse(normal) * ((target - endpoint) / scale)
                guard correction.isFinite else { break }
                var changed = false
                for (joint, column) in zip(joints, columns) {
                    let delta = min(max(simd_dot(column, correction) * 180 / .pi, -10), 10)
                    let old = angles[joint.id] ?? 0
                    let next = min(max(old + delta, Float(joint.motor?.minimum ?? -180)), Float(joint.motor?.maximum ?? 180))
                    changed = changed || abs(next - old) > 0.00001
                    angles[joint.id] = next
                }
                if !changed { break }
            }
            if bestError <= tolerance { break }
        }
        return RobotRigIKResult(angles: best, endpoint: bestPoint, error: bestError, reached: bestError <= tolerance)
    }
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