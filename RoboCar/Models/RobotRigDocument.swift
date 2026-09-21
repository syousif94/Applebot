import Foundation
import simd

struct RobotRigDocument: Codable, Equatable {
    var version = 1
    var assetHash: String
    var groups: [RobotRigGroup] = []
    var millimetersPerModelUnit: Float?
    var collisionExcludedPartIDs: Set<String>?

    func validate(partIDs: Set<String>) throws {
        guard (1...2).contains(version) else { throw RobotRigError.invalid("Unsupported rig version") }
        guard (collisionExcludedPartIDs ?? []).isSubset(of: partIDs) else {
            throw RobotRigError.invalid("Collision exclusions reference missing parts")
        }
        if let scale = millimetersPerModelUnit {
            guard scale.isFinite, scale > 0, (1 / scale).isFinite else { throw RobotRigError.invalid("Model scale must be finite and positive") }
        }
        if groups.contains(where: { $0.linear != nil }) {
            guard version == 2, millimetersPerModelUnit != nil else { throw RobotRigError.invalid("Confirm model units before adding linear motion") }
        }
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
            if let linear = group.linear {
                guard group.axis != nil, linear.minimum.isFinite, linear.maximum.isFinite,
                      linear.minimum <= 0, linear.maximum >= 0, linear.minimum < linear.maximum,
                      Float(linear.minimum).isFinite, Float(linear.maximum).isFinite,
                      Float(linear.minimum) < Float(linear.maximum),
                      Float(linear.maximum - linear.minimum) / millimetersPerModelUnit! > 0,
                      (Float(linear.maximum - linear.minimum) / millimetersPerModelUnit!).isFinite else {
                    throw RobotRigError.invalid("Linear joints require an axis and finite travel limits containing zero")
                }
                if let sourceID = linear.sourceID {
                    guard sourceID != group.id, let source = groups.first(where: { $0.id == sourceID }),
                          source.linear != nil, source.linear?.sourceID == nil, source.parentID == group.parentID,
                          let sourceAxis = source.axis, let axis = group.axis,
                          abs(simd_dot(sourceAxis.direction, axis.direction)) > 0.9999,
                          group.motor == nil,
                          groups.filter({ $0.linear?.sourceID == sourceID }).count == 1 else {
                        throw RobotRigError.invalid("An opposing jaw must share its source's parent and axis direction, with no separate motor or linked chain")
                    }
                }
            }
            if let helper = group.ikHelper {
                guard helper.point.isFinite else { throw RobotRigError.invalid("IK helper must be finite") }
                _ = try ikChain(for: group.id)
            }
            if let binding = group.motor {
                try motorBinding(for: group)!.validate()
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

    func checksCollision(for partID: String) -> Bool {
        collisionExcludedPartIDs?.contains(partID) != true
    }

    mutating func setCollisionChecking(_ enabled: Bool, for partIDs: Set<String>) {
        var excluded = collisionExcludedPartIDs ?? []
        if enabled { excluded.subtract(partIDs) }
        else { excluded.formUnion(partIDs) }
        collisionExcludedPartIDs = excluded.isEmpty ? nil : excluded
    }

    func motionSource(for group: RobotRigGroup) -> RobotRigGroup {
        group.linear?.sourceID.flatMap { id in groups.first { $0.id == id } } ?? group
    }

    func limits(for group: RobotRigGroup) -> ClosedRange<Float> {
        let source = motionSource(for: group)
        return Float(source.linear?.minimum ?? source.motor?.minimum ?? -180)...Float(source.linear?.maximum ?? source.motor?.maximum ?? 180)
    }

    func motorBinding(for group: RobotRigGroup) -> RobotRigMotorBinding? {
        let source = motionSource(for: group)
        guard var binding = source.motor else { return nil }
        if let linear = source.linear {
            binding.minimum = linear.minimum
            binding.maximum = linear.maximum
        }
        return binding
    }

    func couplingSign(for group: RobotRigGroup) -> Float {
        guard group.linear?.sourceID != nil, let axis = group.axis,
              let sourceAxis = motionSource(for: group).axis else { return 1 }
        return simd_dot(axis.direction, sourceAxis.direction) >= 0 ? -1 : 1
    }

    func localTransform(for group: RobotRigGroup, angles: [UUID: Float]) -> simd_float4x4 {
        guard let axis = group.axis else { return matrix_identity_float4x4 }
        let position = (angles[motionSource(for: group).id] ?? 0) * couplingSign(for: group)
        guard group.linear != nil else { return axis.rotation(degrees: position) }
        var transform = matrix_identity_float4x4
        transform.columns.3 = SIMD4(axis.direction * (position / (millimetersPerModelUnit ?? 1)), 1)
        return transform
    }

    func transforms(angles: [UUID: Float]) -> [UUID: simd_float4x4] {
        var result: [UUID: simd_float4x4] = [:]
        func resolve(_ group: RobotRigGroup) -> simd_float4x4 {
            if let cached = result[group.id] { return cached }
            let parent = groups.first { $0.id == group.parentID }
            let inherited = parent.map(resolve) ?? matrix_identity_float4x4
            let local = localTransform(for: group, angles: angles)
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
    var linear: RobotRigLinearMotion?
}

struct RobotRigLinearMotion: Codable, Equatable {
    var minimum: Double = 0
    var maximum: Double = 10
    var sourceID: UUID?
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
        var sourceIDs = Set<UUID>()
        let joints = chain.filter { $0.axis != nil }.map { motionSource(for: $0) }.filter { sourceIDs.insert($0.id).inserted }
        guard !joints.isEmpty, let helper = chain.first?.ikHelper else { throw RobotRigError.invalid("Set a motion axis on at least one group in the IK chain") }
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
                inherited *= localTransform(for: joint, angles: angles)
                result[joint.id] = inherited
            }
            return result
        }
        let modelUnitsPerMM = 1 / (millimetersPerModelUnit ?? 1)
        let scale = max(joints.reduce(Float(0)) { result, joint in
            let limits = limits(for: joint)
            let extent = joint.linear == nil ? simd_distance(joint.axis!.origin, helper.point) : (limits.upperBound - limits.lowerBound) * modelUnitsPerMM
            return max(result, extent)
        }, 0.0001)
        func nativeStep(_ joint: RobotRigGroup) -> Float {
            joint.linear == nil ? 180 / .pi : scale / modelUnitsPerMM
        }
        func maximumStep(_ joint: RobotRigGroup) -> Float {
            let limits = limits(for: joint)
            return joint.linear == nil ? 10 : (limits.upperBound - limits.lowerBound) * 0.1
        }
        func clamped(_ value: Float, for joint: RobotRigGroup) -> Float {
            let limits = limits(for: joint)
            return min(max(value, limits.lowerBound), limits.upperBound)
        }
        let tolerance = scale * 0.001
        var best = initial
        for joint in joints {
            best[joint.id] = clamped(best[joint.id] ?? 0, for: joint)
        }
        let initialPoint = chainTransforms(best)[groupID]! * SIMD4(helper.point, 1)
        var bestPoint = SIMD3(initialPoint.x, initialPoint.y, initialPoint.z)
        var bestError = simd_distance(bestPoint, target)
        let seed = best
        for attempt in 0..<3 {
            var angles = best
            if attempt > 0 {
                for (index, joint) in joints.enumerated() {
                    let perturbation: Float = maximumStep(joint) * 1.2 * (index % 2 == 0 ? 1 : -1) * (attempt == 1 ? 1 : -1)
                    angles[joint.id] = clamped((seed[joint.id] ?? 0) + perturbation, for: joint)
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
                    chain.filter { $0.axis != nil && motionSource(for: $0).id == joint.id }.reduce(SIMD3<Float>.zero) { column, influence in
                        let inherited = influence.parentID.flatMap { transforms[$0] } ?? matrix_identity_float4x4
                        let origin = inherited * SIMD4(influence.axis!.origin, 1)
                        let direction = inherited * SIMD4(influence.axis!.direction, 0)
                        let vector = SIMD3(direction.x, direction.y, direction.z)
                        if influence.linear != nil { return column + vector * couplingSign(for: influence) }
                        return column + simd_cross(vector, (endpoint - SIMD3(origin.x, origin.y, origin.z)) / scale)
                    }
                }
                var normal = matrix_identity_float3x3 * Float(0.0025)
                for column in columns {
                    normal += simd_float3x3(columns: (column * column.x, column * column.y, column * column.z))
                }
                let correction = simd_inverse(normal) * ((target - endpoint) / scale)
                guard correction.isFinite else { break }
                var changed = false
                for (joint, column) in zip(joints, columns) {
                    let delta = min(max(simd_dot(column, correction) * nativeStep(joint), -maximumStep(joint)), maximumStep(joint))
                    let old = angles[joint.id] ?? 0
                    let next = clamped(old + delta, for: joint)
                    changed = changed || abs(next - old) > maximumStep(joint) * 0.000001
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