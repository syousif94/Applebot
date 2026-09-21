import Foundation
import simd

@main
struct RobotRigChecks {
    static func main() throws {
        let triangles = [
            RobotRigTriangle(first: SIMD3(0, 0, 0), second: SIMD3(4, 0, 0), third: SIMD3(0, 2, 0), element: 0, face: 0),
            RobotRigTriangle(first: SIMD3(4, 0, 0), second: SIMD3(4, 2, 0), third: SIMD3(0, 2, 0), element: 1, face: 0),
            RobotRigTriangle(first: SIMD3(8, 0, 0), second: SIMD3(9, 0, 0), third: SIMD3(8, 1, 0), element: 0, face: 1)
        ]
        let geometry = try RobotRigGeometry(triangles: triangles)
        let surface = try geometry.surface(at: 0)
        precondition(surface.triangles == [0, 1])
        precondition(simd_distance(surface.axis.origin, SIMD3(2, 1, 0)) < 0.0001)
        precondition(surface.axis.direction == SIMD3(0, 0, 1))
        precondition(geometry.components() == [[0, 1], [2]])
        let parent = RobotRigGroup(name: "Upper", parts: ["upper"], axis: RobotRigAxis(origin: .zero, direction: SIMD3(0, 0, 1)))
        let child = RobotRigGroup(name: "Lower", parts: ["lower"], parentID: parent.id, axis: RobotRigAxis(origin: SIMD3(1, 0, 0), direction: SIMD3(0, 0, 1)))
        let document = RobotRigDocument(assetHash: "fixture", groups: [parent, child])
        try document.validate(partIDs: ["upper", "lower"])
        let transforms = document.transforms(angles: [parent.id: 90, child.id: 90])
        let tip = transforms[child.id]! * SIMD4<Float>(2, 0, 0, 1)
        precondition(simd_distance(tip, SIMD4(-1, 1, 0, 1)) < 0.0001)
        precondition(document.transforms(angles: [:])[child.id] == matrix_identity_float4x4)
        let restored = try JSONDecoder().decode(RobotRigDocument.self, from: JSONEncoder().encode(document))
        precondition(restored == document)
        var rig = document
        rig.groups[1].ikHelper = RobotRigIKHelper(point: SIMD3(2, 0, 0), rootID: parent.id)
        try rig.validate(partIDs: ["upper", "lower"])
        let solution = try rig.solveIK(for: child.id, target: SIMD3(1, 1, 0), angles: [:])
        precondition(solution.reached && solution.error < 0.003)
        precondition(abs(solution.angles[parent.id] ?? 0) > 1 || abs(solution.angles[child.id] ?? 0) > 1)
        let folded = try rig.solveIK(for: child.id, target: SIMD3(1, 0, 0), angles: [:])
        precondition(folded.reached)
        let unreachable = try rig.solveIK(for: child.id, target: SIMD3(5, 0, 2), angles: solution.angles)
        precondition(!unreachable.reached && unreachable.error.isFinite)
        rig.groups[0].motor = RobotRigMotorBinding(servoID: 1, minimum: -20, maximum: 20)
        rig.groups[1].motor = RobotRigMotorBinding(servoID: 2, minimum: -30, maximum: 30)
        let limited = try rig.solveIK(for: child.id, target: SIMD3(-1, 1, 0), angles: [:])
        precondition(abs(limited.angles[parent.id]!) <= 20 && abs(limited.angles[child.id]!) <= 30)
        rig.groups[1].ikHelper?.rootID = child.id
        let fixedParent = try rig.solveIK(for: child.id, target: SIMD3(0, 2, 0), angles: [parent.id: 90])
        precondition(fixedParent.angles[parent.id] == 90)
        let ikRestored = try JSONDecoder().decode(RobotRigDocument.self, from: JSONEncoder().encode(rig))
        precondition(ikRestored == rig)
        var spatial = document
        spatial.groups[1].axis?.direction = SIMD3(0, 1, 0)
        spatial.groups[1].ikHelper = RobotRigIKHelper(point: SIMD3(2, 0, 0), rootID: parent.id)
        let spatialTarget = spatial.ikEndpoint(for: child.id, angles: [parent.id: 35, child.id: 50])!
        let spatialSolution = try spatial.solveIK(for: child.id, target: spatialTarget, angles: [:])
        precondition(spatialSolution.reached)
        let unrelatedID = UUID()
        let unchanged = try spatial.solveIK(for: child.id, target: spatialTarget, angles: [unrelatedID: 17])
        precondition(unchanged.angles[unrelatedID] == 17)
        do { _ = try spatial.solveIK(for: child.id, target: SIMD3(.nan, 0, 0), angles: [:]); preconditionFailure("Nonfinite IK target accepted") } catch {}
        rig.groups[1].ikHelper?.rootID = UUID()
        do { try rig.validate(partIDs: ["upper", "lower"]); preconditionFailure("Invalid IK root accepted") } catch {}
        print("PASS: IK reachable, straight-chain folding, unreachable, limits, fixed ancestors and helper persistence")
        var cyclic = document
        cyclic.groups[0].parentID = child.id
        do { try cyclic.validate(partIDs: ["upper", "lower"]); preconditionFailure("Cycle accepted") } catch {}
        let motor = RobotRigMotorBinding(servoID: 2, reversed: true, ratio: 2, offset: 720)
        try motor.validate()
        precondition(motor.motorAngle(for: 30) == 660)
        precondition(motor.jointAngle(for: 660) == 30)
        let skewed = try RobotRigGeometry(triangles: [
            RobotRigTriangle(first: SIMD3(0, 0, 0), second: SIMD3(4, 0, 0), third: SIMD3(1, 0.3, 0), element: 0, face: 0),
            RobotRigTriangle(first: SIMD3(4, 0, 0), second: SIMD3(4, 2, 0), third: SIMD3(1, 0.3, 0), element: 0, face: 1),
            RobotRigTriangle(first: SIMD3(4, 2, 0), second: SIMD3(0, 2, 0), third: SIMD3(1, 0.3, 0), element: 0, face: 2),
            RobotRigTriangle(first: SIMD3(0, 2, 0), second: SIMD3(0, 0, 0), third: SIMD3(1, 0.3, 0), element: 0, face: 3)
        ])
        let skewedSurface = try skewed.surface(at: 0)
        precondition(simd_distance(skewedSurface.axis.origin, SIMD3(2, 1, 0)) < 0.0001)
        let bent = try RobotRigGeometry(triangles: [triangles[0],
            RobotRigTriangle(first: SIMD3(4, 0, 0), second: SIMD3(4, 2, 0.01), third: SIMD3(0, 2, 0), element: 0, face: 1)
        ])
        let bentSurface = try bent.surface(at: 0)
        precondition(bentSurface.triangles == [0])
        let mirrored = try RobotRigGeometry(triangles: triangles.map { triangle in
            func transform(_ point: SIMD3<Float>) -> SIMD3<Float> { point * SIMD3(-2, 3, 1) + SIMD3(10, -5, 2) }
            return RobotRigTriangle(first: transform(triangle.first), second: transform(triangle.second), third: transform(triangle.third), element: triangle.element, face: triangle.face)
        })
        let mirroredSurface = try mirrored.surface(at: 0)
        precondition(simd_distance(mirroredSurface.axis.origin, SIMD3(6, -2, 2)) < 0.0001)
        precondition(mirroredSurface.axis.direction == SIMD3(0, 0, -1))
        var duplicate = document
        duplicate.groups[1].parts.insert("upper")
        do { try duplicate.validate(partIDs: ["upper", "lower"]); preconditionFailure("Duplicate part ownership accepted") } catch {}
        var invalidBinding = motor
        invalidBinding.ratio = .nan
        do { try invalidBinding.validate(); preconditionFailure("Invalid gearing accepted") } catch {}
        for _ in 0..<100 {
            precondition(document.transforms(angles: [parent.id: 90, child.id: 90])[child.id] == transforms[child.id])
        }
        print("PASS: planar region, centroid, disconnected components, nested pivots, persistence, cycles, motor mapping")
        print("PASS: unequal triangulation, curved neighbor exclusion, mirrored/nonuniform transforms, duplicate ownership, invalid gearing, no transform drift")
    }
}