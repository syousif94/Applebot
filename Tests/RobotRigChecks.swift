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
        precondition(document.checksCollision(for: "upper") && document.checksCollision(for: "lower"))
        var exclusions = document
        exclusions.setCollisionChecking(false, for: ["upper", "lower"])
        try exclusions.validate(partIDs: ["upper", "lower"])
        precondition(!exclusions.checksCollision(for: "upper") && !exclusions.checksCollision(for: "lower"))
        precondition(exclusions.transforms(angles: [parent.id: 90, child.id: 90])[child.id] == transforms[child.id])
        let restoredExclusions = try JSONDecoder().decode(RobotRigDocument.self, from: JSONEncoder().encode(exclusions))
        precondition(restoredExclusions == exclusions)
        exclusions.setCollisionChecking(true, for: ["upper"])
        precondition(exclusions.checksCollision(for: "upper") && !exclusions.checksCollision(for: "lower"))
        exclusions.setCollisionChecking(true, for: ["lower"])
        precondition(exclusions == document)
        exclusions.setCollisionChecking(false, for: ["missing"])
        do { try exclusions.validate(partIDs: ["upper", "lower"]); preconditionFailure("Missing collision part accepted") } catch {}
        print("PASS: collision exclusion persistence, batch updates, restoration, validation and unchanged transforms")
        let legacyJSON = """
        {"version":1,"assetHash":"legacy","groups":[{"id":"00000000-0000-0000-0000-000000000001","name":"Old hinge","parts":[],"axis":{"origin":[0,0,0],"direction":[0,0,1]}}]}
        """
        let legacy = try JSONDecoder().decode(RobotRigDocument.self, from: Data(legacyJSON.utf8))
        try legacy.validate(partIDs: [])
        precondition(legacy.groups[0].linear == nil && legacy.millimetersPerModelUnit == nil)
        precondition(legacy.groups[0].ikMovementCost == nil)
        let legacyTip = legacy.transforms(angles: [legacy.groups[0].id: 90])[legacy.groups[0].id]! * SIMD4<Float>(1, 0, 0, 1)
        precondition(simd_distance(legacyTip, SIMD4(0, 1, 0, 1)) < 0.0001)
        let slide = RobotRigGroup(name: "Slide", parts: ["slide"], parentID: parent.id,
                      axis: RobotRigAxis(origin: SIMD3(8, 9, 10), direction: SIMD3(1, 0, 0)),
                      motor: RobotRigMotorBinding(servoID: 3, ratio: 180), linear: RobotRigLinearMotion())
        let jaw = RobotRigGroup(name: "Opposing jaw", parts: ["jaw"], parentID: parent.id,
                       axis: slide.axis, linear: RobotRigLinearMotion(sourceID: slide.id))
        var linearRig = RobotRigDocument(version: 2, assetHash: "linear", groups: [parent, slide, jaw], millimetersPerModelUnit: 1000)
        try linearRig.validate(partIDs: ["upper", "slide", "jaw"])
        let linearPose = linearRig.transforms(angles: [parent.id: 90, slide.id: 10, jaw.id: 99])
        precondition(simd_distance(linearPose[slide.id]!.columns.3, SIMD4(0, 0.01, 0, 1)) < 0.0001)
        precondition(simd_distance(linearPose[jaw.id]!.columns.3, SIMD4(0, -0.01, 0, 1)) < 0.0001)
        precondition(linearPose[slide.id]!.columns.0 == linearPose[parent.id]!.columns.0)
        let linearBinding = linearRig.motorBinding(for: jaw)!
        precondition(linearBinding.motorAngle(for: 10) == 1800 && linearBinding.jointAngle(for: 1800) == 10)
        precondition(linearBinding.minimum == 0 && linearBinding.maximum == 10)
        let linearRestored = try JSONDecoder().decode(RobotRigDocument.self, from: JSONEncoder().encode(linearRig))
        precondition(linearRestored == linearRig)
        var reversedLinearBinding = linearBinding
        reversedLinearBinding.reversed = true
        reversedLinearBinding.offset = 720
        precondition(reversedLinearBinding.motorAngle(for: 3) == 180)
        precondition(reversedLinearBinding.jointAngle(for: 180) == 3)
        let followerChild = RobotRigGroup(name: "Jaw tip", parts: [], parentID: jaw.id)
        var withChild = linearRig
        withChild.groups.append(followerChild)
        precondition(withChild.transforms(angles: [slide.id: 3])[followerChild.id] == withChild.transforms(angles: [slide.id: 3])[jaw.id])
        func rejects(_ mutate: (inout RobotRigDocument) -> Void) {
            var invalid = linearRig
            mutate(&invalid)
            do { try invalid.validate(partIDs: ["upper", "slide", "jaw"]); preconditionFailure("Invalid linear rig accepted") } catch {}
        }
        rejects { $0.version = 1 }
        rejects { $0.millimetersPerModelUnit = nil }
        rejects { $0.groups[2].linear?.sourceID = UUID() }
        rejects { $0.groups[2].linear?.sourceID = jaw.id }
        rejects { $0.groups[1].linear?.sourceID = jaw.id }
        rejects { $0.groups[2].parentID = slide.id }
        rejects { $0.groups[2].axis?.direction = SIMD3(0, 1, 0) }
        rejects { $0.groups[1].linear?.minimum = 1 }
        rejects { $0.groups[1].linear?.maximum = .infinity }
        rejects { $0.groups[1].linear?.maximum = Double.leastNonzeroMagnitude }
        rejects { $0.groups[1].motor?.ratio = 0 }
        rejects { $0.groups[1].motor?.mode = .position }
        rejects { draft in
            draft.groups.append(RobotRigGroup(name: "Extra follower", parts: [], parentID: parent.id, axis: slide.axis, linear: RobotRigLinearMotion(sourceID: slide.id)))
        }
        linearRig.millimetersPerModelUnit = 1
        precondition(linearRig.transforms(angles: [slide.id: 3])[jaw.id]!.columns.3 == SIMD4(-3, 0, 0, 1))
        linearRig.groups[2].axis?.direction *= -1
        precondition(linearRig.transforms(angles: [slide.id: 3])[jaw.id]!.columns.3 == SIMD4(-3, 0, 0, 1))
        linearRig.millimetersPerModelUnit = 2
        precondition(linearRig.transforms(angles: [slide.id: 3])[slide.id]!.columns.3 == SIMD4(1.5, 0, 0, 1))
        linearRig.groups[2].motor = RobotRigMotorBinding(servoID: 4)
        do { try linearRig.validate(partIDs: ["upper", "slide", "jaw"]); preconditionFailure("Follower motor accepted") } catch {}
        linearRig.groups[2].motor = nil
        linearRig.millimetersPerModelUnit = 0
        do { try linearRig.validate(partIDs: ["upper", "slide", "jaw"]); preconditionFailure("Zero scale accepted") } catch {}
        linearRig.millimetersPerModelUnit = 1
        linearRig.groups[1].ikHelper = RobotRigIKHelper(point: .zero, rootID: slide.id)
        let slideSolution = try linearRig.solveIK(for: slide.id, target: SIMD3(7, 0, 0), angles: [:])
        precondition(slideSolution.reached && abs(slideSolution.angles[slide.id]! - 7) < 0.02)
        let slideLimited = try linearRig.solveIK(for: slide.id, target: SIMD3(20, 0, 0), angles: [:])
        precondition(!slideLimited.reached && slideLimited.angles[slide.id] == 10)
        linearRig.groups[2].ikHelper = RobotRigIKHelper(point: .zero, rootID: jaw.id)
        let jawSolution = try linearRig.solveIK(for: jaw.id, target: SIMD3(0, -6, 0), angles: [parent.id: 90])
        precondition(jawSolution.reached && abs(jawSolution.angles[slide.id]! - 6) < 0.02)
        precondition(jawSolution.angles[parent.id] == 90 && jawSolution.angles[jaw.id] == nil)
        linearRig.groups[1].ikHelper?.rootID = parent.id
        let mixedTarget = linearRig.ikEndpoint(for: slide.id, angles: [parent.id: 40, slide.id: 8])!
        let mixedSolution = try linearRig.solveIK(for: slide.id, target: mixedTarget, angles: [:])
        precondition(mixedSolution.reached)
        linearRig.millimetersPerModelUnit = 1000
        let scaledSolution = try linearRig.solveIK(for: slide.id, target: mixedTarget / 1000, angles: [:])
        precondition(scaledSolution.reached && abs(scaledSolution.angles[slide.id]! - mixedSolution.angles[slide.id]!) < 0.02)
        linearRig.groups[2].ikHelper?.rootID = parent.id
        let coupledTarget = linearRig.ikEndpoint(for: jaw.id, angles: [parent.id: 35, slide.id: 7])!
        let coupledSolution = try linearRig.solveIK(for: jaw.id, target: coupledTarget, angles: [:])
        precondition(coupledSolution.reached)
        linearRig.groups[1].linear?.minimum = -10
        linearRig.groups[1].ikHelper?.rootID = slide.id
        let negativeSolution = try linearRig.solveIK(for: slide.id, target: SIMD3(-0.005, 0, 0), angles: [:])
        precondition(negativeSolution.reached && abs(negativeSolution.angles[slide.id]! + 5) < 0.02)
        print("PASS: linear and mixed IK, coupled source dependency, limits, fixed ancestors and physical scale")
        print("PASS: linear transforms, parent rotation, paired jaws, physical units, calibration and persistence")
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
        var weighted = document
        weighted.groups[1].axis = parent.axis
        weighted.groups[1].ikHelper = RobotRigIKHelper(point: SIMD3(1, 0, 0), rootID: parent.id)
        let weightedTarget = weighted.ikEndpoint(for: child.id, angles: [child.id: 20])!
        let equalCosts = try weighted.solveIK(for: child.id, target: weightedTarget, angles: [:])
        precondition(equalCosts.reached && abs(equalCosts.angles[parent.id]! - equalCosts.angles[child.id]!) < 0.001)
        weighted.groups[0].ikMovementCost = 1
        weighted.groups[1].ikMovementCost = 1
        let explicitDefaults = try weighted.solveIK(for: child.id, target: weightedTarget, angles: [:])
        precondition(explicitDefaults.angles == equalCosts.angles)
        weighted.groups[0].ikMovementCost = 20
        let costlyParent = try weighted.solveIK(for: child.id, target: weightedTarget, angles: [:])
        precondition(costlyParent.reached && abs(costlyParent.angles[parent.id]!) < abs(costlyParent.angles[child.id]!) / 10)
        weighted.groups[0].ikMovementCost = 1
        weighted.groups[1].ikMovementCost = 20
        let costlyChild = try weighted.solveIK(for: child.id, target: weightedTarget, angles: [:])
        precondition(costlyChild.reached && abs(costlyChild.angles[child.id]!) < abs(costlyChild.angles[parent.id]!) / 10)
        try weighted.validate(partIDs: ["upper", "lower"])
        let weightedRestored = try JSONDecoder().decode(RobotRigDocument.self, from: JSONEncoder().encode(weighted))
        precondition(weightedRestored == weighted)
        weighted.groups[0].motor = RobotRigMotorBinding(servoID: 1, minimum: -2, maximum: 2)
        let limitTarget = weighted.ikEndpoint(for: child.id, angles: [child.id: 8])!
        let weightedLimited = try weighted.solveIK(for: child.id, target: limitTarget, angles: [:])
        precondition(weightedLimited.reached && abs(weightedLimited.angles[parent.id]!) <= 2 && weightedLimited.angles[child.id]! > 5)
        for invalidCost: Float in [0, -1, .nan, .infinity, 0.01, 101] {
            weighted.groups[0].ikMovementCost = invalidCost
            do { try weighted.validate(partIDs: ["upper", "lower"]); preconditionFailure("Invalid IK movement cost accepted") } catch {}
            do { _ = try weighted.solveIK(for: child.id, target: weightedTarget, angles: [:]); preconditionFailure("Invalid IK movement cost solved") } catch {}
        }
        print("PASS: weighted IK preference in both directions, default equivalence, legacy decoding, persistence and invalid costs")
        var weightedLinear = document
        weightedLinear.version = 2
        weightedLinear.millimetersPerModelUnit = 1
        for index in weightedLinear.groups.indices {
            weightedLinear.groups[index].axis = RobotRigAxis(origin: .zero, direction: SIMD3(1, 0, 0))
            weightedLinear.groups[index].linear = RobotRigLinearMotion()
        }
        weightedLinear.groups[1].ikHelper = RobotRigIKHelper(point: .zero, rootID: parent.id)
        weightedLinear.groups[0].ikMovementCost = 20
        let weightedSlide = try weightedLinear.solveIK(for: child.id, target: SIMD3(5, 0, 0), angles: [:])
        precondition(weightedSlide.reached && weightedSlide.angles[parent.id]! < weightedSlide.angles[child.id]! / 10)
        let weightedSlideLimited = try weightedLinear.solveIK(for: child.id, target: SIMD3(15, 0, 0), angles: [:])
        precondition(weightedSlideLimited.reached && weightedSlideLimited.angles[child.id] == 10 && weightedSlideLimited.angles[parent.id]! > 4.9)
        let weightedSlideUnreachable = try weightedLinear.solveIK(for: child.id, target: SIMD3(25, 0, 0), angles: [:])
        precondition(!weightedSlideUnreachable.reached && weightedSlideUnreachable.error.isFinite)
        precondition(weightedSlideUnreachable.angles[parent.id]! <= 10 && weightedSlideUnreachable.angles[child.id]! <= 10)
        weightedLinear.millimetersPerModelUnit = 1000
        let weightedScaled = try weightedLinear.solveIK(for: child.id, target: SIMD3(0.005, 0, 0), angles: [:])
        precondition(weightedScaled.reached && abs(weightedScaled.angles[parent.id]! - weightedSlide.angles[parent.id]!) < 0.001)
        linearRig.groups[1].ikMovementCost = 20
        linearRig.groups[2].ikMovementCost = 0.1
        let weightedJaw = try linearRig.solveIK(for: jaw.id, target: coupledTarget, angles: [:])
        linearRig.groups[2].ikMovementCost = 100
        let weightedJawAgain = try linearRig.solveIK(for: jaw.id, target: coupledTarget, angles: [:])
        precondition(weightedJaw.reached && weightedJaw.angles == weightedJawAgain.angles && weightedJaw.angles[jaw.id] == nil)
        print("PASS: weighted rotation and linear limits, costly joint fallback, unreachable targets, physical scale and shared jaw source cost")
        var following = document
        following.groups[1].ikHelper = RobotRigIKHelper(point: SIMD3(2, 0, 0), rootID: parent.id,
            targetFollow: RobotRigTargetFollow(jointID: parent.id, forward: SIMD3(1, 0, 0)))
        let followed = try following.solveIK(for: child.id, target: SIMD3(0, 2, 0), angles: [:])
        precondition(followed.reached && abs(followed.angles[parent.id]! - 90) < 0.001)
        let head = RobotRigGroup(name: "Head", parts: [], parentID: parent.id,
            axis: RobotRigAxis(origin: .zero, direction: SIMD3(0, 0, 1)))
        following.groups.append(head)
        following.groups[1].ikHelper?.targetFollow?.jointID = head.id
        let branchFollowed = try following.solveIK(for: child.id, target: SIMD3(1, 1, 0), angles: [:])
        precondition(branchFollowed.reached)
        let headForward = following.transforms(angles: branchFollowed.angles)[head.id]! * SIMD4<Float>(1, 0, 0, 0)
        precondition(simd_distance(headForward, SIMD4<Float>(sqrt(0.5), sqrt(0.5), 0, 0)) < 0.001)
        let follow = following.groups[1].ikHelper!.targetFollow!
        let onAxis = following.followingTarget(SIMD3(0, 0, 5), follow: follow, angles: [head.id: 17])
        precondition(onAxis[head.id] == 17)
        let wrapped = following.followingTarget(SIMD3(-1, -0.01, 0), follow: follow, angles: [head.id: 179])
        precondition(abs(wrapped[head.id]! + 179.427) < 0.001)
        following.groups[2].motor = RobotRigMotorBinding(servoID: 3, minimum: -20, maximum: 20)
        let constrained = following.followingTarget(SIMD3(0, 1, 0), follow: follow, angles: [:])
        precondition(constrained[head.id] == 20)
        try following.validate(partIDs: ["upper", "lower"])
        let followRestored = try JSONDecoder().decode(RobotRigDocument.self, from: JSONEncoder().encode(following))
        precondition(followRestored == following && legacy.groups[0].ikHelper == nil)
        following.groups[1].ikHelper?.targetFollow?.forward = SIMD3(0, 0, 1)
        do { try following.validate(partIDs: ["upper", "lower"]); preconditionFailure("Parallel forward accepted") } catch {}
        print("PASS: target following with shared and separate joints, parent compensation, axis degeneracy, wraparound, limits and persistence")
        let collar = RobotRigGroup(name: "Collar", parts: [], axis: parent.axis, targetFollowForward: SIMD3(1, 0, 0))
        let rightArm = RobotRigGroup(name: "Right arm", parts: [], parentID: collar.id,
            axis: RobotRigAxis(origin: SIMD3(1, 0, 0), direction: SIMD3(0, 0, 1)))
        let leftArm = RobotRigGroup(name: "Left arm", parts: [], parentID: collar.id,
            axis: RobotRigAxis(origin: SIMD3(1, 0, 0), direction: SIMD3(0, 0, 1)))
        var twoArms = RobotRigDocument(assetHash: "two-arms", groups: [collar, rightArm, leftArm])
        for index in 1...2 {
            twoArms.groups[index].ikHelper = RobotRigIKHelper(point: SIMD3(2, 0, 0), rootID: collar.id)
        }
        try twoArms.validate(partIDs: [])
        precondition(twoArms.groups[0].ikHelper == nil)
        let rightFollowed = try twoArms.solveIK(for: rightArm.id, target: SIMD3(0, 2, 0), angles: [:])
        precondition(rightFollowed.reached && abs(rightFollowed.angles[collar.id]! - 90) < 0.001)
        let leftFollowed = try twoArms.solveIK(for: leftArm.id, target: SIMD3(0, -2, 0), angles: rightFollowed.angles)
        precondition(leftFollowed.reached && abs(leftFollowed.angles[collar.id]! + 90) < 0.001)
        precondition(leftFollowed.angles[rightArm.id] == rightFollowed.angles[rightArm.id])
        let armsRestored = try JSONDecoder().decode(RobotRigDocument.self, from: JSONEncoder().encode(twoArms))
        precondition(armsRestored == twoArms && armsRestored.targetFollowers().count == 1)
        twoArms.groups[0].targetFollowForward = nil
        precondition(twoArms.targetFollowers().isEmpty)
        precondition(legacy.targetFollowers().isEmpty)
        print("PASS: collar without helper follows either active arm target, persistence, disabling and unchanged inactive joint")
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