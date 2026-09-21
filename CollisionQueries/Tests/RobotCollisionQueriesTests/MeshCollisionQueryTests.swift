import Euclid
import XCTest
import simd
@testable import RobotCollisionQueries

final class MeshCollisionQueryTests: XCTestCase {
    func testMetalOpenSurfacesTouchingAndContainmentBoundaries() throws {
        let horizontal = Mesh([Polygon([Vertex(Vector(-1, -1, 0)), Vertex(Vector(1, -1, 0)), Vertex(Vector(0, 1, 0))])!])
        let vertical = Mesh([Polygon([Vertex(Vector(0, -0.5, -1)), Vertex(Vector(0, -0.5, 1)), Vertex(Vector(0, 0.5, 1))])!])
        let cube = Mesh.cube().triangulate()
        let gpu = try XCTUnwrap(MetalCollisionQuery(surfaces: [.init(mesh: horizontal, closed: false), .init(mesh: vertical, closed: false), .init(mesh: cube, closed: true)]))
        var touching = matrix_identity_float4x4
        touching.columns.3.x = 1
        var penetrating = touching
        penetrating.columns.3.x = 0.999
        var separate = touching
        separate.columns.3.x = 1.001
        let pairs: [MetalCollisionQuery.Pair] = [
            .init(first: 0, second: 1, firstToSecond: matrix_identity_float4x4),
            .init(first: 0, second: 0, firstToSecond: matrix_identity_float4x4),
            .init(first: 2, second: 2, firstToSecond: touching),
            .init(first: 2, second: 2, firstToSecond: penetrating),
            .init(first: 2, second: 2, firstToSecond: separate)
        ]
        XCTAssertEqual(gpu.check(pairs, timeLimit: 2), [.surfaceIntersection, .clear, .clear, .surfaceIntersection, .clear])
    }

    func testGPUWorldChecksIntermediatePosesAndUngroupedContacts() {
        let world = RigCollisionWorld(parts: [part("arm", owner: "arm", mesh: .cube(size: 0.5)),
                                             part("body", owner: nil, mesh: .cube(center: Vector(2, 0, 0), size: 0.5))],
                                      joints: [.init(id: "arm", parent: nil, pivot: SIMD3(2, 0, 0))], moving: ["arm"])
        XCTAssertTrue(world.usesGPU)
        let poses = (0...16).map { step in
            var transform = matrix_identity_float4x4
            transform.columns.3.x = Float(step) / 4
            return ["arm": transform]
        }
        let result = world.check(poses: poses, timeLimit: 2, measureOverlap: false)
        XCTAssertGreaterThan(result.accepted, 0)
        XCTAssertLessThan(result.accepted, poses.count - 1)
        XCTAssertEqual(Set(result.blockedParts), ["arm", "body"])
        XCTAssertEqual(world.check(poses: poses, timeLimit: 0, measureOverlap: false).accepted, 0)
        var invalid = matrix_identity_float4x4
        invalid.columns.3.x = .nan
        XCTAssertEqual(world.check(poses: [[:], ["arm": invalid]], measureOverlap: false).accepted, 0)
    }

    func testGPUWorldWarmedQueryTiming() {
        let bracket = Mesh.cube(size: Vector(4, 4, 1)).subtracting(.cube(size: Vector(3, 3, 2))).makeWatertight()
        let world = RigCollisionWorld(parts: [part("bracket", owner: nil, mesh: bracket),
                                             part("link", owner: "arm", mesh: .cube(size: Vector(2, 0.3, 0.3)))], joints: [], moving: ["arm"])
        XCTAssertTrue(world.usesGPU)
        XCTAssertEqual(world.check(poses: [[:], [:]], timeLimit: 2, measureOverlap: false).accepted, 1)
        var durations: [Double] = []
        for degrees in 0..<60 {
            let transform = simd_float4x4(simd_quatf(angle: Float(degrees) * .pi / 180, axis: SIMD3(0, 0, 1)))
            let started = ContinuousClock.now
            let result = world.check(poses: [[:], ["arm": transform]], timeLimit: 2, measureOverlap: false)
            let elapsed = started.duration(to: .now).components
            durations.append(Double(elapsed.seconds) + Double(elapsed.attoseconds) / 1e18)
            XCTAssertEqual(result.accepted, 1, result.message ?? "")
        }
        let sorted = durations.sorted()
        print("GPU bracket: 60 warmed world queries; mean \(durations.reduce(0, +) / 60 * 1000)ms; p95 \(sorted[56] * 1000)ms")
    }

    func testMetalQueriesMatchRigidSurfaceQueries() throws {
        let bracket = Mesh.cube(size: Vector(4, 4, 1)).subtracting(.cube(size: Vector(3, 3, 2))).makeWatertight().triangulate()
        let link = Mesh.cube(size: Vector(2, 0.3, 0.3)).triangulate()
        let outer = Mesh.cube(size: 5).triangulate()
        let gpu = try XCTUnwrap(MetalCollisionQuery(surfaces: [.init(mesh: link, closed: true), .init(mesh: bracket, closed: true), .init(mesh: outer, closed: true)]))
        var pairs: [MetalCollisionQuery.Pair] = []
        var expected: [MeshCollisionQuery.Result] = []
        for offset in [0.0, 1.5, 4.0] {
            for degrees in stride(from: 0, through: 180, by: 15) {
                var transform = simd_float4x4(simd_quatf(angle: Float(degrees) * .pi / 180, axis: SIMD3(0, 0, 1)))
                transform.columns.3.x = Float(offset)
                pairs.append(.init(first: 0, second: 1, firstToSecond: transform))
                expected.append(MeshCollisionQuery.checkSurfaces(link.rotated(by: .roll(.degrees(Double(degrees)))).translated(by: Vector(offset, 0, 0)), bracket))
            }
        }
        pairs.append(.init(first: 0, second: 2, firstToSecond: matrix_identity_float4x4))
        expected.append(.surfaceIntersection)
        pairs.append(.init(first: 2, second: 0, firstToSecond: matrix_identity_float4x4))
        expected.append(.surfaceIntersection)
        pairs.append(.init(first: 0, second: 2, firstToSecond: matrix_identity_float4x4, contactSphere: SIMD4(0, 0, 0, 10)))
        expected.append(.clear)
        XCTAssertEqual(gpu.check(pairs, timeLimit: 2), expected)
        XCTAssertEqual(gpu.check(pairs, timeLimit: 0), pairs.map { _ in .timedOut })
    }

    func testSIMDBoundsEncloseTransformedCorners() {
        for size in [0.00001, 1.0, 10000.0] {
            let bounds = Bounds(min: Vector(-2 * size, -size, -0.3 * size), max: Vector(size, 3 * size, 0.7 * size))
            for degrees in stride(from: 0, through: 360, by: 7) {
                var transform = simd_float4x4(simd_quatf(angle: Float(degrees) * .pi / 180, axis: simd_normalize(SIMD3<Float>(1, 2, 3))))
                transform.columns.3 = SIMD4(17, -3, 0.25, 1)
                let result = MeshCollisionQuery.transformedBounds(bounds, by: transform)
                for horizontal in [bounds.min.x, bounds.max.x] {
                    for vertical in [bounds.min.y, bounds.max.y] {
                        for depth in [bounds.min.z, bounds.max.z] {
                            let point = transform * SIMD4(Float(horizontal), Float(vertical), Float(depth), 1)
                            XCTAssertTrue(result.intersects(Vector(Double(point.x), Double(point.y), Double(point.z))))
                        }
                    }
                }
            }
        }
    }

    func testPreparedQueriesMatchSurfaceQueriesUnderRigidMotion() {
        let bracket = Mesh.cube(size: Vector(4, 4, 1)).subtracting(.cube(size: Vector(3, 3, 2))).makeWatertight().triangulate()
        let link = Mesh.cube(size: Vector(2, 0.3, 0.3)).triangulate()
        let preparedBracket = MeshCollisionQuery.PreparedSurface(mesh: bracket, closed: true)
        let preparedLink = MeshCollisionQuery.PreparedSurface(mesh: link, closed: true)
        for offset in [0.0, 1.5, 4.0] {
            for degrees in stride(from: 0, through: 180, by: 15) {
                var transform = simd_float4x4(simd_quatf(angle: Float(degrees) * .pi / 180, axis: SIMD3(0, 0, 1)))
                transform.columns.3.x = Float(offset)
                let expected = MeshCollisionQuery.checkSurfaces(link.rotated(by: .roll(.degrees(Double(degrees)))).translated(by: Vector(offset, 0, 0)), bracket)
                XCTAssertEqual(MeshCollisionQuery.checkPrepared(preparedLink, preparedBracket, firstToSecond: transform, isCancelled: { false }), expected)
                XCTAssertEqual(MeshCollisionQuery.checkPrepared(preparedBracket, preparedLink, firstToSecond: transform.inverse, isCancelled: { false }), expected)
            }
        }
        let solid = MeshCollisionQuery.PreparedSurface(mesh: .cube(size: 5), closed: true)
        XCTAssertEqual(MeshCollisionQuery.checkPrepared(preparedLink, solid, firstToSecond: matrix_identity_float4x4, isCancelled: { false }), .surfaceIntersection)
        XCTAssertEqual(MeshCollisionQuery.checkPrepared(solid, preparedLink, firstToSecond: matrix_identity_float4x4, isCancelled: { false }), .surfaceIntersection)
        XCTAssertEqual(MeshCollisionQuery.checkPrepared(preparedLink, solid, firstToSecond: matrix_identity_float4x4, contactSphere: (.zero, 10), isCancelled: { false }), .clear)
        XCTAssertEqual(MeshCollisionQuery.checkPrepared(preparedLink, solid, firstToSecond: matrix_identity_float4x4, isCancelled: { true }), .timedOut)
    }

    func testAdjacentLinksOnlyExcludeJointContact() {
        let upper = Mesh.cube(center: Vector(-1, 0, 0), size: Vector(2.1, 0.2, 0.2))
        let lower = Mesh.cube(center: Vector(1, 0, 0), size: Vector(2.1, 0.2, 0.2))
        let world = RigCollisionWorld(parts: [part("upper", owner: "base", mesh: upper), part("lower", owner: "hinge", mesh: lower)],
                                     joints: [.init(id: "hinge", parent: "base", pivot: .zero)], moving: ["base", "hinge"])
        let bend = simd_float4x4(simd_quatf(angle: .pi / 6, axis: SIMD3(0, 0, 1)))
        XCTAssertEqual(world.check(poses: [[:], ["hinge": bend]], timeLimit: 1, measureOverlap: false).accepted, 1)
        let fold = simd_float4x4(simd_quatf(angle: .pi, axis: SIMD3(0, 0, 1)))
        XCTAssertEqual(Set(world.check(poses: [[:], ["hinge": fold]], timeLimit: 1, measureOverlap: false).blockedParts), ["upper", "lower"])
        let nonAdjacent = RigCollisionWorld(parts: [part("upper", owner: "base", mesh: upper), part("lower", owner: "hinge", mesh: lower)],
                           joints: [.init(id: "hinge", parent: "middle", pivot: .zero), .init(id: "middle", parent: "base", pivot: .zero)], moving: ["base", "middle", "hinge"])
        XCTAssertEqual(Set(nonAdjacent.check(poses: [[:], ["hinge": fold]], timeLimit: 1, measureOverlap: false).blockedParts), ["upper", "lower"])
        var parent = matrix_identity_float4x4
        parent.columns.3 = SIMD4(2, 3, 4, 1)
        XCTAssertEqual(world.check(poses: [[:], ["base": parent, "hinge": parent * bend]], timeLimit: 1, measureOverlap: false).accepted, 1)
        XCTAssertEqual(Set(world.check(poses: [[:], ["base": parent, "hinge": parent * fold]], timeLimit: 1, measureOverlap: false).blockedParts), ["upper", "lower"])
        let distantPivot = RigCollisionWorld(parts: [part("upper", owner: "base", mesh: upper), part("lower", owner: "hinge", mesh: lower)],
                            joints: [.init(id: "hinge", parent: "base", pivot: SIMD3(10, 0, 0))], moving: ["hinge"])
        XCTAssertEqual(Set(distantPivot.check(poses: [[:], ["hinge": bend]], timeLimit: 1, measureOverlap: false).blockedParts), ["upper", "lower"])
    }

    func testWideJointContactWithSurfacePivotAllowsBendButBlocksFold() {
        let upper = Mesh.cube(center: Vector(-1, 0, 0), size: Vector(2.1, 0.2, 0.8))
        let lower = Mesh.cube(center: Vector(1, 0, 0), size: Vector(2.1, 0.2, 0.8))
        let world = RigCollisionWorld(parts: [part("servo", owner: "base", mesh: upper), part("wrist", owner: "hinge", mesh: lower)],
                                      joints: [.init(id: "hinge", parent: "base", pivot: SIMD3(0, 0, -0.4))], moving: ["base", "hinge"])
        let bend = simd_float4x4(simd_quatf(angle: .pi / 6, axis: SIMD3(0, 0, 1)))
        let fold = simd_float4x4(simd_quatf(angle: .pi, axis: SIMD3(0, 0, 1)))
        XCTAssertEqual(world.check(poses: [[:], ["hinge": bend]], timeLimit: 1, measureOverlap: false).accepted, 1)
        XCTAssertEqual(Set(world.check(poses: [[:], ["hinge": fold]], timeLimit: 1, measureOverlap: false).blockedParts), ["servo", "wrist"])
        var parent = simd_float4x4(simd_quatf(angle: .pi / 4, axis: SIMD3(1, 0, 0)))
        parent.columns.3 = SIMD4(2, 3, 4, 1)
        XCTAssertEqual(world.check(poses: [[:], ["base": parent, "hinge": parent * bend]], timeLimit: 1, measureOverlap: false).accepted, 1)
        XCTAssertEqual(Set(world.check(poses: [[:], ["base": parent, "hinge": parent * fold]], timeLimit: 1, measureOverlap: false).blockedParts), ["servo", "wrist"])
    }

    func testJointContactAlongShaftAllowsBendButRejectsOffAxisContact() {
        let upper = Mesh.cube(center: Vector(-1, 0, 0), size: Vector(2.1, 0.2, 0.8))
        let lower = Mesh.cube(center: Vector(1, 0, 0), size: Vector(2.1, 0.2, 0.8))
        let parts = [part("mount", owner: "elbow", mesh: lower), part("servo", owner: "arm", mesh: upper)]
        let world = RigCollisionWorld(parts: parts,
                                      joints: [.init(id: "elbow", parent: "arm", pivot: SIMD3(0, 0, -2), direction: SIMD3(0, 0, 1))], moving: ["elbow"])
        let bend = simd_float4x4(simd_quatf(angle: .pi / 6, axis: SIMD3(0, 0, 1)))
        let fold = simd_float4x4(simd_quatf(angle: .pi, axis: SIMD3(0, 0, 1)))
        XCTAssertEqual(world.check(poses: [[:], ["elbow": bend]], timeLimit: 1, measureOverlap: false).accepted, 1)
        XCTAssertEqual(Set(world.check(poses: [[:], ["elbow": fold]], timeLimit: 1, measureOverlap: false).blockedParts), ["mount", "servo"])
        let offAxis = RigCollisionWorld(parts: parts,
                                        joints: [.init(id: "elbow", parent: "arm", pivot: SIMD3(0, 2, -2), direction: SIMD3(0, 0, 1))], moving: ["elbow"])
        XCTAssertEqual(Set(offAxis.check(poses: [[:], ["elbow": bend]], timeLimit: 1, measureOverlap: false).blockedParts), ["mount", "servo"])
    }

    func testTinyJointContainmentDoesNotCrash() {
        let tiny = Mesh.cube(size: 1).scaled(by: 0.0002)
        let world = RigCollisionWorld(parts: [part("tiny", owner: "arm", mesh: tiny),
                                             part("body", owner: nil, mesh: .cube(size: 2))],
                                      joints: [.init(id: "arm", parent: nil, pivot: .zero)], moving: ["arm"])
        let outcome = world.check(poses: [[:], [:]], timeLimit: 1, measureOverlap: false)
        XCTAssertEqual(Set(outcome.blockedParts), ["tiny", "body"])
        XCTAssertNil(MeshCollisionQuery.jointRegion(tiny.scaled(by: 0.001), tiny.scaled(by: 0.001), pivot: .zero))
    }

    func testInteractiveChecksProposedPoseWithoutVolumeBaseline() {
        let world = RigCollisionWorld(parts: [part("arm", owner: "arm", mesh: .cube()),
                                             part("body", owner: nil, mesh: .cube(size: 2))], joints: [], moving: ["arm"])
        var away = matrix_identity_float4x4
        away.columns.3.x = 3
        let escaped = world.check(poses: [[:], ["arm": away]], measureOverlap: false)
        XCTAssertEqual(escaped.accepted, 1)
        let blocked = world.check(poses: [["arm": away], [:]], measureOverlap: false)
        XCTAssertEqual(blocked.accepted, 0)
        XCTAssertEqual(Set(blocked.blockedParts), ["arm", "body"])
    }

    func testCollisionProxyReducesTrianglesWithinErrorBudget() {
        let source = Mesh.sphere(radius: 2, slices: 64).triangulate()
        let count = source.polygons.count
        let proxy = CollisionMesh.simplify(source)
        XCTAssertLessThan(proxy.mesh.polygons.count, count)
        XCTAssertEqual(source.polygons.count, count)
        XCTAssertTrue(proxy.mesh.isWatertight)
        XCTAssertLessThanOrEqual(proxy.relativeError, 0.001)
        XCTAssertGreaterThan(proxy.mesh.signedVolume, 0)
        XCTAssertEqual(CollisionMesh.simplify(.cube()).mesh.polygons.count, 6)
    }

    func testCollisionProxyPreservesBracketOpeningAndOpenBorders() {
        let bracket = Mesh.cube(size: Vector(4, 4, 1)).subtracting(.cube(size: Vector(3, 3, 2))).makeWatertight()
        let detailed = bracket.triangulate().subdivide().subdivide().subdivide()
        let proxy = CollisionMesh.simplify(detailed, targetTriangles: 100)
        XCTAssertLessThan(proxy.mesh.polygons.count, detailed.polygons.count)
        XCTAssertTrue(proxy.mesh.isWatertight)
        let link = Mesh.cube(size: Vector(2, 0.3, 0.3))
        for degrees in stride(from: 0, through: 180, by: 15) {
            XCTAssertEqual(MeshCollisionQuery.checkSurfaces(proxy.mesh, link.rotated(by: .roll(.degrees(Double(degrees))))), .clear)
        }
        XCTAssertEqual(MeshCollisionQuery.checkSurfaces(proxy.mesh, link.translated(by: Vector(1.5, 0, 0))), .surfaceIntersection)
        let open = Mesh([Polygon([Vertex(Vector(-1, -1, 0)), Vertex(Vector(1, -1, 0)), Vertex(Vector(0, 1, 0))])!])
            .subdivide().subdivide().subdivide()
        let reduced = CollisionMesh.simplify(open, targetTriangles: 1).mesh
        func boundary(_ mesh: Mesh) -> Set<LineSegment> {
            var counts: [LineSegment: Int] = [:]
            for polygon in mesh.polygons {
                for edge in polygon.undirectedEdges { counts[edge, default: 0] += 1 }
            }
            return Set(counts.filter { $0.value == 1 }.keys)
        }
        XCTAssertEqual(boundary(open), boundary(reduced))
        XCTAssertFalse(reduced.isWatertight)
    }

    private final class WorkBudget: @unchecked Sendable {
        private let lock = NSLock()
        private var remaining = 100_000
        func exhausted() -> Bool {
            lock.lock()
            defer { lock.unlock() }
            remaining -= 1
            return remaining < 0
        }
    }

    func testDenseSeparatedSurfacesDoNotScanEveryTrianglePair() {
        func strips(inverted: Bool) -> Mesh {
            Mesh((0..<2_000).map { index in
                let height = Double(index)
                let depth = Double((index + (inverted ? 1 : 0)) % 2)
                return Polygon([Vertex(Vector(0, height, depth)), Vertex(Vector(1, height, depth)),
                                Vertex(Vector(0, height + 0.5, depth))])!
            })
        }
        let first = strips(inverted: false)
        let second = strips(inverted: true)
        XCTAssertTrue(first.bounds.intersects(second.bounds))
        let budget = WorkBudget()
        let result = MeshCollisionQuery.checkSurfaces(first, second, isCancelled: { budget.exhausted() })
        XCTAssertEqual(result, .clear)
        let world = RigCollisionWorld(parts: [part("first", owner: nil, mesh: first), part("second", owner: "arm", mesh: second)],
                          joints: [], moving: ["arm"])
        var shifted = matrix_identity_float4x4
        shifted.columns.3.y = 0.1
        let outcome = world.check(poses: [[:], ["arm": shifted]])
        XCTAssertEqual(outcome.accepted, 1, outcome.message ?? "")
        XCTAssertTrue(outcome.blockedParts.isEmpty)
    }

    func testOpenSurfaceCrossingAndSeparation() {
        let horizontal = Mesh([Polygon([Vertex(Vector(-1, -1, 0)), Vertex(Vector(1, -1, 0)), Vertex(Vector(0, 1, 0))])!])
        let vertical = Mesh([Polygon([Vertex(Vector(0, -0.5, -1)), Vertex(Vector(0, -0.5, 1)), Vertex(Vector(0, 0.5, 1))])!])
        XCTAssertEqual(MeshCollisionQuery.checkSurfaces(horizontal, vertical), .surfaceIntersection)
        XCTAssertEqual(MeshCollisionQuery.checkSurfaces(horizontal, vertical.translated(by: Vector(3, 0, 0))), .clear)
        XCTAssertEqual(MeshCollisionQuery.checkSurfaces(horizontal, horizontal), .clear)
        XCTAssertEqual(MeshCollisionQuery.checkSurfaces(horizontal, vertical, isCancelled: { true }), .timedOut)
        let world = RigCollisionWorld(parts: [part("horizontal", owner: nil, mesh: horizontal), part("vertical", owner: "arm", mesh: vertical)], joints: [], moving: ["arm"])
        var away = matrix_identity_float4x4
        away.columns.3.x = 3
        let result = world.check(poses: [["arm": away], [:]], timeLimit: 1)
        XCTAssertEqual(result.accepted, 0)
        XCTAssertEqual(Set(result.blockedParts), ["horizontal", "vertical"])
        XCTAssertTrue(world.unsupported.isEmpty)
        XCTAssertEqual(world.surfaceCount, 2)
    }

    private func part(_ id: String, owner: String?, mesh: Mesh) -> RigCollisionWorld.Part {
        let vertices = mesh.triangulate().polygons.flatMap { polygon in
            polygon.vertices.map { SIMD3<Float>(Float($0.position.x), Float($0.position.y), Float($0.position.z)) }
        }
        return .init(id: id, name: id, owner: owner, vertices: vertices)
    }

    func testRigWorldStopsIntermediateCollisionAndPreservesLastPose() {
        let world = RigCollisionWorld(parts: [part("moving", owner: "arm", mesh: .cube(size: 0.5)),
                                             part("body", owner: nil, mesh: .cube(center: Vector(2, 0, 0), size: 0.5))],
                                      joints: [], moving: ["arm"])
        let poses = (0...16).map { step in
            var transform = matrix_identity_float4x4
            transform.columns.3.x = Float(step) / 4
            return ["arm": transform]
        }
        let result = world.check(poses: poses, timeLimit: 2)
        XCTAssertLessThan(result.accepted, poses.count - 1)
        XCTAssertGreaterThan(result.accepted, 0)
        XCTAssertEqual(Set(result.blockedParts), ["moving", "body"])
        let sameGroup = RigCollisionWorld(parts: [part("moving", owner: "arm", mesh: .cube()),
                                                 part("attached", owner: "arm", mesh: .cube())], joints: [], moving: ["arm"])
        XCTAssertEqual(sameGroup.check(poses: poses, timeLimit: 2).accepted, poses.count - 1)
    }

    func testRigWorldReportsUnsupportedRatherThanFreezing() {
        let world = RigCollisionWorld(parts: [.init(id: "empty", name: "empty", owner: "arm", vertices: [])], joints: [], moving: ["arm"])
        let result = world.check(poses: [[:], [:]], timeLimit: 1)
        XCTAssertEqual(result.accepted, 1)
        XCTAssertTrue(result.message?.contains("unsupported") == true)
    }

    func testRigWorldJointMotionAndExistingOverlapEscape() {
        let upper = Mesh.cube(center: Vector(-1, 0, 0), size: Vector(2.1, 0.2, 0.2))
        let lower = Mesh.cube(center: Vector(1, 0, 0), size: Vector(2.1, 0.2, 0.2))
        let world = RigCollisionWorld(parts: [part("upper", owner: "base", mesh: upper), part("lower", owner: "hinge", mesh: lower)],
                                      joints: [.init(id: "hinge", parent: "base", pivot: .zero)], moving: ["hinge"])
        let bent = simd_float4x4(simd_quatf(angle: .pi / 3, axis: SIMD3(0, 0, 1)))
        XCTAssertEqual(world.check(poses: [[:], ["hinge": bent]], timeLimit: 2).accepted, 1)
        let folded = simd_float4x4(simd_quatf(angle: .pi, axis: SIMD3(0, 0, 1)))
        XCTAssertEqual(world.check(poses: [[:], ["hinge": folded]], timeLimit: 2).accepted, 0)
        let overlapping = RigCollisionWorld(parts: [part("first", owner: "arm", mesh: .cube()),
                                                    part("second", owner: nil, mesh: .cube(center: Vector(0.75, 0, 0), size: 1))], joints: [], moving: ["arm"])
        var away = matrix_identity_float4x4
        away.columns.3.x = -0.2
        XCTAssertEqual(overlapping.check(poses: [[:], ["arm": away]], timeLimit: 2).accepted, 1)
        away.columns.3.x = 0.2
        XCTAssertEqual(overlapping.check(poses: [[:], ["arm": away]], timeLimit: 2).accepted, 0)
    }

    func testAutomaticJointRegionDoesNotExemptWholeLinks() {
        let upper = Mesh.cube(center: Vector(-1, 0, 0), size: Vector(2.1, 0.2, 0.2))
        let lower = Mesh.cube(center: Vector(1, 0, 0), size: Vector(2.1, 0.2, 0.2))
        let region = MeshCollisionQuery.jointRegion(upper, lower, pivot: .zero, timeLimit: 1)
        XCTAssertNotNil(region)
        let bent = lower.rotated(by: .roll(.degrees(60)))
        XCTAssertEqual(MeshCollisionQuery.check(upper, bent, timeLimit: 1, ignoring: region), .clear)
        let folded = lower.rotated(by: .roll(.degrees(180)))
        guard case .overlap = MeshCollisionQuery.check(upper, folded, timeLimit: 1, ignoring: region) else {
            return XCTFail("Adjacent links must still collide away from the joint")
        }
        XCTAssertNil(MeshCollisionQuery.jointRegion(upper, lower, pivot: Vector(10, 0, 0), timeLimit: 1))
    }

    func testBracketAllowsRotationDespiteOverlappingBounds() {
        let bracket = Mesh.cube(size: Vector(4, 4, 1)).subtracting(.cube(size: Vector(3, 3, 2))).makeWatertight()
        XCTAssertTrue(bracket.isWatertight)
        XCTAssertGreaterThan(bracket.signedVolume, 0)
        let link = Mesh.cube(size: Vector(2, 0.3, 0.3)).triangulate()
        let started = Date()
        for degrees in stride(from: 0, through: 180, by: 5) {
            let rotated = link.rotated(by: .roll(.degrees(Double(degrees))))
            XCTAssertTrue(bracket.bounds.intersects(rotated.bounds))
            XCTAssertEqual(MeshCollisionQuery.check(bracket, rotated, timeLimit: 1), .clear)
        }
        print("37 bracket rotation queries: \(Date().timeIntervalSince(started)) seconds")
        let blocked = link.translated(by: Vector(1.5, 0, 0))
        guard case .overlap = MeshCollisionQuery.check(bracket, blocked, timeLimit: 1) else {
            return XCTFail("Link entering the bracket wall must collide")
        }
    }

    func testTouchingFacesAreNotPenetration() {
        let first = Mesh.cube(size: 1)
        let second = first.translated(by: Vector(1, 0, 0))
        XCTAssertEqual(MeshCollisionQuery.check(first, second, timeLimit: 1), .clear)
    }

    func testOpenMeshIsUnsupported() {
        let cube = Mesh.cube(size: 1)
        let open = Mesh(Array(cube.polygons.dropLast()))
        XCTAssertEqual(MeshCollisionQuery.check(open, cube, timeLimit: 1), .unsupportedMesh)
    }

    func testContainedSolidIsNotClear() {
        let outer = Mesh.cube(size: 4)
        let inner = Mesh.cube(size: 1)
        guard case let .overlap(volume) = MeshCollisionQuery.check(outer, inner, timeLimit: 1) else {
            return XCTFail("Containment must count as collision")
        }
        XCTAssertEqual(volume, 1, accuracy: 1e-8)
    }

    func testSurfaceContainmentAndHollowSolid() {
        let outer = Mesh.cube(size: 4).triangulate()
        let inner = Mesh.cube(size: 1).triangulate()
        XCTAssertEqual(MeshCollisionQuery.checkSurfaces(outer, inner), .surfaceIntersection)
        XCTAssertEqual(MeshCollisionQuery.checkSurfaces(inner, outer), .surfaceIntersection)
        XCTAssertEqual(MeshCollisionQuery.checkSurfaces(inner, inner.translated(by: Vector(1, 0, 0))), .clear)
        let bracket = outer.subtracting(.cube(size: Vector(3, 3, 6))).makeWatertight()
        XCTAssertEqual(MeshCollisionQuery.checkSurfaces(bracket, inner), .clear)
        XCTAssertEqual(MeshCollisionQuery.checkSurfaces(bracket, inner.translated(by: Vector(1.5, 0, 0))), .surfaceIntersection)
    }

    func testDetailedClosedMeshesUseIntersectionQueries() {
        let outer = Mesh.sphere(radius: 2, slices: 64).triangulate()
        let inner = Mesh.cube(size: 0.5)
        XCTAssertGreaterThan(outer.polygons.count, 1_000)
        let world = RigCollisionWorld(parts: [part("outer", owner: nil, mesh: outer), part("inner", owner: "arm", mesh: inner)],
                                      joints: [], moving: ["arm"])
        XCTAssertEqual(world.surfaceCount, 0)
        let result = world.check(poses: [[:], [:]], timeLimit: 2)
        XCTAssertEqual(Set(result.blockedParts), ["outer", "inner"])
        XCTAssertTrue(result.message?.contains("Surface collision") == true)
    }

    func testSeparatedMeshesAreClear() {
        let first = Mesh.cube(size: 1)
        let second = first.translated(by: Vector(3, 0, 0))
        XCTAssertEqual(MeshCollisionQuery.check(first, second, timeLimit: 1), .clear)
    }

    func testInvalidBudgetAndEmptyMeshAreNotClear() {
        XCTAssertEqual(MeshCollisionQuery.check(.cube(), .cube(), timeLimit: 0), .timedOut)
        XCTAssertEqual(MeshCollisionQuery.check(.empty, .cube(), timeLimit: 1), .unsupportedMesh)
        XCTAssertEqual(MeshCollisionQuery.check(.cube(), .cube(), timeLimit: 1, isCancelled: { true }), .timedOut)
        XCTAssertEqual(MeshCollisionQuery.check(Mesh.cube().inverted(), .cube(), timeLimit: 1), .unsupportedMesh)
    }

    func testSmallSolidContainmentUsesRelativeVolumeTolerance() {
        let outer = Mesh.cube(size: 0.001)
        let inner = Mesh.cube(size: 0.0001)
        guard case let .overlap(volume) = MeshCollisionQuery.check(outer, inner, timeLimit: 1) else {
            return XCTFail("Small contained solids must not fall below an absolute volume threshold")
        }
        XCTAssertEqual(volume, 1e-12, accuracy: 1e-15)
    }
}