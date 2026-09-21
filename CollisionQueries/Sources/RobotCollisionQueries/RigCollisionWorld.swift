import Euclid
import Foundation
import simd

public struct RigCollisionWorld: Sendable {
    public struct Part: Sendable {
        public let id: String
        public let name: String
        public let owner: String?
        public let vertices: [SIMD3<Float>]
        public init(id: String, name: String, owner: String?, vertices: [SIMD3<Float>]) {
            self.id = id; self.name = name; self.owner = owner; self.vertices = vertices
        }
    }

    public struct Joint: Sendable {
        public let id: String
        public let parent: String?
        public let pivot: SIMD3<Float>
        public let direction: SIMD3<Float>?
        public init(id: String, parent: String?, pivot: SIMD3<Float>, direction: SIMD3<Float>? = nil) {
            self.id = id; self.parent = parent; self.pivot = pivot; self.direction = direction
        }
    }

    public struct Outcome: Sendable {
        public let accepted: Int
        public let message: String?
        public let blockedParts: [String]
    }

    private struct Solid: Sendable {
        let part: Part
        let mesh: Mesh
        let closed: Bool
        let bounds: Bounds
    }
    private struct Contact: Sendable {
        let region: Mesh
        let owner: String?
    }
    private let solids: [Solid]
    private let moving: Set<String>
    private let joints: [Joint]
    private let scale: Float
    private let gpu: MetalCollisionQuery?
    public var usesGPU: Bool { gpu != nil }
    public let unsupported: [String]
    public let surfaceCount: Int
    public let triangleCounts: [String: Int]
    public let simplificationErrors: [String: Float]

    public init(parts: [Part], joints: [Joint], moving: Set<String>) {
        self.moving = moving
        self.joints = joints
        let extent = parts.flatMap(\.vertices).reduce(Float(0)) { max($0, simd_length($1)) }
        let scale: Float = extent > 0 ? 1 / extent : 1
        self.scale = scale
        var solids: [Solid] = []
        var unsupported: [String] = []
        var triangleCounts: [String: Int] = [:]
        var simplificationErrors: [String: Float] = [:]
        for part in parts {
            if Task.isCancelled { break }
            guard part.vertices.count <= 60_000, part.vertices.count.isMultiple(of: 3) else {
                unsupported.append(part.name); continue
            }
            var polygons: [Polygon] = []
            for offset in stride(from: 0, to: part.vertices.count, by: 3) {
                let points = part.vertices[offset..<(offset + 3)].map { point in
                    let normalized = point * scale
                    let welded = SIMD3((normalized.x * 1e6).rounded(), (normalized.y * 1e6).rounded(), (normalized.z * 1e6).rounded()) / 1e6
                    return Vector(Double(welded.x), Double(welded.y), Double(welded.z))
                }
                if let polygon = Polygon(points.map { Vertex($0) }) { polygons.append(polygon) }
            }
            var mesh = Mesh(polygons)
            if mesh.signedVolume < 0 { mesh = mesh.inverted() }
            guard !mesh.isEmpty else {
                unsupported.append(part.name); continue
            }
            let proxy = CollisionMesh.simplify(mesh, targetTriangles: 256)
            mesh = proxy.mesh
            triangleCounts[part.id] = mesh.polygons.count
            simplificationErrors[part.id] = proxy.relativeError
            let closed = mesh.isWatertight && !mesh.isPlanar && mesh.signedVolume > 0
            solids.append(Solid(part: part, mesh: mesh, closed: closed, bounds: mesh.bounds))
        }
        self.solids = solids
        self.unsupported = unsupported
        self.triangleCounts = triangleCounts
        self.simplificationErrors = simplificationErrors
        surfaceCount = solids.filter { !$0.closed }.count
        gpu = MetalCollisionQuery(surfaces: solids.map { .init(mesh: $0.mesh, closed: $0.closed) })
    }

    private func transformed(_ mesh: Mesh, by transform: simd_float4x4) -> Mesh {
        Mesh(mesh.polygons.compactMap { polygon in
            Polygon(polygon.vertices.map { vertex in
                let point = vertex.position
                let world = transform * SIMD4(Float(point.x) / scale, Float(point.y) / scale, Float(point.z) / scale, 1)
                return Vertex(Vector(Double(world.x * scale), Double(world.y * scale), Double(world.z * scale)))
            })
        })
    }

    public func check(poses: [[String: simd_float4x4]], timeLimit: Double = 0.25, measureOverlap: Bool = true) -> Outcome {
        if !measureOverlap { return checkInteractive(poses: poses, timeLimit: timeLimit) }
        let clock = ContinuousClock()
        var accepted = 0
        var baselines: [String: Double] = [:]
        var contacts: [String: Contact] = [:]
        var prepared = Set<String>()
        var meshTransforms = Array(repeating: matrix_identity_float4x4, count: solids.count)
        var meshes = solids.map(\.mesh)
        let warning = unsupported.isEmpty ? (surfaceCount > 0 ? "Surface checks: \(surfaceCount) open parts" : nil) : "Collision check incomplete: \(unsupported.count) unsupported parts (\(unsupported.prefix(3).joined(separator: ", ")))"
        for (poseIndex, pose) in poses.enumerated() {
            if Task.isCancelled || !timeLimit.isFinite || timeLimit <= 0 {
                return Outcome(accepted: accepted, message: "Collision check timed out; motion held", blockedParts: [])
            }
            let transforms = solids.map { $0.part.owner.flatMap { pose[$0] } ?? matrix_identity_float4x4 }
            let bounds = solids.indices.map { index -> Bounds in
                let bounds = solids[index].bounds
                var transform = transforms[index]
                if transform == matrix_identity_float4x4 { return bounds }
                transform.columns.3.x *= scale
                transform.columns.3.y *= scale
                transform.columns.3.z *= scale
                return MeshCollisionQuery.transformedBounds(bounds, by: transform)
            }
            let order = meshes.indices.sorted { bounds[$0].min.x < bounds[$1].min.x }
            for (offset, first) in order.enumerated() {
                for second in order.dropFirst(offset + 1) {
                    if Task.isCancelled { return Outcome(accepted: accepted, message: "Collision check cancelled; motion held", blockedParts: []) }
                    if bounds[second].min.x > bounds[first].max.x { break }
                    let firstOwner = solids[first].part.owner
                    let secondOwner = solids[second].part.owner
                    guard firstOwner != secondOwner,
                          firstOwner.map(moving.contains) == true || secondOwner.map(moving.contains) == true,
                          bounds[first].intersects(bounds[second]) else { continue }
                    let key = "\(min(first, second)):\(max(first, second))"
                    for index in [first, second] where measureOverlap && meshTransforms[index] != transforms[index] {
                        meshes[index] = transformed(solids[index].mesh, by: transforms[index])
                        meshTransforms[index] = transforms[index]
                    }
                    let useVolume = measureOverlap && solids[first].closed && solids[second].closed
                        && solids[first].mesh.polygons.count + solids[second].mesh.polygons.count <= 1_000
                    if measureOverlap, prepared.insert(key).inserted,
                       let joint = joints.first(where: {
                           ($0.id == firstOwner && $0.parent == secondOwner) || ($0.id == secondOwner && $0.parent == firstOwner)
                       }) {
                        let pivot = joint.pivot * scale
                        if useVolume, let region = MeshCollisionQuery.jointRegion(solids[first].mesh, solids[second].mesh,
                                                                      pivot: Vector(Double(pivot.x), Double(pivot.y), Double(pivot.z)), timeLimit: 0.02) {
                            contacts[key] = Contact(region: region, owner: joint.parent)
                        } else if !useVolume {
                            let center = Vector(Double(pivot.x), Double(pivot.y), Double(pivot.z))
                            let radius = min(solids[first].mesh.bounds.size.length, solids[second].mesh.bounds.size.length) * 0.1
                            if radius.isFinite, radius > 1e-5 {
                                let region = Mesh.sphere(radius: 1, slices: 16).scaled(by: radius).translated(by: center)
                                if region.bounds.intersects(solids[first].mesh.bounds) && region.bounds.intersects(solids[second].mesh.bounds) {
                                    contacts[key] = Contact(region: region, owner: joint.parent)
                                }
                            }
                        }
                    }
                    let region = contacts[key].map { transformed($0.region, by: $0.owner.flatMap { pose[$0] } ?? matrix_identity_float4x4) }
                    let sphere = contacts[key].map { contact in
                        let center = contact.region.bounds.center
                        let transform = contact.owner.flatMap { pose[$0] } ?? matrix_identity_float4x4
                        let point = transform * SIMD4(Float(center.x) / scale, Float(center.y) / scale, Float(center.z) / scale, 1)
                        return (center: Vector(Double(point.x * scale), Double(point.y * scale), Double(point.z * scale)), radius: contact.region.bounds.size.x / 2)
                    }
                    let start = clock.now
                    @Sendable func cancelled() -> Bool {
                        Task.isCancelled || start.duration(to: clock.now) >= .seconds(timeLimit)
                    }
                    let result: MeshCollisionQuery.Result
                    if useVolume {
                        result = MeshCollisionQuery.check(meshes[first], meshes[second], timeLimit: timeLimit, ignoring: region, isCancelled: cancelled)
                    } else {
                        result = MeshCollisionQuery.checkSurfaces(meshes[first], meshes[second],
                                                                 closed: (solids[first].closed, solids[second].closed),
                                                                 contactSphere: sphere, isCancelled: cancelled)
                    }
                    switch result {
                    case .clear: break
                    case let .overlap(volume):
                        if poseIndex == 0 { baselines[key] = volume; continue }
                        if let baseline = baselines[key], volume <= baseline { continue }
                        return Outcome(accepted: accepted,
                                       message: "Self-collision: \(solids[first].part.name) / \(solids[second].part.name)",
                                       blockedParts: [solids[first].part.id, solids[second].part.id])
                    case .unsupportedMesh:
                        return Outcome(accepted: accepted, message: "Unsupported collision geometry: \(solids[first].part.name) / \(solids[second].part.name); motion held", blockedParts: [])
                    case .timedOut:
                        return Outcome(accepted: accepted, message: "Collision query timed out: \(solids[first].part.name) / \(solids[second].part.name); motion held", blockedParts: [])
                    case .surfaceIntersection:
                        if poseIndex == 0 { continue }
                        return Outcome(accepted: accepted, message: "Surface collision: \(solids[first].part.name) / \(solids[second].part.name)",
                                       blockedParts: [solids[first].part.id, solids[second].part.id])
                    }
                }
            }
            accepted = poseIndex
        }
        let message = baselines.isEmpty ? warning : "Existing mesh overlap; only non-deepening motion accepted" + (warning.map { "; \($0)" } ?? "")
        return Outcome(accepted: accepted, message: message, blockedParts: [])
    }

    private func checkInteractive(poses: [[String: simd_float4x4]], timeLimit: Double) -> Outcome {
        var accepted = 0
        let started = ContinuousClock.now
        let warning = unsupported.isEmpty ? (surfaceCount > 0 ? "Surface checks: \(surfaceCount) open parts" : nil)
            : "Collision check incomplete: \(unsupported.count) unsupported parts (\(unsupported.prefix(3).joined(separator: ", ")))"
        guard let gpu else {
            return Outcome(accepted: 0, message: "GPU collision checks unavailable; motion held", blockedParts: [])
        }
        for (poseIndex, pose) in poses.enumerated() where poseIndex > 0 {
            let elapsed = started.duration(to: .now).components
            let remaining = timeLimit - Double(elapsed.seconds) - Double(elapsed.attoseconds) / 1e18
            guard remaining.isFinite, remaining > 0, !Task.isCancelled else {
                return Outcome(accepted: accepted, message: "GPU collision check timed out or cancelled; motion held", blockedParts: [])
            }
            let transforms = solids.map { solid -> simd_float4x4 in
                var transform = solid.part.owner.flatMap { pose[$0] } ?? matrix_identity_float4x4
                transform.columns.3.x *= scale
                transform.columns.3.y *= scale
                transform.columns.3.z *= scale
                return transform
            }
            guard transforms.allSatisfy({ transform in
                (0..<4).allSatisfy { column in (0..<4).allSatisfy { transform[column][$0].isFinite } }
                    && abs(simd_determinant(transform)) > 1e-8
            }) else {
                return Outcome(accepted: accepted, message: "Invalid collision transform; motion held", blockedParts: [])
            }
            let inverses = transforms.map { $0.inverse }
            let bounds = solids.indices.map { MeshCollisionQuery.transformedBounds(solids[$0].bounds, by: transforms[$0]) }
            let order = solids.indices.sorted { bounds[$0].min.x < bounds[$1].min.x }
            var pairs: [MetalCollisionQuery.Pair] = []
            for (offset, first) in order.enumerated() {
                for second in order.dropFirst(offset + 1) {
                    if bounds[second].min.x > bounds[first].max.x { break }
                    let firstOwner = solids[first].part.owner
                    let secondOwner = solids[second].part.owner
                    guard firstOwner != secondOwner,
                          firstOwner.map(moving.contains) == true || secondOwner.map(moving.contains) == true,
                          bounds[first].intersects(bounds[second]) else { continue }
                    var pair = MetalCollisionQuery.Pair(first: first, second: second, firstToSecond: inverses[second] * transforms[first])
                    if let firstOwner, let secondOwner, let joint = joints.first(where: {
                        ($0.id == firstOwner && $0.parent == secondOwner) || ($0.id == secondOwner && $0.parent == firstOwner)
                    }) {
                        let pivot = joint.pivot * scale
                        if let contact = Self.jointContactSphere(solids[first].bounds, solids[second].bounds, pivot: pivot, direction: joint.direction) {
                            let parent = joint.parent == firstOwner ? transforms[first] : transforms[second]
                            let local = inverses[second] * parent * SIMD4(contact.center, 1)
                            pair.contactSphere = SIMD4(local.x, local.y, local.z, contact.radius)
                        }
                    }
                    pairs.append(pair)
                }
            }
            let preparationElapsed = started.duration(to: .now).components
            let budget = timeLimit - Double(preparationElapsed.seconds) - Double(preparationElapsed.attoseconds) / 1e18
            guard budget > 0, !Task.isCancelled else {
                return Outcome(accepted: accepted, message: "GPU collision check timed out or cancelled; motion held", blockedParts: [])
            }
            let results = gpu.check(pairs, timeLimit: budget)
            for (pair, result) in zip(pairs, results) where result != .clear {
                if result == .surfaceIntersection {
                    return Outcome(accepted: accepted, message: "Self-collision: \(solids[pair.first].part.name) / \(solids[pair.second].part.name)",
                                   blockedParts: [solids[pair.first].part.id, solids[pair.second].part.id])
                }
                return Outcome(accepted: accepted, message: result == .timedOut ? "GPU collision check timed out; motion held" : "GPU collision query failed; motion held", blockedParts: [])
            }
            accepted = poseIndex
        }
        return Outcome(accepted: accepted, message: warning, blockedParts: [])
    }

    private static func jointContactSphere(_ first: Bounds, _ second: Bounds, pivot: SIMD3<Float>, direction: SIMD3<Float>?) -> (center: SIMD3<Float>, radius: Float)? {
        guard first.intersects(second) else { return nil }
        let lower = SIMD3<Float>(Float(max(first.min.x, second.min.x)), Float(max(first.min.y, second.min.y)), Float(max(first.min.z, second.min.z)))
        let upper = SIMD3<Float>(Float(min(first.max.x, second.max.x)), Float(min(first.max.y, second.max.y)), Float(min(first.max.z, second.max.z)))
        let tolerance = Float(min(first.size.length, second.size.length)) * 0.1
          var center = pivot
          if let direction, direction.x.isFinite, direction.y.isFinite, direction.z.isFinite,
             simd_length_squared(direction) > 1e-8 {
            let axis = simd_normalize(direction)
            center += axis * simd_dot((lower + upper) * 0.5 - pivot, axis)
          }
        guard tolerance.isFinite, tolerance > 1e-5,
              simd_distance(center, simd_clamp(center, lower, upper)) <= tolerance else { return nil }
          let farthest = simd_max(simd_abs(lower - center), simd_abs(upper - center))
          return (center, max(tolerance, simd_length(farthest) + tolerance * 0.05))
    }
}