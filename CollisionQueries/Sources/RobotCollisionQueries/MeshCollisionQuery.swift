import Euclid
import Foundation
import simd

public struct MeshCollisionQuery {
    static func transformedBounds(_ bounds: Bounds, by transform: simd_float4x4) -> Bounds {
        let minimum = SIMD3(bounds.min.x, bounds.min.y, bounds.min.z)
        let maximum = SIMD3(bounds.max.x, bounds.max.y, bounds.max.z)
        let center = (minimum + maximum) * 0.5
        let extent = (maximum - minimum) * 0.5
        let horizontal = SIMD3<Double>(Double(transform.columns.0.x), Double(transform.columns.0.y), Double(transform.columns.0.z))
        let vertical = SIMD3<Double>(Double(transform.columns.1.x), Double(transform.columns.1.y), Double(transform.columns.1.z))
        let depth = SIMD3<Double>(Double(transform.columns.2.x), Double(transform.columns.2.y), Double(transform.columns.2.z))
        let translation = SIMD3<Double>(Double(transform.columns.3.x), Double(transform.columns.3.y), Double(transform.columns.3.z))
        let transformedCenter = horizontal * center.x + vertical * center.y + depth * center.z + translation
        let transformedExtent = simd_abs(horizontal) * extent.x + simd_abs(vertical) * extent.y + simd_abs(depth) * extent.z
        var magnitude: SIMD3<Double> = simd_abs(horizontal) * (abs(center.x) + extent.x)
        magnitude += simd_abs(vertical) * (abs(center.y) + extent.y)
        magnitude += simd_abs(depth) * (abs(center.z) + extent.z)
        magnitude += simd_abs(translation)
        let padding = simd_max(magnitude, SIMD3(repeating: 1)) * (Double(Float.ulpOfOne) * 8)
        let lower = transformedCenter - transformedExtent - padding
        let upper = transformedCenter + transformedExtent + padding
        return Bounds(min: Vector(lower.x, lower.y, lower.z), max: Vector(upper.x, upper.y, upper.z))
    }

    fileprivate struct SurfacePolygon: Sendable {
        let polygon: Polygon
        let bounds: Bounds
        init(_ polygon: Polygon) {
            self.polygon = polygon
            bounds = polygon.bounds
        }
    }

    fileprivate indirect enum SurfaceTree: Sendable {
        case leaf(Bounds, [SurfacePolygon])
        case branch(Bounds, SurfaceTree, SurfaceTree)

        var bounds: Bounds {
            switch self {
            case let .leaf(bounds, _), let .branch(bounds, _, _): return bounds
            }
        }

        init?(_ polygons: [Polygon], isCancelled: () -> Bool) {
            self.init(polygons.map(SurfacePolygon.init), isCancelled: isCancelled)
        }

        private init?(_ polygons: [SurfacePolygon], isCancelled: () -> Bool) {
            guard !isCancelled() else { return nil }
            var minimum = SIMD3<Double>(repeating: .infinity)
            var maximum = SIMD3<Double>(repeating: -.infinity)
            for polygon in polygons {
                let lower = polygon.bounds.min
                let upper = polygon.bounds.max
                minimum = simd_min(minimum, SIMD3(lower.x, lower.y, lower.z))
                maximum = simd_max(maximum, SIMD3(upper.x, upper.y, upper.z))
            }
            let bounds = Bounds(min: Vector(minimum.x, minimum.y, minimum.z), max: Vector(maximum.x, maximum.y, maximum.z))
            guard polygons.count > 8 else { self = .leaf(bounds, polygons); return }
            let size = bounds.size
            func center(_ polygon: SurfacePolygon) -> Double {
                let center = polygon.bounds.center
                return size.x >= size.y && size.x >= size.z ? center.x : (size.y >= size.z ? center.y : center.z)
            }
            let sorted = polygons.sorted { center($0) < center($1) }
            let midpoint = sorted.count / 2
            guard let first = SurfaceTree(Array(sorted[..<midpoint]), isCancelled: isCancelled),
                  let second = SurfaceTree(Array(sorted[midpoint...]), isCancelled: isCancelled) else { return nil }
            self = .branch(bounds, first, second)
        }

        func query(_ polygon: SurfacePolygon, isCancelled: () -> Bool, crosses: (Polygon, Polygon) -> Bool) -> Result {
            guard !isCancelled() else { return .timedOut }
            guard bounds.intersects(polygon.bounds) else { return .clear }
            switch self {
            case let .leaf(_, candidates):
                for candidate in candidates where polygon.bounds.intersects(candidate.bounds) {
                    guard !isCancelled() else { return .timedOut }
                    if crosses(polygon.polygon, candidate.polygon) || crosses(candidate.polygon, polygon.polygon) { return .surfaceIntersection }
                }
                return .clear
            case let .branch(_, first, second):
                let result = first.query(polygon, isCancelled: isCancelled, crosses: crosses)
                return result == .clear ? second.query(polygon, isCancelled: isCancelled, crosses: crosses) : result
            }
        }

        func intersections(with segment: LineSegment, isCancelled: () -> Bool, hits: inout [Double]) -> Bool {
            guard !isCancelled() else { return false }
            guard bounds.intersects(segment.bounds) else { return true }
            switch self {
            case let .leaf(_, polygons):
                for polygon in polygons {
                    if let point = segment.intersection(with: polygon.polygon) {
                        hits.append((point - segment.start).length)
                    }
                }
                return true
            case let .branch(_, first, second):
                return first.intersections(with: segment, isCancelled: isCancelled, hits: &hits)
                    && second.intersections(with: segment, isCancelled: isCancelled, hits: &hits)
            }
        }

        func contains(_ point: Vector, isCancelled: () -> Bool) -> Bool? {
            guard bounds.intersects(point) else { return false }
            let direction = Vector(1, 0.371390676, 0.694746591).normalized()
            guard let ray = LineSegment(start: point, end: point + direction * (bounds.size.length * 2 + 1)) else { return false }
            var hits: [Double] = []
            guard intersections(with: ray, isCancelled: isCancelled, hits: &hits) else { return nil }
            let sorted = hits.sorted()
            if let first = sorted.first, first < 1e-8 { return false }
            var previous = -Double.infinity
            var count = 0
            for distance in sorted where distance - previous > 1e-8 {
                count += 1
                previous = distance
            }
            return !count.isMultiple(of: 2)
        }

        func visit(overlapping destination: Bounds, transform: simd_float4x4,
                   isCancelled: () -> Bool, query: (Polygon) -> Result) -> Result {
            if isCancelled() { return .timedOut }
            guard transformedBounds(bounds, by: transform).intersects(destination) else { return .clear }
            switch self {
            case let .leaf(_, polygons):
                for polygon in polygons {
                    if isCancelled() { return .timedOut }
                    let result = query(polygon.polygon)
                    if result != .clear { return result }
                }
                return .clear
            case let .branch(_, first, second):
                let result = first.visit(overlapping: destination, transform: transform, isCancelled: isCancelled, query: query)
                return result == .clear ? second.visit(overlapping: destination, transform: transform, isCancelled: isCancelled, query: query) : result
            }
        }
    }

    public enum Result: Equatable, Sendable {
        case clear
        case overlap(volume: Double)
        case surfaceIntersection
        case unsupportedMesh
        case timedOut
    }

    struct PreparedSurface: Sendable {
        fileprivate let mesh: Mesh
        fileprivate let tree: SurfaceTree?
        fileprivate let closed: Bool
        init(mesh: Mesh, closed: Bool) {
            self.mesh = mesh
            self.closed = closed
            tree = SurfaceTree(mesh.polygons, isCancelled: { Task.isCancelled })
        }
    }

    static func checkPrepared(_ first: PreparedSurface, _ second: PreparedSurface,
                              firstToSecond: simd_float4x4,
                              contactSphere: (center: Vector, radius: Double)? = nil,
                              isCancelled: @Sendable () -> Bool) -> Result {
        guard let firstTree = first.tree, let secondTree = second.tree else { return .timedOut }
        func point(_ point: Vector, by transform: simd_float4x4) -> Vector {
            let result = transform * SIMD4(Float(point.x), Float(point.y), Float(point.z), 1)
            return Vector(Double(result.x), Double(result.y), Double(result.z))
        }
        func allowed(_ point: Vector) -> Bool {
            guard let sphere = contactSphere else { return false }
            return (point - sphere.center).length <= sphere.radius
        }
        func crosses(_ source: Polygon, _ destination: Polygon) -> Bool {
            for index in source.vertices.indices {
                let start = source.vertices[index].position
                let end = source.vertices[(index + 1) % source.vertices.count].position
                let startSide = destination.plane.normal.dot(start) - destination.plane.w
                let endSide = destination.plane.normal.dot(end) - destination.plane.w
                guard startSide * endSide < -1e-14,
                      let edge = LineSegment(start: start, end: end),
                      let hit = edge.intersection(with: destination) else { continue }
                if !allowed(hit) { return true }
            }
            return false
        }
        let forward = firstTree.visit(overlapping: secondTree.bounds, transform: firstToSecond, isCancelled: isCancelled) { polygon in
            let vertices = polygon.vertices.map { Vertex(point($0.position, by: firstToSecond)) }
            guard Bounds(vertices.map(\.position)).intersects(secondTree.bounds) else { return .clear }
            guard let transformed = Polygon(vertices) else { return .unsupportedMesh }
            let result = secondTree.query(SurfacePolygon(transformed), isCancelled: isCancelled, crosses: crosses)
            if result != .clear { return result }
            if second.closed {
                let center = vertices.reduce(Vector.zero) { $0 + $1.position } / Double(vertices.count)
                if !allowed(center) {
                    guard let inside = secondTree.contains(center, isCancelled: isCancelled) else { return .timedOut }
                    if inside { return .surfaceIntersection }
                }
            }
            return .clear
        }
        if forward != .clear { return forward }
        if first.closed {
            let inverse = firstToSecond.inverse
            return secondTree.visit(overlapping: firstTree.bounds, transform: inverse, isCancelled: isCancelled) { polygon in
                let center = polygon.vertices.reduce(Vector.zero) { $0 + $1.position } / Double(polygon.vertices.count)
                if allowed(center) { return .clear }
                guard let inside = firstTree.contains(point(center, by: inverse), isCancelled: isCancelled) else { return .timedOut }
                return inside ? .surfaceIntersection : .clear
            }
        }
        return .clear
    }

    public static func check(_ first: Mesh, _ second: Mesh, timeLimit: TimeInterval = 0.05,
                             ignoring jointRegion: Mesh? = nil,
                             isCancelled: @Sendable () -> Bool = { false }) -> Result {
        guard timeLimit.isFinite, timeLimit > 0 else { return .timedOut }
        let clock = ContinuousClock()
        let started = clock.now
        @Sendable func stopped() -> Bool { isCancelled() || started.duration(to: clock.now) >= .seconds(timeLimit) }
        guard !stopped() else { return .timedOut }
        guard !first.isEmpty, !second.isEmpty, first.isWatertight, second.isWatertight,
              !first.isPlanar, !second.isPlanar,
              first.signedVolume > 0, second.signedVolume > 0 else { return .unsupportedMesh }
        guard !stopped() else { return .timedOut }
        guard first.bounds.intersects(second.bounds) else { return .clear }
        var intersection = Mesh(first.polygons).intersection(Mesh(second.polygons), isCancelled: stopped)
        guard !stopped() else { return .timedOut }
        if let jointRegion, !intersection.isEmpty {
            intersection = intersection.subtracting(Mesh(jointRegion.polygons), isCancelled: stopped)
            guard !stopped() else { return .timedOut }
        }
        let volume = abs(intersection.signedVolume)
        guard volume.isFinite else { return .unsupportedMesh }
        let tolerance = min(first.signedVolume, second.signedVolume) * 1e-10
        return volume > tolerance ? .overlap(volume: volume) : .clear
    }

    public static func jointRegion(_ first: Mesh, _ second: Mesh, pivot: Vector,
                                   timeLimit: TimeInterval = 0.1) -> Mesh? {
        let radius = min(first.bounds.size.length, second.bounds.size.length) * 0.1
        guard radius.isFinite, radius > 1e-5, first.isWatertight, second.isWatertight,
              first.signedVolume > 0, second.signedVolume > 0 else { return nil }
        let clock = ContinuousClock()
        let start = clock.now
        let overlap = Mesh(first.polygons).intersection(Mesh(second.polygons), isCancelled: {
            start.duration(to: clock.now) >= .seconds(timeLimit)
        })
        guard start.duration(to: clock.now) < .seconds(timeLimit), !overlap.isEmpty,
              overlap.polygons.flatMap(\.vertices).allSatisfy({ ($0.position - pivot).length <= radius }) else { return nil }
        return Mesh.sphere(radius: 1, slices: 16).scaled(by: radius).translated(by: pivot)
    }

    public static func checkSurfaces(_ first: Mesh, _ second: Mesh, ignoring region: Mesh? = nil,
                                     closed: (Bool, Bool)? = nil,
                                     contactSphere: (center: Vector, radius: Double)? = nil,
                                     isCancelled: @Sendable () -> Bool = { false }) -> Result {
        guard !first.isEmpty, !second.isEmpty else { return .unsupportedMesh }
        guard first.bounds.intersects(second.bounds) else { return .clear }
        guard let tree = SurfaceTree(second.polygons, isCancelled: isCancelled) else { return .timedOut }
        func allowed(_ point: Vector) -> Bool {
            if let sphere = contactSphere, (point - sphere.center).length <= sphere.radius { return true }
            return region?.intersects(point) == true
        }
        func crosses(_ source: Polygon, _ destination: Polygon) -> Bool {
            for index in source.vertices.indices {
                let start = source.vertices[index].position
                let end = source.vertices[(index + 1) % source.vertices.count].position
                let startSide = destination.plane.normal.dot(start) - destination.plane.w
                let endSide = destination.plane.normal.dot(end) - destination.plane.w
                guard startSide * endSide < -1e-14,
                      let edge = LineSegment(start: start, end: end),
                      let point = edge.intersection(with: destination) else { continue }
                if !allowed(point) { return true }
            }
            return false
        }
        for polygon in first.polygons {
            let result = tree.query(SurfacePolygon(polygon), isCancelled: isCancelled, crosses: crosses)
            if result != .clear { return result }
        }
        let firstClosed = closed?.0 ?? (first.isWatertight && !first.isPlanar && first.signedVolume > 0)
        let secondClosed = closed?.1 ?? (second.isWatertight && !second.isPlanar && second.signedVolume > 0)
        for (surface, solid, isClosed, existingTree) in [(first, second, secondClosed, tree as SurfaceTree?), (second, first, firstClosed, nil)] where isClosed {
            var points: [Vector] = []
            for polygon in surface.polygons {
                if isCancelled() { return .timedOut }
                let point = polygon.vertices.reduce(Vector.zero) { $0 + $1.position } / Double(polygon.vertices.count)
                if allowed(point) { continue }
                if solid.bounds.intersects(point) { points.append(point) }
            }
            if points.isEmpty { continue }
            guard let containment = existingTree ?? SurfaceTree(solid.polygons, isCancelled: isCancelled) else { return .timedOut }
            for point in points {
                guard let inside = containment.contains(point, isCancelled: isCancelled) else { return .timedOut }
                if inside { return .surfaceIntersection }
            }
        }
        return .clear
    }
}