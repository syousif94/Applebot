import Foundation
import simd

struct RobotRigTriangle {
    var first: SIMD3<Float>
    var second: SIMD3<Float>
    var third: SIMD3<Float>
    var element: Int
    var face: Int

    var cross: SIMD3<Float> { simd_cross(second - first, third - first) }
    var area: Float { simd_length(cross) * 0.5 }
    var center: SIMD3<Float> { (first + second + third) / 3 }
}

struct RobotRigSurface {
    var triangles: Set<Int>
    var axis: RobotRigAxis
}

struct RobotRigGeometry {
    let triangles: [RobotRigTriangle]
    let tolerance: Float
    let neighbors: [[Int]]

    private struct VertexKey: Hashable {
        let components: SIMD3<Int64>
    }
    private struct Edge: Hashable {
        let lower: Int
        let upper: Int
    }

    init(triangles: [RobotRigTriangle]) throws {
        guard !triangles.isEmpty, triangles.count <= 1_000_000 else { throw RobotRigError.invalid("Unsupported triangle count") }
        var lower = SIMD3<Float>(repeating: .greatestFiniteMagnitude)
        var upper = -lower
        for triangle in triangles {
            for point in [triangle.first, triangle.second, triangle.third] {
                guard point.isFinite, simd_length(point) < 1e10 else { throw RobotRigError.invalid("Invalid model coordinates") }
                lower = simd_min(lower, point)
                upper = simd_max(upper, point)
            }
        }
        let tolerance = max(simd_length(upper - lower) * 1e-6, 1e-8)
        self.tolerance = tolerance
        self.triangles = triangles
        var vertexIDs: [VertexKey: Int] = [:]
        var edges: [Edge: [Int]] = [:]
        for (index, triangle) in triangles.enumerated() {
            let ids = [triangle.first, triangle.second, triangle.third].map { point -> Int in
                let scaled = (point - lower) / tolerance
                let key = VertexKey(components: SIMD3(Int64(scaled.x.rounded()), Int64(scaled.y.rounded()), Int64(scaled.z.rounded())))
                if let existing = vertexIDs[key] { return existing }
                let next = vertexIDs.count
                vertexIDs[key] = next
                return next
            }
            for offset in 0..<3 {
                let start = ids[offset]
                let end = ids[(offset + 1) % 3]
                if start != end { edges[Edge(lower: min(start, end), upper: max(start, end)), default: []].append(index) }
            }
        }
        var neighbors = Array(repeating: [Int](), count: triangles.count)
        for faces in edges.values where faces.count == 2 && faces[0] != faces[1] {
            neighbors[faces[0]].append(faces[1])
            neighbors[faces[1]].append(faces[0])
        }
        self.neighbors = neighbors
    }

    func components() -> [[Int]] {
        var visited = Set<Int>()
        var result: [[Int]] = []
        for start in triangles.indices where !visited.contains(start) {
            var component = [start]
            visited.insert(start)
            var cursor = 0
            while cursor < component.count {
                for next in neighbors[component[cursor]] where visited.insert(next).inserted { component.append(next) }
                cursor += 1
            }
            result.append(component.sorted())
        }
        return result
    }

    func surface(at seed: Int, angularTolerance: Float = 1) throws -> RobotRigSurface {
        guard triangles.indices.contains(seed), angularTolerance.isFinite, (0.01...5).contains(angularTolerance) else {
            throw RobotRigError.invalid("Invalid face selection")
        }
        let base = triangles[seed]
        guard base.area > tolerance * tolerance else { throw RobotRigError.invalid("Degenerate face") }
        let normal = simd_normalize(base.cross)
        let threshold = cos(angularTolerance * .pi / 180)
        var accepted: Set<Int> = [seed]
        var checked: Set<Int> = [seed]
        var pending = [seed]
        var cursor = 0
        while cursor < pending.count {
            for next in neighbors[pending[cursor]] where checked.insert(next).inserted {
                let triangle = triangles[next]
                guard triangle.area > tolerance * tolerance,
                      simd_dot(normal, simd_normalize(triangle.cross)) >= threshold,
                      [triangle.first, triangle.second, triangle.third].allSatisfy({ abs(simd_dot($0 - base.first, normal)) <= tolerance * 4 }) else { continue }
                accepted.insert(next)
                pending.append(next)
            }
            cursor += 1
        }
        var total: Double = 0
        var weighted = SIMD3<Double>.zero
        for index in accepted {
            let triangle = triangles[index]
            let area = Double(triangle.area)
            weighted += SIMD3<Double>(triangle.center) * area
            total += area
        }
        guard total > 0 else { throw RobotRigError.invalid("Empty surface") }
        return RobotRigSurface(triangles: accepted, axis: RobotRigAxis(origin: SIMD3<Float>(weighted / total), direction: normal))
    }
}