import CMeshSimplifier
import Euclid

enum CollisionMesh {
    struct Proxy {
        let mesh: Mesh
        let relativeError: Float
    }

    static func simplify(_ source: Mesh, targetTriangles: Int = 1_000, errorLimit: Float = 0.001) -> Proxy {
        guard source.polygons.count > targetTriangles, targetTriangles > 0,
              errorLimit.isFinite, errorLimit >= 0, !Task.isCancelled else {
            return Proxy(mesh: source, relativeError: 0)
        }
        let triangles = source.triangulate().polygons
        var lookup: [Vector: UInt32] = [:]
        var vertices: [Vector] = []
        var positions: [Float] = []
        var indices: [UInt32] = []
        for polygon in triangles {
            for vertex in polygon.vertices {
                let point = vertex.position
                if let index = lookup[point] { indices.append(index) }
                else {
                    let index = UInt32(vertices.count)
                    lookup[point] = index
                    vertices.append(point)
                    positions.append(contentsOf: [Float(point.x), Float(point.y), Float(point.z)])
                    indices.append(index)
                }
            }
        }
        guard positions.allSatisfy(\.isFinite), !Task.isCancelled else { return Proxy(mesh: source, relativeError: 0) }
        var output = [UInt32](repeating: 0, count: indices.count)
        var error: Float = 0
        let count = meshopt_simplify(&output, indices, indices.count, positions, vertices.count, 12,
                                    targetTriangles * 3, errorLimit, UInt32(meshopt_SimplifyLockBorder), &error)
        guard count > 0, count < indices.count, count.isMultiple(of: 3), error.isFinite, error <= errorLimit,
              !Task.isCancelled else { return Proxy(mesh: source, relativeError: 0) }
        let polygons = stride(from: 0, to: count, by: 3).compactMap { offset in
            Polygon((offset..<(offset + 3)).map { Vertex(vertices[Int(output[$0])]) })
        }
        let reduced = Mesh(polygons)
        guard polygons.count == count / 3, !reduced.isEmpty,
              !source.isWatertight || reduced.isWatertight,
              source.signedVolume <= 0 || reduced.signedVolume > 0 else { return Proxy(mesh: source, relativeError: 0) }
        return Proxy(mesh: reduced, relativeError: error)
    }
}