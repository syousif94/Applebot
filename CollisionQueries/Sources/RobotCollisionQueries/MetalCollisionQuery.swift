import Euclid
import Foundation
import Metal
import simd

final class MetalCollisionQuery: @unchecked Sendable {
    struct Surface {
        let mesh: Mesh
        let closed: Bool
    }

    struct Pair {
        let first: Int
        let second: Int
        let firstToSecond: simd_float4x4
        var contactSphere = SIMD4<Float>(0, 0, 0, -1)
    }

    private struct Triangle {
        let first: SIMD4<Float>
        let second: SIMD4<Float>
        let third: SIMD4<Float>
        var center: SIMD3<Float> { (first.xyz + second.xyz + third.xyz) / 3 }
    }

    private struct Node {
        let minimum: SIMD4<Float>
        let maximum: SIMD4<Float>
        var range: SIMD4<UInt32>
    }

    private struct SurfaceRange {
        let triangles: Range<Int>
        let root: Int
        let closed: Bool
    }

    private struct Job {
        let transform: simd_float4x4
        let sphere: SIMD4<Float>
        let source: SIMD4<UInt32>
        let destination: SIMD4<UInt32>
    }

    private struct Context {
        let device: MTLDevice
        let pipeline: MTLComputePipelineState
    }

    private static let context: Context? = {
        guard let device = MTLCreateSystemDefaultDevice(),
              let url = Bundle.module.url(forResource: "Collision", withExtension: "metal"),
              let source = try? String(contentsOf: url),
              let library = try? device.makeLibrary(source: source, options: nil),
              let function = library.makeFunction(name: "collide"),
              let pipeline = try? device.makeComputePipelineState(function: function) else { return nil }
        return Context(device: device, pipeline: pipeline)
    }()

    private let context: Context
    private let queue: MTLCommandQueue
    private let triangles: MTLBuffer
    private let nodes: MTLBuffer
    private let surfaces: [SurfaceRange]
    private let available = DispatchSemaphore(value: 1)

    init?(surfaces: [Surface]) {
        guard let context = Self.context, let queue = context.device.makeCommandQueue() else { return nil }
        var triangles: [Triangle] = []
        var nodes: [Node] = []
        var ranges: [SurfaceRange] = []
        func vector(_ point: Vector) -> SIMD4<Float> { SIMD4(Float(point.x), Float(point.y), Float(point.z), 0) }
        func appendTree(_ input: [Triangle]) {
            let nodeIndex = nodes.count
            var minimum = SIMD3<Float>(repeating: .infinity)
            var maximum = SIMD3<Float>(repeating: -.infinity)
            for triangle in input {
                minimum = simd_min(minimum, simd_min(triangle.first.xyz, simd_min(triangle.second.xyz, triangle.third.xyz)))
                maximum = simd_max(maximum, simd_max(triangle.first.xyz, simd_max(triangle.second.xyz, triangle.third.xyz)))
            }
            nodes.append(Node(minimum: SIMD4(minimum, 0), maximum: SIMD4(maximum, 0), range: .zero))
            if input.count <= 8 {
                let start = triangles.count
                triangles.append(contentsOf: input)
                nodes[nodeIndex].range = SIMD4(UInt32(start), UInt32(input.count), UInt32(nodes.count), 0)
            } else {
                let extent = maximum - minimum
                let axis = extent.x >= extent.y && extent.x >= extent.z ? 0 : (extent.y >= extent.z ? 1 : 2)
                let sorted = input.sorted { $0.center[axis] < $1.center[axis] }
                let midpoint = sorted.count / 2
                appendTree(Array(sorted[..<midpoint]))
                appendTree(Array(sorted[midpoint...]))
                nodes[nodeIndex].range.z = UInt32(nodes.count)
            }
        }
        for surface in surfaces {
            let start = triangles.count
            let root = nodes.count
            let input = surface.mesh.triangulate().polygons.map { polygon in
                Triangle(first: vector(polygon.vertices[0].position), second: vector(polygon.vertices[1].position), third: vector(polygon.vertices[2].position))
            }
            guard !input.isEmpty else { return nil }
            appendTree(input)
            ranges.append(SurfaceRange(triangles: start..<triangles.count, root: root, closed: surface.closed))
        }
        guard let triangleBuffer = Self.buffer(triangles, device: context.device),
              let nodeBuffer = Self.buffer(nodes, device: context.device) else { return nil }
        self.context = context
        self.queue = queue
        self.triangles = triangleBuffer
        self.nodes = nodeBuffer
        self.surfaces = ranges
    }

    private static func buffer<Element>(_ values: [Element], device: MTLDevice) -> MTLBuffer? {
        values.withUnsafeBytes { bytes in
            guard let address = bytes.baseAddress, !bytes.isEmpty else { return nil }
            return device.makeBuffer(bytes: address, length: bytes.count, options: .storageModeShared)
        }
    }

    func check(_ pairs: [Pair], timeLimit: Double) -> [MeshCollisionQuery.Result] {
        guard !pairs.isEmpty else { return [] }
        guard timeLimit.isFinite, timeLimit > 0, !Task.isCancelled else { return pairs.map { _ in .timedOut } }
        var jobs: [Job] = []
        var width = 0
        for (index, pair) in pairs.enumerated() {
            for reverse in [false, true] {
                let source = surfaces[reverse ? pair.second : pair.first]
                let destination = surfaces[reverse ? pair.first : pair.second]
                let transform = reverse ? pair.firstToSecond.inverse : pair.firstToSecond
                var sphere = pair.contactSphere
                if reverse && sphere.w >= 0 { sphere = SIMD4((transform * SIMD4(sphere.xyz, 1)).xyz, sphere.w) }
                jobs.append(Job(transform: transform, sphere: sphere,
                                source: SIMD4(UInt32(source.triangles.lowerBound), UInt32(source.triangles.count), UInt32(index), 0),
                                destination: SIMD4(UInt32(destination.root), destination.closed ? 1 : 0, 0, 0)))
                width = max(width, source.triangles.count)
            }
        }
        guard let jobBuffer = Self.buffer(jobs, device: context.device),
              let results = Self.buffer([UInt32](repeating: 0, count: pairs.count), device: context.device),
              let command = queue.makeCommandBuffer(), let encoder = command.makeComputeCommandEncoder() else {
            return pairs.map { _ in .unsupportedMesh }
        }
        encoder.setComputePipelineState(context.pipeline)
        encoder.setBuffer(triangles, offset: 0, index: 0)
        encoder.setBuffer(nodes, offset: 0, index: 1)
        encoder.setBuffer(jobBuffer, offset: 0, index: 2)
        encoder.setBuffer(results, offset: 0, index: 3)
        encoder.dispatchThreads(MTLSize(width: width, height: jobs.count, depth: 1),
                                threadsPerThreadgroup: MTLSize(width: context.pipeline.threadExecutionWidth, height: 1, depth: 1))
        encoder.endEncoding()
        guard available.wait(timeout: .now()) == .success else { return pairs.map { _ in .timedOut } }
        let completion = DispatchSemaphore(value: 0)
        let available = self.available
        command.addCompletedHandler { _ in
            available.signal()
            completion.signal()
        }
        command.commit()
        guard completion.wait(timeout: .now() + timeLimit) == .success, !Task.isCancelled else { return pairs.map { _ in .timedOut } }
        guard command.status == .completed else { return pairs.map { _ in .unsupportedMesh } }
        let flags = results.contents().bindMemory(to: UInt32.self, capacity: pairs.count)
        return pairs.indices.map { flags[$0] == 0 ? .clear : (flags[$0] & 1 != 0 ? .surfaceIntersection : .unsupportedMesh) }
    }
}

private extension SIMD4 where Scalar == Float {
    var xyz: SIMD3<Float> { SIMD3(x, y, z) }
}