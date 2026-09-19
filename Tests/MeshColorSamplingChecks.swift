import simd

@main
struct MeshColorSamplingChecks {
    static func main() {
        let normal = SIMD3<Float>(0, 0, 1)
        let position = SIMD3<Float>(0, 0, -1)
        let headOn = MeshColorSampling.quality(normal: normal, towardCamera: SIMD3(0, 0, 1))!
        let oblique = MeshColorSampling.quality(normal: normal, towardCamera: SIMD3(1, 0, 1))!
        let red = SIMD4<UInt8>(255, 0, 0, 255)
        let blue = SIMD4<UInt8>(0, 0, 255, 255)
        var cache = MeshVertexColorCache()
        cache.reconcile(positions: [position], normals: [normal])
        precondition(cache.colors == nil)
        cache.record(color: red, quality: headOn, at: 0, position: position, normal: normal)
        cache.record(color: blue, quality: oblique, at: 0, position: position, normal: normal)
        precondition(cache.colors == [255, 0, 0, 255], "An angled sample overwrote a head-on sample")
        var reversed = MeshVertexColorCache()
        reversed.reconcile(positions: [position], normals: [normal])
        reversed.record(color: blue, quality: oblique, at: 0, position: position, normal: normal)
        reversed.record(color: red, quality: headOn, at: 0, position: position, normal: normal)
        precondition(reversed.colors == cache.colors)
        precondition(!cache.accepts(headOn, at: 0))
        precondition(MeshColorSampling.quality(normal: normal, towardCamera: SIMD3(4, 0, 1)) == nil)
        precondition(MeshColorSampling.quality(normal: normal, towardCamera: -normal) == nil)
        precondition(MeshColorSampling.quality(normal: .zero, towardCamera: normal) == nil)
        precondition(MeshColorSampling.quality(normal: normal, towardCamera: SIMD3(.nan, 0, 1)) == nil)
        let closer = MeshColorSampling.quality(normal: normal, towardCamera: normal * 0.5)!
        precondition(cache.accepts(closer, at: 0))
        let closerOblique = MeshColorSampling.quality(normal: normal, towardCamera: SIMD3(0.1, 0, 0.1))!
        precondition(!cache.accepts(closerOblique, at: 0))
        print("PASS: best-view color selection, order independence, grazing/backface rejection")

        let intrinsics = simd_float3x3(columns: (SIMD3(800, 0, 0), SIMD3(0, 600, 0), SIMD3(470, 350, 1)))
        let resolution = SIMD2<Float>(1000, 800)
        let imageSize = SIMD2(500, 400)
        func project(_ point: SIMD3<Float>) -> SIMD2<Int>? {
            MeshColorSampling.project(cameraPoint: point, intrinsics: intrinsics,
                                      imageResolution: resolution, imageSize: imageSize)
        }
        precondition(project(position) == SIMD2(235, 175))
        precondition(project(SIMD3(0.1, 0.1, -1)) == SIMD2(275, 145))
        precondition(project(SIMD3(0, 0, 1)) == nil)
        precondition(project(SIMD3(.nan, 0, -1)) == nil)
        precondition(project(SIMD3(0, .infinity, -1)) == nil)
        precondition(project(SIMD3(100, 0, -1)) == nil)
        print("PASS: native pixel projection, axis signs, principal point, buffer scaling, invalid inputs")

        cache.reconcile(positions: [position + SIMD3(0.005, 0, 0)], normals: [normal])
        precondition(cache.colors == [255, 0, 0, 255])
        cache.reconcile(positions: [position + SIMD3(0.012, 0, 0)], normals: [normal])
        precondition(cache.colors == nil, "Small changes accumulated beyond the sampled position")
        reversed.reconcile(positions: [position], normals: [-normal])
        precondition(reversed.colors == nil)
        cache.reconcile(positions: [position, position], normals: [normal, normal])
        cache.record(color: red, quality: headOn, at: 0, position: position, normal: normal)
        precondition(cache.colors == [255, 0, 0, 255, 180, 180, 180, 255])
        cache.reconcile(positions: [position], normals: [normal])
        precondition(cache.colors == nil)
        cache.reconcile(positions: [], normals: [])
        precondition(cache.colors == nil)
        print("PASS: geometry invalidation, normal changes, count changes, exact RGBA size, empty cache")
    }
}