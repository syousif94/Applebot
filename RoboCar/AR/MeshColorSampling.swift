import simd

enum MeshColorSampling {
    struct Quality {
        let incidence: Float
        let distance: Float
    }

    static let minimumIncidence: Float = 0.35
    static let incidenceImprovement: Float = 0.02

    static func quality(normal: SIMD3<Float>, towardCamera: SIMD3<Float>) -> Quality? {
        let normalLength = simd_length(normal)
        let distance = simd_length(towardCamera)
        guard normalLength.isFinite, normalLength > 0.0001,
              distance.isFinite, distance > 0.05 else { return nil }
        let incidence = simd_dot(normal / normalLength, towardCamera / distance)
        guard incidence.isFinite, incidence >= minimumIncidence else { return nil }
        return Quality(incidence: min(incidence, 1), distance: distance)
    }

    static func shouldReplace(_ previous: Quality?, with candidate: Quality) -> Bool {
        guard let previous else { return true }
        if candidate.incidence > previous.incidence + incidenceImprovement { return true }
        return candidate.incidence >= previous.incidence
            && candidate.distance < previous.distance * 0.9
    }

    static func project(
        cameraPoint: SIMD3<Float>,
        intrinsics: simd_float3x3,
        imageResolution: SIMD2<Float>,
        imageSize: SIMD2<Int>
    ) -> SIMD2<Int>? {
        let depth = -cameraPoint.z
        guard depth.isFinite, depth > 0.05,
              imageResolution.x.isFinite, imageResolution.y.isFinite,
              imageResolution.x > 0, imageResolution.y > 0,
              imageSize.x > 0, imageSize.y > 0 else { return nil }
        let horizontal = (intrinsics[0][0] * cameraPoint.x / depth + intrinsics[2][0])
            * Float(imageSize.x) / imageResolution.x
        let vertical = (intrinsics[2][1] - intrinsics[1][1] * cameraPoint.y / depth)
            * Float(imageSize.y) / imageResolution.y
        guard horizontal.isFinite, vertical.isFinite,
              horizontal >= 0, vertical >= 0,
              horizontal.rounded() < Float(imageSize.x),
              vertical.rounded() < Float(imageSize.y) else { return nil }
        return SIMD2(Int(horizontal.rounded()), Int(vertical.rounded()))
    }
}

struct MeshVertexColorCache {
    private struct Entry {
        var position: SIMD3<Float>
        var normal: SIMD3<Float>
        var quality: MeshColorSampling.Quality?
        var color = SIMD4<UInt8>(180, 180, 180, 255)
    }

    private var entries: [Entry] = []

    var colors: [UInt8]? {
        guard entries.contains(where: { $0.quality != nil }) else { return nil }
        return entries.flatMap { [$0.color.x, $0.color.y, $0.color.z, $0.color.w] }
    }

    mutating func reconcile(positions: [SIMD3<Float>], normals: [SIMD3<Float>]) {
        guard positions.count == normals.count else {
            entries.removeAll()
            return
        }
        guard entries.count == positions.count else {
            entries = zip(positions, normals).map { Entry(position: $0.0, normal: $0.1) }
            return
        }
        for index in entries.indices {
            let previous = entries[index]
            let displacement = simd_distance(previous.position, positions[index])
            let normalAgreement = simd_dot(simd_normalize(previous.normal), simd_normalize(normals[index]))
            if !displacement.isFinite || displacement > 0.01 || !normalAgreement.isFinite || normalAgreement < 0.98 {
                entries[index] = Entry(position: positions[index], normal: normals[index])
            }
        }
    }

    func accepts(_ quality: MeshColorSampling.Quality, at index: Int) -> Bool {
        MeshColorSampling.shouldReplace(entries[index].quality, with: quality)
    }

    mutating func record(
        color: SIMD4<UInt8>, quality: MeshColorSampling.Quality, at index: Int,
        position: SIMD3<Float>, normal: SIMD3<Float>
    ) {
        guard accepts(quality, at: index) else { return }
        entries[index] = Entry(position: position, normal: normal, quality: quality, color: color)
    }
}