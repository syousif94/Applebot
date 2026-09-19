//
//  PersonTracker.swift
//  RoboCar
//
//  Created by Sammy Yousif on 3/17/26.
//

import Foundation
import Vision
import CoreML
import ARKit
import simd

/// Tracks all visible people with stable IDs using VNDetectHumanRectanglesRequest
/// and OSNet ReID embeddings. A person is selected for following externally
/// (e.g. by tapping their bounding box), then uses VNTrackObjectRequest for
/// efficient frame-to-frame tracking of that person.
///
/// Each detected person maintains a gallery of up to 4 diverse OSNet embeddings
/// rather than a single embedding. Re-ID matching uses max similarity across the
/// gallery and requires a ratio-test margin over the next best candidate, making
/// identity assignment robust to pose/lighting variation while resisting false
/// matches between different people.
///
/// Flow:
///   1. `startScanning()` — enters scanning mode. Every few frames, detects all
///      people and assigns stable IDs / ReID embeddings.
///   2. `activateExternalPerson(...)` or `activatePerson(id:frame:)` selects the
///      person to follow, seeds the target gallery, and starts a VN tracker.
///   3. `update(frame:)` — in scanning mode, runs multi-person detection. In
///      tracking mode, runs the VN tracker on the active person, with ReID
///      fallback if the tracker loses them.
///   4. `stopTracking()` — clears all state and returns to idle.
class PersonTracker {

    // MARK: - Singleton

    static let shared = PersonTracker()

    // MARK: - Types

    /// A single detected person with a stable session-level ID.
    struct DetectedPerson {
        let id: UUID
        var boundingBox: CGRect          // Vision normalized coordinates
        var gallery: [[Float]]           // OSNet 512-dim embeddings, newest at end
        var worldPosition: simd_float2?  // (ARKit X, ARKit Z)
        var lastSeen: Date

        static let maxGallerySize = 4

        /// Last gallery entry — convenience for callers wanting a single vector.
        var embedding: [Float]? { gallery.last }

        /// Add an embedding to the gallery only if it is sufficiently different
        /// from all existing entries (diversity guard) so we accumulate distinct
        /// appearance samples rather than near-duplicate frames.
        mutating func addToGallery(_ embedding: [Float]) {
            let maxSim = gallery.map { PersonTracker.cosineSimilarity($0, embedding) }.max() ?? 0
            guard maxSim < 0.92 else { return }
            gallery.append(embedding)
            if gallery.count > Self.maxGallerySize { gallery.removeFirst() }
        }

        /// Max cosine similarity between `query` and any gallery entry.
        func bestSimilarity(to query: [Float]) -> Float {
            gallery.map { PersonTracker.cosineSimilarity($0, query) }.max() ?? 0
        }
    }

    enum TrackingState: String {
        case idle
        case scanning       // Detecting all people, waiting for activation gesture
        case tracking       // VN tracker is following the activated person
        case reacquiring    // Tracker lost — running detection + ReID
        case lost           // Could not re-acquire after lostFrameThreshold
    }

    // MARK: - Configuration

    /// Minimum cosine similarity to accept a ReID match.
    var reidMatchThreshold: Float = 0.62

    /// Ratio-test margin: best ReID sim must beat second-best by at least this.
    var reidMargin: Float = 0.08

    /// How many consecutive frames of lost tracking before announcing loss.
    var lostFrameThreshold: Int = 20  // ~1s at 20fps

    /// Standoff distance — how far from the person the car should stop (meters).
    var standoffDistance: Float = 0.8

    /// How often to add a new sample to the target gallery during active tracking.
    var embeddingRefreshInterval: TimeInterval = 2.0

    /// How many frames between full detection scans during scanning mode.
    var scanDetectionInterval: Int = 5

    /// IoU threshold for matching detections to existing tracked people.
    var iouMatchThreshold: CGFloat = 0.25

    /// Maximum time (seconds) before a person not re-detected is pruned.
    var personPruneTimeout: TimeInterval = 3.0

    // MARK: - State

    private(set) var state: TrackingState = .idle

    /// All currently detected/tracked people (updated during scanning).
    private(set) var detectedPeople: [DetectedPerson] = []

    /// The ID of the person being followed (activated by tap or by name).
    private(set) var activePersonID: UUID? = nil

    /// World position of the tracked person (ARKit X = x, ARKit Z = y), updated each frame.
    private(set) var trackedWorldPosition: simd_float2? = nil

    /// Bounding box of the tracked person in normalized image coordinates (Vision convention).
    private(set) var trackedBoundingBox: CGRect? = nil

    /// Gallery of appearance embeddings for the actively tracked person.
    private var targetGallery: [[Float]] = []

    static let targetGalleryMaxSize = 6

    /// VN tracker request for frame-to-frame tracking.
    private var trackRequest: VNTrackObjectRequest?

    /// Sequence request handler (reused across frames for tracker continuity).
    private var sequenceHandler = VNSequenceRequestHandler()

    /// The CoreML model for generating ReID embeddings.
    private var reidModel: VNCoreMLModel?

    /// Frame counter for lost tracking.
    private var lostFrameCount: Int = 0

    /// Frame counter for scanning detection cadence.
    private var scanFrameCount: Int = 0

    /// Last time a new sample was added to the target gallery.
    private var lastGallerySampleDate: Date = .distantPast

    /// Callbacks
    var onStateChanged: ((TrackingState) -> Void)?
    var onPositionUpdated: ((simd_float2) -> Void)?
    var onPeopleUpdated: (([DetectedPerson]) -> Void)?
    var onPersonActivated: ((DetectedPerson) -> Void)?
    var onLog: ((String) -> Void)?

    // MARK: - Init

    private init() {
        loadReIDModel()
    }

    private func loadReIDModel() {
        guard let modelURL = Bundle.main.url(forResource: "OSNetReID", withExtension: "mlmodelc")
                          ?? Bundle.main.url(forResource: "OSNetReID", withExtension: "mlpackage") else {
            log("⚠️ OSNetReID model not found in bundle")
            return
        }
        do {
            let mlModel = try MLModel(contentsOf: modelURL)
            reidModel = try VNCoreMLModel(for: mlModel)
            log("✅ OSNet ReID model loaded")
        } catch {
            log("⚠️ Failed to load OSNet model: \(error)")
        }
    }

    // MARK: - Public API

    /// Start scanning for people. Detects all visible people, assigns stable IDs,
    /// and watches for an open-palm gesture to activate following.
    func startScanning() {
        stopTracking()
        state = .scanning
        scanFrameCount = 0
        onStateChanged?(.scanning)
        log("👀 Scanning for people — tap a person to start following")
    }

    /// Legacy entry point: detect the closest person and immediately start tracking.
    /// Prefer `startScanning()` for gesture-based activation.
    func startTracking(frame: ARFrame) {
        stopTracking()

        let pixelBuffer = frame.capturedImage
        let handler = VNImageRequestHandler(cvPixelBuffer: pixelBuffer, orientation: .right, options: [:])
        let detectRequest = VNDetectHumanRectanglesRequest()

        do {
            try handler.perform([detectRequest])
        } catch {
            log("❌ Person detection failed: \(error)")
            return
        }

        guard let results = detectRequest.results, !results.isEmpty else {
            log("❌ No people detected in frame")
            return
        }

        let sorted = results.sorted { $0.boundingBox.area > $1.boundingBox.area }
        let bbox = sorted[0].boundingBox
        log("👤 Detected \(results.count) people — tracking largest (area: \(String(format: "%.3f", bbox.area)))")

        var gallery: [[Float]] = []
        if let emb = generateEmbedding(pixelBuffer: pixelBuffer, boundingBox: bbox) { gallery = [emb] }

        let person = DetectedPerson(
            id: UUID(),
            boundingBox: bbox,
            gallery: gallery,
            worldPosition: projectToWorld(boundingBox: bbox, frame: frame),
            lastSeen: Date()
        )
        activatePersonForTracking(person: person, pixelBuffer: pixelBuffer, frame: frame)
    }

    /// Update tracking with the current frame. Call every frame from updateFrame().
    func update(frame: ARFrame) {
        switch state {
        case .idle:
            return
        case .scanning:
            updateScanning(frame: frame)
        case .tracking, .reacquiring, .lost:
            updateActiveTracking(frame: frame)
        }
    }

    /// Manually activate a specific detected person by their ID.
    func activatePerson(id: UUID, frame: ARFrame) {
        guard let person = detectedPeople.first(where: { $0.id == id }) else {
            log("❌ No detected person with ID \(id.uuidString.prefix(8))")
            return
        }
        activatePersonForTracking(person: person, pixelBuffer: frame.capturedImage, frame: frame)
    }

    /// Activate tracking for a person identified externally (e.g. by the always-on scanner).
    /// Accepts the pre-computed appearance gallery so no additional embedding generation is needed.
    func activateExternalPerson(id: UUID, gallery: [[Float]], boundingBox: CGRect, worldPosition: simd_float2?) {
        stopTracking()

        activePersonID = id
        targetGallery = gallery
        lastGallerySampleDate = Date()

        if !targetGallery.isEmpty {
            log("🧬 External target gallery set (\(targetGallery.count) samples)")
        }

        let observation = VNDetectedObjectObservation(boundingBox: boundingBox)
        let request = VNTrackObjectRequest(detectedObjectObservation: observation)
        request.trackingLevel = .accurate
        trackRequest = request
        sequenceHandler = VNSequenceRequestHandler()

        trackedBoundingBox = boundingBox
        trackedWorldPosition = worldPosition
        lostFrameCount = 0
        state = .tracking
        onStateChanged?(.tracking)

        if let wp = worldPosition { onPositionUpdated?(wp) }

        let person = DetectedPerson(
            id: id,
            boundingBox: boundingBox,
            gallery: gallery,
            worldPosition: worldPosition,
            lastSeen: Date()
        )
        onPersonActivated?(person)
    }

    /// Stop all tracking and scanning.
    func stopTracking() {
        state = .idle
        trackedWorldPosition = nil
        trackedBoundingBox = nil
        targetGallery = []
        trackRequest = nil
        lostFrameCount = 0
        scanFrameCount = 0
        activePersonID = nil
        detectedPeople = []
        onStateChanged?(.idle)
    }

    // MARK: - Scanning Pipeline

    private func updateScanning(frame: ARFrame) {
        scanFrameCount += 1
        guard scanFrameCount % scanDetectionInterval == 0 else { return }

        let pixelBuffer = frame.capturedImage
        let handler = VNImageRequestHandler(cvPixelBuffer: pixelBuffer, orientation: .right, options: [:])
        let detectRequest = VNDetectHumanRectanglesRequest()

        do {
            try handler.perform([detectRequest])
        } catch {
            log("⚠️ Scan detection failed: \(error)")
            return
        }

        reconcileDetectedPeople(persons: detectRequest.results ?? [], pixelBuffer: pixelBuffer, frame: frame)

        let now = Date()
        detectedPeople.removeAll { now.timeIntervalSince($0.lastSeen) > personPruneTimeout }
        onPeopleUpdated?(detectedPeople)

        if !detectedPeople.isEmpty && scanFrameCount % (scanDetectionInterval * 4) == 0 {
            log("👥 \(detectedPeople.count) people visible")
        }
    }

    /// Match new detections to existing tracked people using IoU (primary) then
    /// gallery-based ReID with a ratio test (fallback for people who moved).
    private func reconcileDetectedPeople(persons: [VNHumanObservation], pixelBuffer: CVPixelBuffer, frame: ARFrame) {
        var matchedExistingIDs = Set<UUID>()
        var matchedDetectionIndices = Set<Int>()

        // Phase 1: greedy IoU matching (best overlap first)
        struct IouCandidate {
            let detectionIndex: Int
            let existingID: UUID
            let score: CGFloat
        }
        var iouCandidates: [IouCandidate] = []
        for (i, person) in persons.enumerated() {
            for existing in detectedPeople {
                let iou = computeIoU(person.boundingBox, existing.boundingBox)
                if iou > iouMatchThreshold {
                    iouCandidates.append(IouCandidate(detectionIndex: i, existingID: existing.id, score: iou))
                }
            }
        }
        iouCandidates.sort { $0.score > $1.score }

        var updatedPeople: [DetectedPerson] = []

        for c in iouCandidates {
            guard !matchedExistingIDs.contains(c.existingID),
                  !matchedDetectionIndices.contains(c.detectionIndex) else { continue }
            matchedExistingIDs.insert(c.existingID)
            matchedDetectionIndices.insert(c.detectionIndex)

            let bbox = persons[c.detectionIndex].boundingBox
            var existing = detectedPeople.first(where: { $0.id == c.existingID })!
            existing.boundingBox = bbox
            existing.lastSeen = Date()
            if let worldPos = projectToWorld(boundingBox: bbox, frame: frame) {
                existing.worldPosition = worldPos
            }
            // Add a diverse gallery sample so the identity stays fresh across poses
            if let emb = generateEmbedding(pixelBuffer: pixelBuffer, boundingBox: bbox) {
                existing.addToGallery(emb)
            }
            updatedPeople.append(existing)
        }

        // Phase 2: gallery ReID + ratio test for unmatched detections
        for (i, person) in persons.enumerated() where !matchedDetectionIndices.contains(i) {
            let bbox = person.boundingBox
            guard let embedding = generateEmbedding(pixelBuffer: pixelBuffer, boundingBox: bbox) else {
                // Can't embed — treat as new
                updatedPeople.append(newPerson(bbox: bbox, embedding: nil, frame: frame))
                continue
            }

            // Score every unmatched existing person against this embedding
            let candidates = detectedPeople
                .filter { !matchedExistingIDs.contains($0.id) }
                .map { ($0.id, $0.bestSimilarity(to: embedding)) }
                .sorted { $0.1 > $1.1 }

            let bestSim = candidates.first?.1 ?? 0
            let secondSim = candidates.dropFirst().first?.1 ?? 0

            if bestSim > reidMatchThreshold && bestSim - secondSim >= reidMargin,
               let matchID = candidates.first?.0 {
                matchedExistingIDs.insert(matchID)
                matchedDetectionIndices.insert(i)

                var existing = detectedPeople.first(where: { $0.id == matchID })!
                existing.boundingBox = bbox
                existing.lastSeen = Date()
                existing.addToGallery(embedding)
                if let worldPos = projectToWorld(boundingBox: bbox, frame: frame) {
                    existing.worldPosition = worldPos
                }
                updatedPeople.append(existing)
                log("🧬 Re-identified \(matchID.uuidString.prefix(8)) (sim: \(String(format: "%.2f", bestSim)))")
            } else {
                updatedPeople.append(newPerson(bbox: bbox, embedding: embedding, frame: frame))
            }
        }

        // Carry forward people not matched to any detection; pruned later by timeout
        for existing in detectedPeople where !matchedExistingIDs.contains(existing.id) {
            updatedPeople.append(existing)
        }

        detectedPeople = updatedPeople
    }

    private func newPerson(bbox: CGRect, embedding: [Float]?, frame: ARFrame) -> DetectedPerson {
        let id = UUID()
        var gallery: [[Float]] = []
        if let emb = embedding { gallery = [emb] }
        let person = DetectedPerson(
            id: id, boundingBox: bbox, gallery: gallery,
            worldPosition: projectToWorld(boundingBox: bbox, frame: frame),
            lastSeen: Date()
        )
        log("👤 New person detected (ID: \(id.uuidString.prefix(8)))")
        return person
    }

    // MARK: - Activation

    private func activatePersonForTracking(person: DetectedPerson, pixelBuffer: CVPixelBuffer, frame: ARFrame) {
        let bbox = person.boundingBox

        activePersonID = person.id
        targetGallery = person.gallery
        if targetGallery.isEmpty, let emb = generateEmbedding(pixelBuffer: pixelBuffer, boundingBox: bbox) {
            targetGallery = [emb]
        }
        lastGallerySampleDate = Date()
        log("🧬 Target gallery: \(targetGallery.count) samples")

        let observation = VNDetectedObjectObservation(boundingBox: bbox)
        let request = VNTrackObjectRequest(detectedObjectObservation: observation)
        request.trackingLevel = .accurate
        trackRequest = request
        sequenceHandler = VNSequenceRequestHandler()

        trackedBoundingBox = bbox
        lostFrameCount = 0
        state = .tracking
        onStateChanged?(.tracking)

        if let worldPos = projectToWorld(boundingBox: bbox, frame: frame) {
            trackedWorldPosition = worldPos
            onPositionUpdated?(worldPos)
        }
        onPersonActivated?(person)
    }

    // MARK: - Active Person Tracking

    private func updateActiveTracking(frame: ARFrame) {
        let pixelBuffer = frame.capturedImage

        if let request = trackRequest {
            do {
                try sequenceHandler.perform([request], on: pixelBuffer, orientation: .right)

                if let result = request.results?.first as? VNDetectedObjectObservation,
                   result.confidence > 0.3 {
                    let bbox = result.boundingBox
                    trackedBoundingBox = bbox

                    if let worldPos = projectToWorld(boundingBox: bbox, frame: frame) {
                        trackedWorldPosition = worldPos
                        onPositionUpdated?(worldPos)
                    }

                    if state != .tracking {
                        state = .tracking
                        onStateChanged?(.tracking)
                        log("👤 Re-acquired person via tracker")
                    }
                    lostFrameCount = 0

                    let newObservation = VNDetectedObjectObservation(boundingBox: bbox)
                    let newRequest = VNTrackObjectRequest(detectedObjectObservation: newObservation)
                    newRequest.trackingLevel = .accurate
                    trackRequest = newRequest

                    // Periodically add a diverse sample to the target gallery so
                    // re-acquisition works even after significant appearance change.
                    if Date().timeIntervalSince(lastGallerySampleDate) >= embeddingRefreshInterval,
                       let newEmb = generateEmbedding(pixelBuffer: pixelBuffer, boundingBox: bbox) {
                        let maxSim = targetGallery.map { Self.cosineSimilarity($0, newEmb) }.max() ?? 0
                        if maxSim < 0.92 {
                            targetGallery.append(newEmb)
                            if targetGallery.count > Self.targetGalleryMaxSize { targetGallery.removeFirst() }
                        }
                        lastGallerySampleDate = Date()
                    }

                    return
                }
            } catch {
                // Tracker failed — fall through to re-acquisition
            }
        }

        lostFrameCount += 1

        if lostFrameCount >= lostFrameThreshold && state != .lost {
            state = .lost
            onStateChanged?(.lost)
            log("❌ Person lost — could not re-acquire")
            return
        }

        if state != .reacquiring {
            state = .reacquiring
            onStateChanged?(.reacquiring)
            log("🔍 Tracker lost — attempting re-acquisition...")
        }

        guard lostFrameCount % 3 == 0 else { return }
        reacquire(frame: frame)
    }

    // MARK: - Re-acquisition

    private func reacquire(frame: ARFrame) {
        guard !targetGallery.isEmpty else { return }

        let pixelBuffer = frame.capturedImage
        let handler = VNImageRequestHandler(cvPixelBuffer: pixelBuffer, orientation: .right, options: [:])
        let detectRequest = VNDetectHumanRectanglesRequest()

        do { try handler.perform([detectRequest]) } catch { return }
        guard let results = detectRequest.results, !results.isEmpty else { return }

        // Score all candidates; apply ratio test before accepting
        let scored: [(bbox: CGRect, sim: Float)] = results.compactMap { obs in
            guard let emb = generateEmbedding(pixelBuffer: pixelBuffer, boundingBox: obs.boundingBox) else { return nil }
            let sim = targetGallery.map { Self.cosineSimilarity($0, emb) }.max() ?? 0
            return (obs.boundingBox, sim)
        }.sorted { $0.sim > $1.sim }

        let bestSim = scored.first?.sim ?? 0
        let secondSim = scored.dropFirst().first?.sim ?? 0

        guard bestSim > reidMatchThreshold && bestSim - secondSim >= reidMargin,
              let match = scored.first else { return }

        log("🧬 Re-acquired (sim: \(String(format: "%.3f", match.sim)))")

        let observation = VNDetectedObjectObservation(boundingBox: match.bbox)
        let request = VNTrackObjectRequest(detectedObjectObservation: observation)
        request.trackingLevel = .accurate
        trackRequest = request
        sequenceHandler = VNSequenceRequestHandler()

        trackedBoundingBox = match.bbox
        lostFrameCount = 0
        state = .tracking
        onStateChanged?(.tracking)

        if let worldPos = projectToWorld(boundingBox: match.bbox, frame: frame) {
            trackedWorldPosition = worldPos
            onPositionUpdated?(worldPos)
        }
    }

    // MARK: - Embedding Generation

    /// Generate a 512-dim OSNet embedding from a person crop. Returns a L2-normalized
    /// vector so cosine similarity reduces to a dot product.
    func generateEmbedding(pixelBuffer: CVPixelBuffer, boundingBox: CGRect) -> [Float]? {
        guard let model = reidModel else { return nil }

        let request = VNCoreMLRequest(model: model)
        request.imageCropAndScaleOption = .scaleFill
        request.regionOfInterest = boundingBox

        let handler = VNImageRequestHandler(cvPixelBuffer: pixelBuffer, orientation: .right, options: [:])
        do { try handler.perform([request]) } catch { return nil }

        guard let result = request.results?.first as? VNCoreMLFeatureValueObservation,
              let multiArray = result.featureValue.multiArrayValue else { return nil }

        let count = multiArray.count
        var embedding = [Float](repeating: 0, count: count)
        for i in 0..<count { embedding[i] = multiArray[i].floatValue }

        // L2-normalize so cosine similarity == dot product
        var norm: Float = 0
        for v in embedding { norm += v * v }
        norm = norm.squareRoot()
        guard norm > 1e-6 else { return nil }
        for i in 0..<count { embedding[i] /= norm }

        return embedding
    }

    // MARK: - Similarity

    /// Cosine similarity between two L2-normalized vectors (dot product).
    static func cosineSimilarity(_ a: [Float], _ b: [Float]) -> Float {
        guard a.count == b.count, !a.isEmpty else { return 0 }
        var dot: Float = 0
        for i in 0..<a.count { dot += a[i] * b[i] }
        return dot
    }

    /// Instance-method alias so call sites that captured `self` don't need updating.
    func cosineSimilarity(_ a: [Float], _ b: [Float]) -> Float {
        Self.cosineSimilarity(a, b)
    }

    // MARK: - Geometry Helpers

    private func computeIoU(_ a: CGRect, _ b: CGRect) -> CGFloat {
        let intersection = a.intersection(b)
        guard !intersection.isNull else { return 0 }
        let intersectionArea = intersection.width * intersection.height
        let unionArea = a.area + b.area - intersectionArea
        guard unionArea > 0 else { return 0 }
        return intersectionArea / unionArea
    }

    // MARK: - Depth Projection

    /// Project the center of a bounding box to 3D world coordinates using
    /// the LiDAR depth map.
    /// Returns world position as (ARKit X, ARKit Z) — matching the occupancy grid convention.
    func projectToWorld(boundingBox: CGRect, frame: ARFrame) -> simd_float2? {
        guard let sceneDepth = frame.sceneDepth ?? frame.smoothedSceneDepth else { return nil }

        let depthMap = sceneDepth.depthMap
        let depthWidth = CVPixelBufferGetWidth(depthMap)
        let depthHeight = CVPixelBufferGetHeight(depthMap)

        CVPixelBufferLockBaseAddress(depthMap, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(depthMap, .readOnly) }

        guard let baseAddress = CVPixelBufferGetBaseAddress(depthMap) else { return nil }
        let floatBuffer = baseAddress.assumingMemoryBound(to: Float32.self)
        let floatsPerRow = CVPixelBufferGetBytesPerRow(depthMap) / MemoryLayout<Float32>.size

        // Vision bbox origin is bottom-left; .right orientation maps Vision X→depth Y, Vision Y→depth X
        let depthPixelX = Int(boundingBox.midY * CGFloat(depthWidth))
        let depthPixelY = Int((1.0 - boundingBox.midX) * CGFloat(depthHeight))

        let px = max(0, min(depthWidth - 1, depthPixelX))
        let py = max(0, min(depthHeight - 1, depthPixelY))

        var depthSamples: [Float] = []
        let r = 2
        for dy in -r...r {
            for dx in -r...r {
                let sx = max(0, min(depthWidth - 1, px + dx))
                let sy = max(0, min(depthHeight - 1, py + dy))
                let d = floatBuffer[sy * floatsPerRow + sx]
                if d > 0.01 && d < 10.0 { depthSamples.append(d) }
            }
        }
        guard !depthSamples.isEmpty else { return nil }
        depthSamples.sort()
        let depth = depthSamples[depthSamples.count / 2]

        let intrinsics = frame.camera.intrinsics
        let imageRes = frame.camera.imageResolution
        let scaleX = Float(depthWidth) / Float(imageRes.width)
        let scaleY = Float(depthHeight) / Float(imageRes.height)
        let fx = intrinsics[0][0] * scaleX
        let fy = intrinsics[1][1] * scaleY
        let cx = intrinsics[2][0] * scaleX
        let cy = intrinsics[2][1] * scaleY

        let camX = (Float(px) - cx) / fx * depth
        let camY = (Float(py) - cy) / fy * depth
        let camPoint = simd_float4(camX, -camY, -depth, 1.0)
        let worldPoint = frame.camera.transform * camPoint

        return simd_float2(worldPoint.x, worldPoint.z)
    }

    // MARK: - Logging

    private func log(_ message: String) {
        print("[PersonTracker] \(message)")
        onLog?(message)
    }
}

// MARK: - CGRect Helpers

private extension CGRect {
    var area: CGFloat { width * height }
}
