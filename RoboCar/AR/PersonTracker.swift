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
/// Flow:
///   1. `startScanning()` — enters scanning mode. Every few frames, detects all
///      people and assigns stable IDs / ReID embeddings.
///   2. `activateExternalPerson(...)` or `activatePerson(id:frame:)` selects the
///      person to follow, captures their ReID embedding, and starts a VN tracker.
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
        var embedding: [Float]?          // OSNet 512-dim ReID embedding
        var worldPosition: simd_float2?  // (ARKit X, ARKit Z)
        var lastSeen: Date
    }
    
    enum TrackingState: String {
        case idle
        case scanning       // Detecting all people, waiting for activation gesture
        case tracking       // VN tracker is following the activated person
        case reacquiring    // Tracker lost — running detection + ReID
        case lost           // Could not re-acquire after lostFrameThreshold
    }
    
    // MARK: - Configuration
    
    /// Minimum cosine similarity to accept a ReID match
    var reidMatchThreshold: Float = 0.55
    
    /// How many consecutive frames of lost tracking before announcing loss
    var lostFrameThreshold: Int = 20  // ~1s at 20fps
    
    /// Standoff distance — how far from the person the car should stop (meters)
    var standoffDistance: Float = 0.8
    
    /// How often to refresh the stored embedding from the tracked person (seconds)
    var embeddingRefreshInterval: TimeInterval = 2.0
    
    /// How many frames between full detection scans during scanning mode
    var scanDetectionInterval: Int = 5
    
    /// IoU threshold for matching detections to existing tracked people
    var iouMatchThreshold: CGFloat = 0.25
    
    /// Maximum time (seconds) before a person not re-detected is pruned
    var personPruneTimeout: TimeInterval = 3.0
    
    // MARK: - State
    
    private(set) var state: TrackingState = .idle
    
    /// All currently detected/tracked people (updated during scanning)
    private(set) var detectedPeople: [DetectedPerson] = []
    
    /// The ID of the person being followed (activated by tap or by name)
    private(set) var activePersonID: UUID? = nil
    
    /// World position of the tracked person (ARKit X = x, ARKit Z = y), updated each frame
    private(set) var trackedWorldPosition: simd_float2? = nil
    
    /// Bounding box of the tracked person in normalized image coordinates (Vision convention)
    private(set) var trackedBoundingBox: CGRect? = nil
    
    /// The target embedding generated when tracking started
    private var targetEmbedding: [Float]? = nil
    
    /// VN tracker request for frame-to-frame tracking
    private var trackRequest: VNTrackObjectRequest?
    
    /// Sequence request handler (reused across frames for tracker continuity)
    private var sequenceHandler = VNSequenceRequestHandler()
    
    /// The CoreML model for generating ReID embeddings
    private var reidModel: VNCoreMLModel?
    
    /// Frame counter for lost tracking
    private var lostFrameCount: Int = 0
    
    /// Frame counter for scanning detection cadence
    private var scanFrameCount: Int = 0
    
    /// Last time the stored embedding was refreshed
    private var lastEmbeddingRefresh: Date = .distantPast
    
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
            // Try compiled model in bundle
            if let compiledURL = Bundle.main.url(forResource: "OSNetReID", withExtension: "mlmodelc") {
                do {
                    let mlModel = try MLModel(contentsOf: compiledURL)
                    reidModel = try VNCoreMLModel(for: mlModel)
                    log("✅ OSNet ReID model loaded")
                } catch {
                    log("⚠️ Failed to load OSNet model: \(error)")
                }
            } else {
                log("⚠️ OSNetReID model not found in bundle")
            }
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
        
        // Pick the largest bounding box (closest person)
        let sorted = results.sorted { $0.boundingBox.area > $1.boundingBox.area }
        let bestMatch = sorted[0]
        let bbox = bestMatch.boundingBox
        
        log("👤 Detected \(results.count) people — tracking largest (area: \(String(format: "%.3f", bbox.area)))")
        
        let person = DetectedPerson(
            id: UUID(),
            boundingBox: bbox,
            embedding: generateEmbedding(pixelBuffer: pixelBuffer, boundingBox: bbox),
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
        let pixelBuffer = frame.capturedImage
        activatePersonForTracking(person: person, pixelBuffer: pixelBuffer, frame: frame)
    }
    
    /// Activate tracking for a person identified externally (e.g. by the always-on scanner).
    /// Provides a pre-computed embedding and bounding box so no ARFrame is needed for
    /// embedding generation — only for depth projection.
    func activateExternalPerson(id: UUID, embedding: [Float], boundingBox: CGRect, worldPosition: simd_float2?) {
        stopTracking()
        
        activePersonID = id
        targetEmbedding = embedding
        lastEmbeddingRefresh = Date()
        log("🧬 External target embedding set (512-dim)")
        
        // Initialize VN object tracker on this bounding box
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
        
        if let wp = worldPosition {
            onPositionUpdated?(wp)
        }
        
        let person = DetectedPerson(
            id: id,
            boundingBox: boundingBox,
            embedding: embedding,
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
        targetEmbedding = nil
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
        
        let personResults = detectRequest.results ?? []
        
        // Update the tracked people list (match detections to existing IDs)
        reconcileDetectedPeople(persons: personResults, pixelBuffer: pixelBuffer, frame: frame)
        
        // Prune people not seen recently
        let now = Date()
        detectedPeople.removeAll { now.timeIntervalSince($0.lastSeen) > personPruneTimeout }
        
        // Notify listeners
        onPeopleUpdated?(detectedPeople)
        
        if !detectedPeople.isEmpty && scanFrameCount % (scanDetectionInterval * 4) == 0 {
            log("👥 \(detectedPeople.count) people visible")
        }
    }
    
    /// Match new person detections to existing tracked people using IoU and ReID.
    private func reconcileDetectedPeople(persons: [VNHumanObservation], pixelBuffer: CVPixelBuffer, frame: ARFrame) {
        var matchedExistingIDs = Set<UUID>()
        var matchedDetectionIndices = Set<Int>()
        
        // Build IoU match pairs
        struct MatchCandidate {
            let detectionIndex: Int
            let existingID: UUID
            let score: CGFloat
        }
        var candidates: [MatchCandidate] = []
        
        for (i, person) in persons.enumerated() {
            for existing in detectedPeople {
                let iou = computeIoU(person.boundingBox, existing.boundingBox)
                if iou > iouMatchThreshold {
                    candidates.append(MatchCandidate(detectionIndex: i, existingID: existing.id, score: iou))
                }
            }
        }
        
        // Greedy matching: best IoU first
        candidates.sort { $0.score > $1.score }
        
        var updatedPeople: [DetectedPerson] = []
        
        for candidate in candidates {
            guard !matchedExistingIDs.contains(candidate.existingID),
                  !matchedDetectionIndices.contains(candidate.detectionIndex) else { continue }
            
            matchedExistingIDs.insert(candidate.existingID)
            matchedDetectionIndices.insert(candidate.detectionIndex)
            
            let bbox = persons[candidate.detectionIndex].boundingBox
            var existing = detectedPeople.first(where: { $0.id == candidate.existingID })!
            existing.boundingBox = bbox
            existing.lastSeen = Date()
            if let worldPos = projectToWorld(boundingBox: bbox, frame: frame) {
                existing.worldPosition = worldPos
            }
            // Refresh embedding if we don't have one yet
            if existing.embedding == nil {
                existing.embedding = generateEmbedding(pixelBuffer: pixelBuffer, boundingBox: bbox)
            }
            updatedPeople.append(existing)
        }
        
        // For unmatched detections, try ReID against unmatched existing people
        for (i, person) in persons.enumerated() where !matchedDetectionIndices.contains(i) {
            let bbox = person.boundingBox
            let embedding = generateEmbedding(pixelBuffer: pixelBuffer, boundingBox: bbox)
            
            var reidMatch: (id: UUID, similarity: Float)?
            if let embed = embedding {
                for existing in detectedPeople where !matchedExistingIDs.contains(existing.id) {
                    if let existEmbed = existing.embedding {
                        let sim = cosineSimilarity(embed, existEmbed)
                        if sim > reidMatchThreshold {
                            if reidMatch == nil || sim > reidMatch!.similarity {
                                reidMatch = (existing.id, sim)
                            }
                        }
                    }
                }
            }
            
            if let match = reidMatch, !matchedExistingIDs.contains(match.id) {
                // Re-identified existing person at new position
                matchedExistingIDs.insert(match.id)
                matchedDetectionIndices.insert(i)
                
                var existing = detectedPeople.first(where: { $0.id == match.id })!
                existing.boundingBox = bbox
                existing.lastSeen = Date()
                existing.embedding = embedding ?? existing.embedding
                if let worldPos = projectToWorld(boundingBox: bbox, frame: frame) {
                    existing.worldPosition = worldPos
                }
                updatedPeople.append(existing)
                log("🧬 Re-identified person \(match.id.uuidString.prefix(8)) (sim: \(String(format: "%.2f", match.similarity)))")
            } else {
                // Brand new person
                let id = UUID()
                let worldPos = projectToWorld(boundingBox: bbox, frame: frame)
                let newPerson = DetectedPerson(
                    id: id,
                    boundingBox: bbox,
                    embedding: embedding,
                    worldPosition: worldPos,
                    lastSeen: Date()
                )
                updatedPeople.append(newPerson)
                log("👤 New person detected (ID: \(id.uuidString.prefix(8)))")
            }
        }
        
        // Carry forward existing people not matched to any detection (may still be around)
        for existing in detectedPeople where !matchedExistingIDs.contains(existing.id) {
            updatedPeople.append(existing)  // will be pruned later if lastSeen is old
        }
        
        detectedPeople = updatedPeople
    }
    
    // MARK: - Activation
    
    /// Transition from scanning to tracking a specific person.
    private func activatePersonForTracking(person: DetectedPerson, pixelBuffer: CVPixelBuffer, frame: ARFrame) {
        let bbox = person.boundingBox
        
        activePersonID = person.id
        targetEmbedding = person.embedding ?? generateEmbedding(pixelBuffer: pixelBuffer, boundingBox: bbox)
        lastEmbeddingRefresh = Date()
        
        if targetEmbedding != nil {
            log("🧬 Target embedding captured (512-dim)")
        }
        
        // Initialize VN object tracker on this bounding box
        let observation = VNDetectedObjectObservation(boundingBox: bbox)
        let request = VNTrackObjectRequest(detectedObjectObservation: observation)
        request.trackingLevel = .accurate
        trackRequest = request
        sequenceHandler = VNSequenceRequestHandler()
        
        trackedBoundingBox = bbox
        lostFrameCount = 0
        state = .tracking
        onStateChanged?(.tracking)
        
        // Project initial position
        if let worldPos = projectToWorld(boundingBox: bbox, frame: frame) {
            trackedWorldPosition = worldPos
            onPositionUpdated?(worldPos)
        }
        
        onPersonActivated?(person)
    }
    
    // MARK: - Active Person Tracking
    
    private func updateActiveTracking(frame: ARFrame) {
        let pixelBuffer = frame.capturedImage
        
        // Try VN tracker first
        if let request = trackRequest {
            do {
                try sequenceHandler.perform([request], on: pixelBuffer, orientation: .right)
                
                if let result = request.results?.first as? VNDetectedObjectObservation,
                   result.confidence > 0.3 {
                    // Tracker succeeded
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
                    
                    // Update the tracker's observation for next frame
                    let newObservation = VNDetectedObjectObservation(boundingBox: bbox)
                    let newRequest = VNTrackObjectRequest(detectedObjectObservation: newObservation)
                    newRequest.trackingLevel = .accurate
                    trackRequest = newRequest
                    
                    // Periodically refresh embedding from tracked person
                    if Date().timeIntervalSince(lastEmbeddingRefresh) >= embeddingRefreshInterval {
                        if let newEmbed = generateEmbedding(pixelBuffer: pixelBuffer, boundingBox: bbox) {
                            // Blend with existing embedding (exponential moving average)
                            if let existing = targetEmbedding {
                                let alpha: Float = 0.3
                                targetEmbedding = zip(existing, newEmbed).map { (1 - alpha) * $0 + alpha * $1 }
                                // Re-normalize
                                let norm = sqrtf(targetEmbedding!.reduce(0) { $0 + $1 * $1 })
                                if norm > 0 {
                                    targetEmbedding = targetEmbedding!.map { $0 / norm }
                                }
                            } else {
                                targetEmbedding = newEmbed
                            }
                            lastEmbeddingRefresh = Date()
                        }
                    }
                    
                    return
                }
            } catch {
                // Tracker failed — fall through to re-acquisition
            }
        }
        
        // Tracker lost — attempt re-acquisition via detection + ReID
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
        
        // Run person detection + ReID matching every 3 lost frames to save compute
        guard lostFrameCount % 3 == 0 else { return }
        
        reacquire(frame: frame)
    }
    
    // MARK: - Re-acquisition
    
    private func reacquire(frame: ARFrame) {
        guard let targetEmbed = targetEmbedding else { return }
        
        let pixelBuffer = frame.capturedImage
        let handler = VNImageRequestHandler(cvPixelBuffer: pixelBuffer, orientation: .right, options: [:])
        
        let detectRequest = VNDetectHumanRectanglesRequest()
        
        do {
            try handler.perform([detectRequest])
        } catch { return }
        
        guard let results = detectRequest.results, !results.isEmpty else { return }
        
        // Compare each detected person's embedding to the target
        var bestMatch: (bbox: CGRect, similarity: Float)?
        
        for person in results {
            let bbox = person.boundingBox
            guard let embedding = generateEmbedding(pixelBuffer: pixelBuffer, boundingBox: bbox) else { continue }
            
            let similarity = cosineSimilarity(targetEmbed, embedding)
            
            if similarity > reidMatchThreshold {
                if bestMatch == nil || similarity > bestMatch!.similarity {
                    bestMatch = (bbox, similarity)
                }
            }
        }
        
        guard let match = bestMatch else { return }
        
        log("🧬 ReID match: \(String(format: "%.3f", match.similarity)) similarity")
        
        // Re-initialize tracker on matched person
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
    
    /// Generate a 512-dim OSNet embedding from a person crop.
    func generateEmbedding(pixelBuffer: CVPixelBuffer, boundingBox: CGRect) -> [Float]? {
        guard let model = reidModel else { return nil }
        
        let request = VNCoreMLRequest(model: model)
        request.imageCropAndScaleOption = .scaleFill
        request.regionOfInterest = boundingBox
        
        let handler = VNImageRequestHandler(cvPixelBuffer: pixelBuffer, orientation: .right, options: [:])
        
        do {
            try handler.perform([request])
        } catch {
            return nil
        }
        
        guard let result = request.results?.first as? VNCoreMLFeatureValueObservation,
              let multiArray = result.featureValue.multiArrayValue else {
            return nil
        }
        
        // Convert MLMultiArray to [Float]
        let count = multiArray.count
        var embedding = [Float](repeating: 0, count: count)
        for i in 0..<count {
            embedding[i] = multiArray[i].floatValue
        }
        
        // L2-normalize so that `cosineSimilarity` (a plain dot product) yields a
        // true cosine in [-1, 1]. Without this, raw magnitudes vary frame-to-frame
        // and ReID matching becomes unstable (same person gets new IDs).
        var norm: Float = 0
        for value in embedding { norm += value * value }
        norm = norm.squareRoot()
        if norm > 1e-6 {
            for i in 0..<count { embedding[i] /= norm }
        }
        
        return embedding
    }
    
    /// Cosine similarity between two L2-normalized vectors.
    func cosineSimilarity(_ a: [Float], _ b: [Float]) -> Float {
        guard a.count == b.count else { return 0 }
        var dot: Float = 0
        for i in 0..<a.count {
            dot += a[i] * b[i]
        }
        return dot
    }
    
    // MARK: - Geometry Helpers
    
    /// Intersection over Union of two CGRects.
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
        let bytesPerRow = CVPixelBufferGetBytesPerRow(depthMap)
        let floatsPerRow = bytesPerRow / MemoryLayout<Float32>.size
        
        // Bounding box center in normalized image coordinates (Vision convention: origin bottom-left)
        let centerX = boundingBox.midX
        let centerY = boundingBox.midY
        
        // Convert to depth map pixel coordinates
        // Vision bbox is in normalized coordinates with origin at bottom-left.
        // The depth map orientation matches the camera's native orientation.
        // Since we use .right orientation for Vision, we need to map accordingly.
        // For a landscape-right camera with .right orientation:
        //   Vision X → depth Y (inverted), Vision Y → depth X
        let depthPixelX = Int(centerY * CGFloat(depthWidth))
        let depthPixelY = Int((1.0 - centerX) * CGFloat(depthHeight))
        
        // Clamp to valid range
        let px = max(0, min(depthWidth - 1, depthPixelX))
        let py = max(0, min(depthHeight - 1, depthPixelY))
        
        // Sample depth — take the median of a 5x5 patch for robustness
        var depthSamples: [Float] = []
        let patchRadius = 2
        for dy in -patchRadius...patchRadius {
            for dx in -patchRadius...patchRadius {
                let sx = max(0, min(depthWidth - 1, px + dx))
                let sy = max(0, min(depthHeight - 1, py + dy))
                let d = floatBuffer[sy * floatsPerRow + sx]
                if d > 0.01 && d < 10.0 {
                    depthSamples.append(d)
                }
            }
        }
        
        guard !depthSamples.isEmpty else { return nil }
        depthSamples.sort()
        let depth = depthSamples[depthSamples.count / 2]  // median
        
        // Project to 3D using camera intrinsics
        let intrinsics = frame.camera.intrinsics
        let imageRes = frame.camera.imageResolution
        let scaleX = Float(depthWidth) / Float(imageRes.width)
        let scaleY = Float(depthHeight) / Float(imageRes.height)
        let fx = intrinsics[0][0] * scaleX
        let fy = intrinsics[1][1] * scaleY
        let cx = intrinsics[2][0] * scaleX
        let cy = intrinsics[2][1] * scaleY
        
        // Camera-local 3D point (intrinsics convention: x-right, y-down, z-forward)
        let camX = (Float(px) - cx) / fx * depth
        let camY = (Float(py) - cy) / fy * depth
        let camPoint = simd_float4(camX, -camY, -depth, 1.0)
        
        // Transform to world space
        let worldPoint = frame.camera.transform * camPoint
        
        // Return in occupancy grid convention: (ARKit X, ARKit Z)
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
