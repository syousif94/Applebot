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
/// and OSNet ReID embeddings. Waits for an activation gesture (raised open palm)
/// to select which person to follow, then uses VNTrackObjectRequest for efficient
/// frame-to-frame tracking of that person.
///
/// Flow:
///   1. `startScanning()` — enters scanning mode. Every few frames, detects all
///      people and runs hand pose detection looking for an open-palm gesture.
///   2. When a person holds an open palm for several consecutive detection cycles,
///      they become the *active* person, their ReID embedding is captured, and
///      a VN tracker begins.
///   3. `update(frame:)` — in scanning mode, runs multi-person detection + gesture
///      check. In tracking mode, runs the VN tracker on the active person, with
///      ReID fallback if the tracker loses them.
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
        /// Whether this person is currently showing the activation gesture
        var isGesturing: Bool = false
        /// How many consecutive detection cycles the gesture has been seen
        var gestureStreakCount: Int = 0
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
    
    /// Minimum confidence for hand joint detection
    var handConfidenceThreshold: Float = 0.3
    
    /// Timeout (seconds) for completing a multi-step gesture before it resets
    var gestureTimeout: TimeInterval = 3.0
    
    /// IoU threshold for matching detections to existing tracked people
    var iouMatchThreshold: CGFloat = 0.25
    
    /// Maximum time (seconds) before a person not re-detected is pruned
    var personPruneTimeout: TimeInterval = 3.0
    
    // MARK: - State
    
    private(set) var state: TrackingState = .idle
    
    /// All currently detected/tracked people (updated during scanning)
    private(set) var detectedPeople: [DetectedPerson] = []
    
    /// The ID of the person activated via gesture (the one being followed)
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
        log("👀 Scanning for people — raise an open hand to activate following")
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
        
        // Run person detection and hand pose detection in a single perform call
        let detectRequest = VNDetectHumanRectanglesRequest()
        let handPoseRequest = VNDetectHumanHandPoseRequest()
        handPoseRequest.maximumHandCount = 6
        
        do {
            try handler.perform([detectRequest, handPoseRequest])
        } catch {
            log("⚠️ Scan detection failed: \(error)")
            return
        }
        
        let personResults = detectRequest.results ?? []
        let handResults = handPoseRequest.results ?? []
        
        // Update the tracked people list (match detections to existing IDs)
        reconcileDetectedPeople(persons: personResults, pixelBuffer: pixelBuffer, frame: frame)
        
        // Check for activation gesture — open palm matched to a person
        checkForActivationGesture(hands: handResults, frame: frame)
        
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
    
    // MARK: - Gesture Detection
    
    /// State machine for the activation gesture (peace-sign double-tap).
    enum ActivateGesturePhase {
        case idle
        case peaceOpen1         // First peace sign detected
        case peaceClosed1       // Fingers closed after first peace
        case peaceOpen2         // Second peace sign detected → activate!
    }
    
    /// State machine for the cancel gesture (index finger wag).
    enum CancelGesturePhase: Int {
        case idle = 0
        case movedRight1 = 1   // First rightward movement
        case movedLeft1 = 2    // First leftward movement
        case movedRight2 = 3   // Second rightward movement
        case movedLeft2 = 4    // Complete — cancel!
    }
    
    /// Per-person activation gesture state
    private var activatePhase: [UUID: ActivateGesturePhase] = [:]
    private var activatePhaseTimestamp: [UUID: Date] = [:]
    
    /// Cancel gesture state (not per-person, any hand can cancel)
    private var cancelPhase: CancelGesturePhase = .idle
    private var cancelLastWristX: CGFloat?
    private var cancelPhaseTimestamp: Date = .distantPast
    /// Minimum fingertip X movement (normalised coords) to count as a direction change
    private let wagMinDelta: CGFloat = 0.04
    
    /// Callback when cancel gesture is detected
    var onCancelGesture: (() -> Void)?
    
    /// Check all detected hands for activation and cancel gestures, matched to people.
    private func checkForActivationGesture(hands: [VNHumanHandPoseObservation], frame: ARFrame) {
        let now = Date()
        
        // Classify each hand
        var peaceHands: [(wrist: CGPoint, hand: VNHumanHandPoseObservation)] = []
        var fistHands: [CGPoint] = []   // wrist locations of closed hands
        var pointerHands: [(wrist: CGPoint, wristX: CGFloat)] = []
        
        for hand in hands {
            if let wrist = detectPeaceSign(hand: hand), detectedPeople.contains(where: { isRaisedHandPoint(wrist, inside: $0.boundingBox) }) {
                peaceHands.append((wrist, hand))
            } else if let wrist = detectClosedHand(hand: hand), detectedPeople.contains(where: { isRaisedHandPoint(wrist, inside: $0.boundingBox) }) {
                fistHands.append(wrist)
            }
            if let info = detectIndexPointer(hand: hand), detectedPeople.contains(where: { isRaisedHandPoint(info.wrist, inside: $0.boundingBox) }) {
                pointerHands.append(info)
            }
        }
        
        // --- Activation gesture: peace-sign double-tap ---
        var gesturedPersonIDs = Set<UUID>()
        
        for (wrist, _) in peaceHands {
            for i in detectedPeople.indices {
                guard isRaisedHandPoint(wrist, inside: detectedPeople[i].boundingBox) else { continue }
                let pid = detectedPeople[i].id
                gesturedPersonIDs.insert(pid)
                
                let phase = activatePhase[pid] ?? .idle
                let phaseTime = activatePhaseTimestamp[pid] ?? .distantPast
                let elapsed = now.timeIntervalSince(phaseTime)
                
                switch phase {
                case .idle:
                    activatePhase[pid] = .peaceOpen1
                    activatePhaseTimestamp[pid] = now
                    detectedPeople[i].isGesturing = true
                    detectedPeople[i].gestureStreakCount = 1
                    log("✌️ Person \(pid.uuidString.prefix(8)) — peace sign detected (1/2)")
                case .peaceOpen1:
                    // Still holding peace — keep waiting for close
                    detectedPeople[i].isGesturing = true
                case .peaceClosed1:
                    if elapsed < gestureTimeout {
                        activatePhase[pid] = .peaceOpen2
                        activatePhaseTimestamp[pid] = now
                        detectedPeople[i].isGesturing = true
                        detectedPeople[i].gestureStreakCount = 2
                        log("✌️ Person \(pid.uuidString.prefix(8)) — second peace sign! Activating…")
                        let pixelBuffer = frame.capturedImage
                        activatePersonForTracking(person: detectedPeople[i], pixelBuffer: pixelBuffer, frame: frame)
                        activatePhase.removeAll()
                        activatePhaseTimestamp.removeAll()
                        return
                    } else {
                        // Timeout — restart
                        activatePhase[pid] = .peaceOpen1
                        activatePhaseTimestamp[pid] = now
                        detectedPeople[i].gestureStreakCount = 1
                    }
                case .peaceOpen2:
                    break // shouldn't reach here
                }
                break  // one hand matches at most one person
            }
        }
        
        // Check for fist (closed hand) to transition peaceOpen1 → peaceClosed1
        for wrist in fistHands {
            for i in detectedPeople.indices {
                guard isRaisedHandPoint(wrist, inside: detectedPeople[i].boundingBox) else { continue }
                let pid = detectedPeople[i].id
                let phase = activatePhase[pid] ?? .idle
                let phaseTime = activatePhaseTimestamp[pid] ?? .distantPast
                let elapsed = now.timeIntervalSince(phaseTime)
                
                if phase == .peaceOpen1 && elapsed < gestureTimeout {
                    activatePhase[pid] = .peaceClosed1
                    activatePhaseTimestamp[pid] = now
                    log("✊ Person \(pid.uuidString.prefix(8)) — fingers closed (waiting for second peace)")
                }
                break
            }
        }
        
        // Reset gesture state for people NOT involved this cycle
        for i in detectedPeople.indices {
            let pid = detectedPeople[i].id
            if !gesturedPersonIDs.contains(pid) {
                detectedPeople[i].isGesturing = false
                detectedPeople[i].gestureStreakCount = 0
            }
            // Timeout stale phases
            if let phaseTime = activatePhaseTimestamp[pid],
               now.timeIntervalSince(phaseTime) > gestureTimeout {
                activatePhase.removeValue(forKey: pid)
                activatePhaseTimestamp.removeValue(forKey: pid)
            }
        }
        
        // --- Cancel gesture: index finger wag left-right (one full sweep) ---
        if let (wrist, tipX) = pointerHands.first {
            let _ = wrist  // wrist position available if needed
            let elapsed = now.timeIntervalSince(cancelPhaseTimestamp)
            
            if elapsed > gestureTimeout {
                // Timeout — restart
                cancelPhase = .idle
                cancelLastWristX = nil
            }
            
            if let lastX = cancelLastWristX {
                let delta = tipX - lastX
                
                switch cancelPhase {
                case .idle:
                    if abs(delta) > wagMinDelta {
                        cancelPhase = delta > 0 ? .movedRight1 : .movedLeft1
                        cancelPhaseTimestamp = now
                        cancelLastWristX = tipX
                    }
                case .movedRight1:
                    if delta < -wagMinDelta {
                        // Moved right then left — cancel!
                        log("👆↔️ Cancel gesture detected (finger wag R→L)!")
                        cancelPhase = .idle
                        cancelLastWristX = nil
                        onCancelGesture?()
                        return
                    } else if delta > wagMinDelta {
                        cancelLastWristX = tipX  // accumulating rightward
                    }
                case .movedLeft1:
                    if delta > wagMinDelta {
                        // Moved left then right — cancel!
                        log("👆↔️ Cancel gesture detected (finger wag L→R)!")
                        cancelPhase = .idle
                        cancelLastWristX = nil
                        onCancelGesture?()
                        return
                    } else if delta < -wagMinDelta {
                        cancelLastWristX = tipX  // accumulating leftward
                    }
                case .movedRight2, .movedLeft2:
                    break // unused now
                }
            } else {
                cancelLastWristX = tipX
                cancelPhaseTimestamp = now
            }
        }
    }
    
    /// Detect a peace/victory sign: index + middle extended, ring + little curled.
    /// Returns the wrist location if detected, nil otherwise.
    func detectPeaceSign(hand: VNHumanHandPoseObservation) -> CGPoint? {
        guard let wrist = try? hand.recognizedPoint(.wrist),
              let indexTip = try? hand.recognizedPoint(.indexTip),
              let middleTip = try? hand.recognizedPoint(.middleTip),
              let ringTip = try? hand.recognizedPoint(.ringTip),
              let littleTip = try? hand.recognizedPoint(.littleTip),
              let indexPIP = try? hand.recognizedPoint(.indexPIP),
              let middlePIP = try? hand.recognizedPoint(.middlePIP),
              let ringPIP = try? hand.recognizedPoint(.ringPIP),
              let littlePIP = try? hand.recognizedPoint(.littlePIP)
        else { return nil }
        
        let minConf = handConfidenceThreshold
        guard wrist.confidence > minConf,
              indexTip.confidence > minConf,
              middleTip.confidence > minConf
        else { return nil }
        
        func dist(_ a: CGPoint, _ b: CGPoint) -> CGFloat {
            hypot(a.x - b.x, a.y - b.y)
        }
        
        let w = wrist.location
        
        // Index + middle EXTENDED (tip farther from wrist than PIP)
        let indexExtended = dist(indexTip.location, w) > dist(indexPIP.location, w)
        let middleExtended = dist(middleTip.location, w) > dist(middlePIP.location, w)
        
        // Ring + little CURLED (tip closer to wrist than PIP, or at least not extended)
        let ringCurled = dist(ringTip.location, w) < dist(ringPIP.location, w) * 1.15
        let littleCurled = dist(littleTip.location, w) < dist(littlePIP.location, w) * 1.15
        
        guard indexExtended && middleExtended && ringCurled && littleCurled else { return nil }
        
        return wrist.location
    }
    
    /// Detect a closed fist / curled fingers. Returns wrist location if detected.
    func detectClosedHand(hand: VNHumanHandPoseObservation) -> CGPoint? {
        guard let wrist = try? hand.recognizedPoint(.wrist),
              let indexTip = try? hand.recognizedPoint(.indexTip),
              let middleTip = try? hand.recognizedPoint(.middleTip),
              let indexPIP = try? hand.recognizedPoint(.indexPIP),
              let middlePIP = try? hand.recognizedPoint(.middlePIP)
        else { return nil }
        
        let minConf = handConfidenceThreshold
        guard wrist.confidence > minConf else { return nil }
        
        func dist(_ a: CGPoint, _ b: CGPoint) -> CGFloat {
            hypot(a.x - b.x, a.y - b.y)
        }
        
        let w = wrist.location
        
        // Both index and middle should be curled
        let indexCurled = dist(indexTip.location, w) < dist(indexPIP.location, w) * 1.15
        let middleCurled = dist(middleTip.location, w) < dist(middlePIP.location, w) * 1.15
        
        guard indexCurled && middleCurled else { return nil }
        
        return wrist.location
    }
    
    /// Detect a pointing hand (index finger extended).
    /// Returns (wrist location, index fingertip X in normalised coords) or nil.
    /// Requires index extended above the palm and middle finger NOT extended.
    func detectIndexPointer(hand: VNHumanHandPoseObservation) -> (wrist: CGPoint, wristX: CGFloat)? {
        guard let wrist = try? hand.recognizedPoint(.wrist),
              let indexTip = try? hand.recognizedPoint(.indexTip),
              let indexPIP = try? hand.recognizedPoint(.indexPIP),
              let middleTip = try? hand.recognizedPoint(.middleTip),
              let middlePIP = try? hand.recognizedPoint(.middlePIP),
              let middleMCP = try? hand.recognizedPoint(.middleMCP)
        else { return nil }
        
        let minConf = handConfidenceThreshold
        guard wrist.confidence > minConf,
              indexTip.confidence > minConf
        else { return nil }
        
        func dist(_ a: CGPoint, _ b: CGPoint) -> CGFloat {
            hypot(a.x - b.x, a.y - b.y)
        }
        
        let w = wrist.location
        let indexExtended = dist(indexTip.location, w) > dist(indexPIP.location, w)
        
        // Index fingertip must be above the palm (higher Y in Vision coords = higher in image)
        let palmCenterY = (wrist.location.y + middleMCP.location.y) / 2
        let indexAbovePalm = indexTip.location.y > palmCenterY
        
        // Middle finger must NOT be extended
        let middleExtended = dist(middleTip.location, w) > dist(middlePIP.location, w)
        
        guard indexExtended && indexAbovePalm && !middleExtended else { return nil }
        
        // Return fingertip X (not wrist X) — the tip sweeps much more during a wag
        return (wrist.location, indexTip.location.x)
    }
    
    /// Legacy open-palm detection kept for reference but no longer used for activation.
    func detectOpenPalm(hand: VNHumanHandPoseObservation) -> CGPoint? {
        guard let wrist = try? hand.recognizedPoint(.wrist),
              let thumbTip = try? hand.recognizedPoint(.thumbTip),
              let indexTip = try? hand.recognizedPoint(.indexTip),
              let middleTip = try? hand.recognizedPoint(.middleTip),
              let ringTip = try? hand.recognizedPoint(.ringTip),
              let littleTip = try? hand.recognizedPoint(.littleTip),
              let thumbIP = try? hand.recognizedPoint(.thumbIP),
              let indexPIP = try? hand.recognizedPoint(.indexPIP),
              let middlePIP = try? hand.recognizedPoint(.middlePIP),
              let ringPIP = try? hand.recognizedPoint(.ringPIP),
              let littlePIP = try? hand.recognizedPoint(.littlePIP)
        else { return nil }
        
        let minConf = handConfidenceThreshold
        guard wrist.confidence > minConf,
              indexTip.confidence > minConf,
              middleTip.confidence > minConf,
              ringTip.confidence > minConf,
              littleTip.confidence > minConf
        else { return nil }
        
        func dist(_ a: CGPoint, _ b: CGPoint) -> CGFloat {
            hypot(a.x - b.x, a.y - b.y)
        }
        
        let w = wrist.location
        let fingersExtended =
            dist(indexTip.location, w)  > dist(indexPIP.location, w)  &&
            dist(middleTip.location, w) > dist(middlePIP.location, w) &&
            dist(ringTip.location, w)   > dist(ringPIP.location, w)   &&
            dist(littleTip.location, w) > dist(littlePIP.location, w) &&
            dist(thumbTip.location, w)  > dist(thumbIP.location, w)
        
        guard fingersExtended else { return nil }
        
        let spread = min(
            dist(indexTip.location, middleTip.location),
            dist(middleTip.location, ringTip.location),
            dist(ringTip.location, littleTip.location)
        )
        let palmSize = dist(wrist.location, middleTip.location)
        guard palmSize > 0, spread / palmSize > 0.05 else { return nil }
        
        return wrist.location
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

    private func isRaisedHandPoint(_ point: CGPoint, inside personBox: CGRect) -> Bool {
        let expandedBox = personBox.insetBy(dx: -0.08, dy: -0.04)
        guard expandedBox.contains(point) else { return false }
        let upperBodyThreshold = personBox.origin.y + personBox.height * 0.45
        return point.y >= upperBodyThreshold
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
