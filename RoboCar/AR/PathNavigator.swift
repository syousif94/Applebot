//
//  PathNavigator.swift
//  RoboCar
//
//  Created by Sammy Yousif on 2/20/26.
//

import Foundation
import simd

/// Navigation state machine
enum NavigationState: Equatable {
    case idle
    case navigating
    case paused          // obstacle blocking — waiting to clear
    case arrived
}

/// Drives the ESP32 along a planned path using pure-pursuit steering.
///
/// Key design points:
///   - Reads `OccupancyGrid.devicePosition` each tick (~10 Hz)
///   - Computes a lookahead waypoint and steering via pure pursuit
///   - **Inverts commands** because the phone is mounted facing backward on the car
///   - Integrates with `ObstacleDetector` — pauses when an obstacle is in the way
///   - Supports live path re-planning as the map updates
class PathNavigator {
    
    // MARK: - Singleton
    
    static let shared = PathNavigator()
    
    // MARK: - Configuration
    
    /// Driving speed (0–1 scale) fed into `drive(x:y:)`
    var cruiseSpeed: Float = 0.55
    
    /// Lookahead distance for pure pursuit (meters)
    var lookaheadDistance: Float = 0.45
    
    /// Distance to consider "arrived" at the final target (meters)
    var arrivalThreshold: Float = 0.12
    
    /// Maximum time to wait for an obstacle to clear before reversing (seconds)
    let obstaclePauseTimeout: TimeInterval = 3.0
    
    /// Grace period after resuming from a pause — ignore obstacles briefly so
    /// the new path has time to steer away (seconds).
    private let resumeGracePeriod: TimeInterval = 0.3
    
    /// Timestamp when we last resumed from paused → navigating
    private var lastResumeTime: Date = .distantPast
    
    /// Half-angle of the forward cone used to decide if an obstacle is "blocking"
    /// the direction of travel (radians). Obstacles outside this cone trigger a
    /// re-plan but not a full reverse.  ~60° half-angle → 120° forward cone.
    var forwardConeHalfAngle: Float = .pi / 3.0  // 60°
    
    /// How often the navigation timer fires (seconds)
    private let tickInterval: TimeInterval = 0.1  // 10 Hz
    
    /// How often to re-plan the path during navigation (seconds)
    var replanInterval: TimeInterval = 2.0
    
    // MARK: - State
    
    private(set) var state: NavigationState = .idle {
        didSet {
            if state != oldValue {
                onStateChanged?(state)
            }
        }
    }
    
    /// The original target world position
    private(set) var targetPoint: (x: Float, y: Float)?
    
    /// Current path waypoints in world coordinates
    private(set) var currentPath: [(x: Float, y: Float)] = []
    
    /// Index of the next waypoint we're pursuing
    private(set) var currentWaypointIndex: Int = 0
    
    /// Timer driving the control loop
    private var navTimer: Timer?
    
    /// Timestamp of last re-plan
    private var lastReplanTime: Date = .distantPast
    
    /// Timestamp when we entered paused state
    private var pauseStartTime: Date?
    
    /// Whether we're currently executing a reverse maneuver
    private var isReversing: Bool = false
    
    /// Timestamp when reverse maneuver started
    private var reverseStartTime: Date?
    
    /// Duration of the reverse phase (seconds)
    private let reverseDuration: TimeInterval = 0.35
    
    /// Duration of the pause after reversing to let sensors update (seconds)
    private let reversePauseDuration: TimeInterval = 0.5
    
    /// Phase of reverse maneuver: 0 = backing up, 1 = pause
    private var reversePhase: Int = 0
    
    /// Stuck detection: last recorded position
    private var lastStuckCheckPos: (x: Float, y: Float)?
    
    /// Stuck detection: timestamp of last position sample
    private var lastStuckCheckTime: Date?
    
    /// How long to wait before checking if stuck (seconds)
    private let stuckCheckInterval: TimeInterval = 1.0
    
    /// Minimum distance to have moved in stuckCheckInterval to not be stuck (meters)
    private let stuckMinDistance: Float = 0.03  // 3cm
    
    /// Spinning stuck detection: timestamp when continuous spinning started
    private var spinStartTime: Date?
    
    /// Maximum time to spin in place before triggering an escape (seconds)
    private let maxSpinDuration: TimeInterval = 2.0
    
    /// Reference to the occupancy grid
    weak var occupancyGrid: OccupancyGrid?
    
    /// Callbacks
    var onStateChanged: ((NavigationState) -> Void)?
    var onPathUpdated: ((_ path: [(x: Float, y: Float)]) -> Void)?
    var onLog: ((String) -> Void)?
    
    private init() {}
    
    // MARK: - Public API
    
    /// Start navigating to the given world-coordinate target along the provided path.
    func startNavigation(to target: (x: Float, y: Float), path: [(x: Float, y: Float)]) {
        guard !path.isEmpty else {
            log("❌ Cannot navigate — no path")
            return
        }
        
        stopNavigation()
        
        targetPoint = target
        currentPath = path
        currentWaypointIndex = 0
        lastReplanTime = Date()
        pauseStartTime = nil
        lastStuckCheckPos = nil
        lastStuckCheckTime = nil
        spinStartTime = nil
        state = .navigating
        
        log("🧭 Navigation started — \(path.count) waypoints to (\(String(format: "%.2f", target.x)), \(String(format: "%.2f", target.y)))")
        
        // Start the control loop on the main thread
        navTimer = Timer.scheduledTimer(withTimeInterval: tickInterval, repeats: true) { [weak self] _ in
            self?.tick()
        }
    }
    
    /// Stop navigation and halt motors.
    func stopNavigation() {
        navTimer?.invalidate()
        navTimer = nil
        currentPath = []
        currentWaypointIndex = 0
        targetPoint = nil
        pauseStartTime = nil
        isReversing = false
        reversePhase = 0
        reverseStartTime = nil
        lastStuckCheckPos = nil
        lastStuckCheckTime = nil
        spinStartTime = nil
        
        if state != .idle {
            ESP32BLEManager.shared.stopAll()
            state = .idle
            log("🛑 Navigation stopped")
        }
    }
    
    /// Update the path (called when the map changes and a re-plan produces a new route).
    func updatePath(_ newPath: [(x: Float, y: Float)]) {
        guard state == .navigating || state == .paused else { return }
        guard !newPath.isEmpty else { return }
        
        currentPath = newPath
        currentWaypointIndex = 0
        lastReplanTime = Date()
        
        // If we were paused and a new clear path exists, resume
        if state == .paused {
            state = .navigating
            pauseStartTime = nil
            lastResumeTime = Date()
            log("▶️ Resumed navigation with new path")
        }
        
        onPathUpdated?(newPath)
    }
    
    /// Whether a re-plan is due (called by the view controller to decide when to re-plan).
    var needsReplan: Bool {
        guard state == .navigating || state == .paused else { return false }
        return Date().timeIntervalSince(lastReplanTime) >= replanInterval
    }
    
    // MARK: - Control Loop
    
    private func tick() {
        guard let grid = occupancyGrid else { return }
        let pos = grid.devicePosition
        
        guard let target = targetPoint else {
            stopNavigation()
            return
        }
        
        // Check arrival at final target
        let dxTarget = target.x - pos.x
        let dyTarget = target.y - pos.y
        let distToTarget = sqrtf(dxTarget * dxTarget + dyTarget * dyTarget)
        
        if distToTarget < arrivalThreshold {
            ESP32BLEManager.shared.stopAll()
            state = .arrived
            log("✅ Arrived at target!")
            navTimer?.invalidate()
            navTimer = nil
            return
        }
        
        // Check obstacle detector
        let detector = ObstacleDetector.shared
        
        // If currently reversing, run the reverse maneuver
        if isReversing {
            tickReverse()
            return
        }
        
        // Grace period: right after resuming from a pause, skip obstacle checks
        // so the car has a chance to follow the newly replanned path.
        let inGracePeriod = Date().timeIntervalSince(lastResumeTime) < resumeGracePeriod
        
        if detector.obstacleDetected && !inGracePeriod {
            // Determine the intended travel direction so we can check whether
            // the obstacle is actually blocking our path, not just off to the side.
            let travelHeading: Float
            if currentWaypointIndex < currentPath.count {
                let wp = currentPath[min(currentWaypointIndex, currentPath.count - 1)]
                travelHeading = atan2f(wp.x - pos.x, wp.y - pos.y)
            } else {
                travelHeading = atan2f(dxTarget, dyTarget)
            }
            
            let blocking = isObstacleBlockingTravel(detector: detector, travelHeading: travelHeading)
            
            if blocking {
                if state != .paused {
                    // First response: just stop and request a re-plan around the obstacle.
                    // Do NOT reverse — let A* find a new route.
                    ESP32BLEManager.shared.stopAll()
                    state = .paused
                    pauseStartTime = Date()
                    lastReplanTime = .distantPast  // force immediate replan
                    log("⏸️ Obstacle blocking — stopping for replan")
                } else if let start = pauseStartTime,
                          Date().timeIntervalSince(start) > obstaclePauseTimeout {
                    // We've waited long enough for a clear path — reverse to escape
                    log("⏱️ Still blocked after \(String(format: "%.1f", obstaclePauseTimeout))s — reversing")
                    startReverse(detector: detector)
                    pauseStartTime = Date()
                }
                return
            } else {
                // Obstacle is off to the side — don't stop, but force a
                // re-plan so A* routes around it. Keep navigating.
                if state == .paused {
                    state = .navigating
                    pauseStartTime = nil
                    lastResumeTime = Date()
                    log("▶️ Obstacle no longer ahead — resuming")
                }
                // Trigger an early re-plan so the path steers clear
                if Date().timeIntervalSince(lastReplanTime) > 0.5 {
                    lastReplanTime = .distantPast
                }
            }
        } else {
            // No obstacle (or in grace period) — resume if we were paused
            if state == .paused {
                state = .navigating
                pauseStartTime = nil
                lastResumeTime = Date()
                log("▶️ Obstacle cleared — resuming")
            }
        }
        
        guard state == .navigating else { return }
        
        // Advance waypoint index past waypoints we've already passed
        advanceWaypoint(pos: pos)
        
        guard currentWaypointIndex < currentPath.count else {
            // Ran out of waypoints but haven't reached target — request re-plan
            ESP32BLEManager.shared.stopAll()
            lastReplanTime = .distantPast
            log("🔄 Path exhausted — requesting re-plan")
            return
        }
        
        // Pure pursuit: find lookahead point on the path
        let lookaheadPt = findLookaheadPoint(pos: pos)
        
        // Compute steering toward the lookahead point
        let dx = lookaheadPt.x - pos.x
        let dy = lookaheadPt.y - pos.y
        
        // Target heading in world space (same convention as OccupancyGrid)
        let targetHeading = atan2f(dx, dy)
        
        // ------------------------------------------------------------------
        // Forward corridor obstacle check (grid + raw LiDAR depth)
        // Scans a car-width corridor along the travel heading up to the slow
        // zone radius. If an obstacle is within stopRadius → full stop.
        // If between stopRadius and slowZoneRadius → reduce speed.
        // Skip during grace period so the car can follow a newly replanned path.
        // ------------------------------------------------------------------
        
        // Also check the direct heading to the next waypoint, not just the
        // lookahead point — catches obstacles between us and the immediate
        // next waypoint that the lookahead might skip over.
        let waypointHeading: Float
        if currentWaypointIndex < currentPath.count {
            let wp = currentPath[currentWaypointIndex]
            waypointHeading = atan2f(wp.x - pos.x, wp.y - pos.y)
        } else {
            waypointHeading = targetHeading
        }
        
        // Always check the corridor for obstacles in the travel direction,
        // even during the grace period. The grace period only suppresses the
        // radial bubble check above — the directional corridor scan must
        // remain active so the car never drives blind into a front obstacle.
        let corridorDist: Float?
        do {
            // Check both the lookahead heading and the direct waypoint heading;
            // take the nearer obstacle of the two.
            let d1 = detector.corridorObstacleDistance(heading: targetHeading)
            let d2 = (targetHeading == waypointHeading) ? nil
                : detector.corridorObstacleDistance(heading: waypointHeading)
            switch (d1, d2) {
            case let (a?, b?): corridorDist = min(a, b)
            case let (a?, nil): corridorDist = a
            case let (nil, b?): corridorDist = b
            case (nil, nil):    corridorDist = nil
            }
        }
        
        if let dist = corridorDist, dist <= detector.stopRadius {
            // Near the final destination? Declare arrived early rather than
            // fighting through the last few centimeters into an obstacle.
            if distToTarget < arrivalThreshold * 3 {
                ESP32BLEManager.shared.stopAll()
                state = .arrived
                log("✅ Arrived near target (obstacle in final approach)")
                navTimer?.invalidate()
                navTimer = nil
                return
            }
            
            // Obstacle within hard-stop distance along travel direction —
            // always stop, then replan.
            ESP32BLEManager.shared.stopAll()
            if state != .paused {
                state = .paused
                pauseStartTime = Date()
                log("🛑 Corridor obstacle at \(String(format: "%.0f", dist * 1000))mm — stopping")
            }
            lastReplanTime = .distantPast  // force immediate replan
            return
        }
        
        // Slow-zone factor: 1.0 = full speed, 0.5 = half speed near obstacles
        let corridorSlowFactor: Float
        if let dist = corridorDist, dist <= detector.slowZoneRadius {
            // Linear ramp from 0.5 (at stopRadius) to 1.0 (at slowZoneRadius)
            let range = detector.slowZoneRadius - detector.stopRadius
            let progress = (dist - detector.stopRadius) / max(range, 0.01)
            corridorSlowFactor = 0.5 + 0.5 * progress
        } else {
            corridorSlowFactor = 1.0
        }
        
        // Heading error (how much we need to turn)
        var headingError = targetHeading - pos.heading
        // Normalize to [-π, π]
        while headingError > .pi  { headingError -= 2 * .pi }
        while headingError < -.pi { headingError += 2 * .pi }
        
        let absError = fabsf(headingError)
        let turnSign: Float = headingError >= 0 ? 1.0 : -1.0
        
        // Threshold for "large turn" — above this we spin in place instead of arcing
        let sharpTurnThreshold: Float = .pi / 4.0  // 45°
        
        let isSpinning = absError > sharpTurnThreshold
        
        // Stuck detection: if power is being applied but the car isn't moving, reverse.
        // Skip this check when we're intentionally spinning in place (no translation expected).
        // BUT track how long we've been spinning — if we spin too long, we're stuck.
        if !isSpinning {
            spinStartTime = nil  // not spinning, reset spin timer
            if let lastPos = lastStuckCheckPos, let lastTime = lastStuckCheckTime {
                if Date().timeIntervalSince(lastTime) >= stuckCheckInterval {
                    let movedDx = pos.x - lastPos.x
                    let movedDy = pos.y - lastPos.y
                    let movedDist = sqrtf(movedDx * movedDx + movedDy * movedDy)
                    
                    if movedDist < stuckMinDistance {
                        log("🚧 Stuck — power applied but not moving, escaping")
                        startReverse(detector: ObstacleDetector.shared)
                        state = .paused
                        pauseStartTime = Date()
                        lastStuckCheckPos = (pos.x, pos.y)
                        lastStuckCheckTime = Date()
                        return
                    }
                    
                    lastStuckCheckPos = (pos.x, pos.y)
                    lastStuckCheckTime = Date()
                }
            } else {
                lastStuckCheckPos = (pos.x, pos.y)
                lastStuckCheckTime = Date()
            }
        } else {
            // Spinning in place — track how long and trigger escape if too long
            if spinStartTime == nil {
                spinStartTime = Date()
            } else if let spinStart = spinStartTime,
                      Date().timeIntervalSince(spinStart) >= maxSpinDuration {
                log("🚧 Stuck spinning for \(String(format: "%.1f", maxSpinDuration))s — escaping")
                startReverse(detector: ObstacleDetector.shared)
                state = .paused
                pauseStartTime = Date()
                spinStartTime = nil
                return
            }
            // Reset translation stuck timer while spinning
            lastStuckCheckPos = (pos.x, pos.y)
            lastStuckCheckTime = Date()
        }
        
        var leftPower: Float
        var rightPower: Float
        
        if isSpinning {
            // Sharp turn: spin in place with opposite wheel directions.
            let spinPower: Float = 0.40
            leftPower  =  turnSign * spinPower
            rightPower = -turnSign * spinPower
        } else {
            // Small correction: proportional differential steering while driving forward.
            // Reduce cruise speed proportionally when heading error is large to
            // avoid carrying too much speed into a turn.
            // Apply corridor slow factor when approaching obstacles ahead.
            let turnGain: Float = 1.2
            let turn = max(-1.0, min(1.0, headingError * turnGain))
            let slowdownFactor: Float = 1.0 - 0.5 * (absError / sharpTurnThreshold)
            let basePower: Float = max(0.40, cruiseSpeed * max(0.5, slowdownFactor) * corridorSlowFactor)
            leftPower  = basePower + turn * basePower
            rightPower = basePower - turn * basePower
        }
        
        // Clamp to [-100%, 100%] and enforce minimum |50%| (dead zone avoidance)
        // Values between -50% and 50% don't have enough torque to move.
        // Zero is allowed (motor off).
        func clampPower(_ p: Float) -> Int8 {
            let clamped = max(-1.0, min(1.0, p))
            let scaled = Int(clamped * 100)
            if scaled == 0 { return 0 }
            if scaled > 0 && scaled < 50 { return 50 }
            if scaled < 0 && scaled > -50 { return -50 }
            return Int8(clamping: scaled)
        }
        
        let leftInt  = clampPower(leftPower)
        let rightInt = clampPower(rightPower)
        
        // A & C = left side, B & D = right side
        ESP32BLEManager.shared.setAllMotors(a: leftInt, b: rightInt, c: leftInt, d: rightInt)
    }
    
    // MARK: - Escape Maneuver
    
    /// Start an escape maneuver: back up first to create clearance, then
    /// pause for sensors to update, re-evaluate the clear heading, spin and drive out.
    /// Falls back to blind reverse + turn if no clear heading exists.
    private func startReverse(detector: ObstacleDetector) {
        isReversing = true
        reversePhase = 0  // always start with backup
        reverseStartTime = Date()
        
        // Phase 0: always reverse first to create clearance
        let reversePower: Int8 = -60
        ESP32BLEManager.shared.setAllMotors(a: reversePower, b: reversePower, c: reversePower, d: reversePower)
        log("↩️ Backing up to create clearance...")
    }
    
    /// Called each tick while isReversing is true — manages the escape phases.
    /// Phase 0: back up to create clearance
    /// Phase 1: pause — stop motors, let sensors update
    /// After the pause, finish and force a replan so A* picks a fresh route.
    private func tickReverse() {
        guard let startTime = reverseStartTime else {
            finishReverse()
            return
        }
        
        let elapsed = Date().timeIntervalSince(startTime)
        
        switch reversePhase {
        case 0:
            // Phase 0: backing up
            if elapsed >= reverseDuration {
                // Stop and pause to let sensors refresh
                ESP32BLEManager.shared.stopAll()
                reversePhase = 1
                reverseStartTime = Date()
                log("↩️ Backup done — pausing for sensor update")
            }
            
        default:
            // Phase 1: paused — waiting for LiDAR / obstacle detector to update
            if elapsed >= reversePauseDuration {
                log("↩️ Pause done — requesting replan from new position")
                finishReverse()
            }
        }
    }
    
    /// End the escape maneuver and request a re-plan.
    private func finishReverse() {
        ESP32BLEManager.shared.stopAll()
        isReversing = false
        reversePhase = 0
        reverseStartTime = nil
        
        // Force a re-plan from the new position
        lastReplanTime = .distantPast
        
        // If obstacle is now clear, resume navigating
        if !ObstacleDetector.shared.obstacleDetected {
            state = .navigating
            pauseStartTime = nil
            log("▶️ Reverse complete — obstacle cleared, resuming")
        } else {
            log("⏸️ Reverse complete — still blocked, waiting for re-plan")
        }
    }
    
    // MARK: - Bearing-Aware Obstacle Check
    
    /// Returns `true` when the obstacle centroid is within the forward travel cone,
    /// meaning it actually blocks the intended direction of movement.
    /// Obstacles off to the side (outside the cone) return `false`.
    private func isObstacleBlockingTravel(detector: ObstacleDetector, travelHeading: Float) -> Bool {
        guard let blockedWorld = detector.blockedWorldDirection else {
            // No directional info available — be conservative
            return true
        }
        
        // blockedWorldDirection is a unit vector from the device TOWARD the obstacle.
        // Convert it to a heading angle in the same convention as travelHeading
        // (atan2(x, y), where x = ARKit X, y = ARKit Z).
        let obstacleHeading = atan2f(blockedWorld.x, blockedWorld.y)
        
        // Angular difference between our travel direction and the obstacle
        var angleDiff = obstacleHeading - travelHeading
        while angleDiff >  .pi { angleDiff -= 2 * .pi }
        while angleDiff < -.pi { angleDiff += 2 * .pi }
        
        return fabsf(angleDiff) < forwardConeHalfAngle
    }
    
    // MARK: - Pure Pursuit Helpers
    
    /// Advance the waypoint index: only skip a waypoint if the car has passed
    /// it along the direction toward the next waypoint (i.e. it is behind the
    /// next waypoint from the car's perspective on the path).
    private func advanceWaypoint(pos: DevicePosition) {
        while currentWaypointIndex < currentPath.count - 1 {
            let wp = currentPath[currentWaypointIndex]
            let next = currentPath[currentWaypointIndex + 1]
            
            // Direction along the path segment from current waypoint to next
            let segDx = next.x - wp.x
            let segDy = next.y - wp.y
            let segLenSq = segDx * segDx + segDy * segDy
            
            guard segLenSq > 1e-8 else {
                // Degenerate zero-length segment — skip it
                currentWaypointIndex += 1
                continue
            }
            
            // Project the car position onto the segment direction
            let toPosX = pos.x - wp.x
            let toPosY = pos.y - wp.y
            let dot = toPosX * segDx + toPosY * segDy
            
            // dot > 0 means the car is past the current waypoint in the
            // direction of the next waypoint, so the current wp is "behind"
            // the next wp from our point of view — safe to skip.
            if dot > 0 {
                currentWaypointIndex += 1
            } else {
                break
            }
        }
    }
    
    /// Find a lookahead point on the path ahead of the current position.
    private func findLookaheadPoint(pos: DevicePosition) -> (x: Float, y: Float) {
        // Start from current waypoint index and walk forward to find a point
        // at approximately `lookaheadDistance` from the device
        var bestPt = currentPath[min(currentWaypointIndex, currentPath.count - 1)]
        
        for i in currentWaypointIndex..<currentPath.count {
            let wp = currentPath[i]
            let dx = wp.x - pos.x
            let dy = wp.y - pos.y
            let dist = sqrtf(dx * dx + dy * dy)
            
            bestPt = wp
            
            if dist >= lookaheadDistance {
                break
            }
        }
        
        return bestPt
    }
    
    // MARK: - Logging
    
    private func log(_ message: String) {
        print("[Nav] \(message)")
        onLog?(message)
    }
}
