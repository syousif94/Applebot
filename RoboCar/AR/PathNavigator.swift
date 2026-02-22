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
    var lookaheadDistance: Float = 0.30
    
    /// Distance to consider "arrived" at the final target (meters)
    var arrivalThreshold: Float = 0.12
    
    /// Maximum time to wait for an obstacle to clear before re-planning (seconds)
    let obstaclePauseTimeout: TimeInterval = 5.0
    
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
    private var currentWaypointIndex: Int = 0
    
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
    private let reverseDuration: TimeInterval = 0.6
    
    /// Duration of the turn-away phase after reversing (seconds)
    private let reverseTurnDuration: TimeInterval = 0.5
    
    /// Phase of reverse maneuver: 0 = backing up, 1 = turning away
    private var reversePhase: Int = 0
    
    /// Stuck detection: last recorded position
    private var lastStuckCheckPos: (x: Float, y: Float)?
    
    /// Stuck detection: timestamp of last position sample
    private var lastStuckCheckTime: Date?
    
    /// How long to wait before checking if stuck (seconds)
    private let stuckCheckInterval: TimeInterval = 1.0
    
    /// Minimum distance to have moved in stuckCheckInterval to not be stuck (meters)
    private let stuckMinDistance: Float = 0.03  // 3cm
    
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
        
        if detector.obstacleDetected {
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
                // Obstacle is in the forward travel cone — stop and reverse
                if state != .paused {
                    ESP32BLEManager.shared.stopAll()
                    state = .paused
                    pauseStartTime = Date()
                    log("⏸️ Obstacle ahead — reversing")
                    startReverse(detector: detector)
                } else if let start = pauseStartTime,
                          Date().timeIntervalSince(start) > obstaclePauseTimeout {
                    log("⏱️ Still blocked ahead — reversing again")
                    startReverse(detector: detector)
                    pauseStartTime = Date()
                }
                return
            } else {
                // Obstacle is off to the side — don't reverse, but force a
                // re-plan so A* routes around it. Keep navigating.
                if state == .paused {
                    state = .navigating
                    pauseStartTime = nil
                    log("▶️ Obstacle no longer ahead — resuming")
                }
                // Trigger an early re-plan so the path steers clear
                if Date().timeIntervalSince(lastReplanTime) > 0.5 {
                    lastReplanTime = .distantPast
                }
            }
        } else {
            // No obstacle at all — resume if we were paused
            if state == .paused {
                state = .navigating
                pauseStartTime = nil
                log("▶️ Obstacle cleared — resuming")
            }
        }
        
        guard state == .navigating else { return }
        
        // Stuck detection: if power is being applied but the car isn't moving, reverse
        if let lastPos = lastStuckCheckPos, let lastTime = lastStuckCheckTime {
            if Date().timeIntervalSince(lastTime) >= stuckCheckInterval {
                let movedDx = pos.x - lastPos.x
                let movedDy = pos.y - lastPos.y
                let movedDist = sqrtf(movedDx * movedDx + movedDy * movedDy)
                
                if movedDist < stuckMinDistance {
                    log("🚧 Stuck — power applied but not moving, reversing")
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
        
        // Heading error (how much we need to turn)
        var headingError = targetHeading - pos.heading
        // Normalize to [-π, π]
        while headingError > .pi  { headingError -= 2 * .pi }
        while headingError < -.pi { headingError += 2 * .pi }
        
        // Convert to drive commands using differential steering directly.
        // Proportional steering: turn harder when heading error is large.
        // At full turn, one side goes forward and the other reverses (spin in place).
        let turnGain: Float = 1.5  // Proportional gain for steering
        let turn = max(-1.0, min(1.0, headingError * turnGain))
        
        // Both sides get equal base power, steering is differential on top
        let basePower: Float = cruiseSpeed  // 55% nominal
        
        // Differential: positive turn = turn right (more left, less right)
        // At full turn (1.0), left = base + base = 2*base, right = base - base = 0
        // We scale turn component by basePower so a full turn gives opposite sides
        var leftPower  = basePower + turn * basePower
        var rightPower = basePower - turn * basePower
        
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
    
    // MARK: - Reverse Maneuver
    
    /// Start a reverse-and-turn maneuver to escape an obstacle.
    private func startReverse(detector: ObstacleDetector) {
        isReversing = true
        reversePhase = 0
        reverseStartTime = Date()
        
        // Phase 0: reverse at 60% power
        let reversePower: Int8 = -60
        ESP32BLEManager.shared.setAllMotors(a: reversePower, b: reversePower, c: reversePower, d: reversePower, bypassObstacleFilter: true)
        log("↩️ Reversing...")
    }
    
    /// Called each tick while isReversing is true — manages the reverse phases.
    private func tickReverse() {
        guard let startTime = reverseStartTime else {
            finishReverse()
            return
        }
        
        let elapsed = Date().timeIntervalSince(startTime)
        
        switch reversePhase {
        case 0:
            // Phase 0: reversing
            if elapsed >= reverseDuration {
                // Move to phase 1: turn away from the obstacle
                reversePhase = 1
                reverseStartTime = Date()
                
                // Determine turn direction: turn away from the blocked side
                let detector = ObstacleDetector.shared
                let turnRight: Bool
                if let blocked = detector.blockedLocalDirection {
                    // blocked.x > 0 means obstacle is to the right → turn left
                    turnRight = blocked.x < 0
                } else {
                    turnRight = Bool.random()
                }
                
                let turnPower: Int8 = 60
                if turnRight {
                    ESP32BLEManager.shared.setAllMotors(a: turnPower, b: -turnPower, c: turnPower, d: -turnPower, bypassObstacleFilter: true)
                } else {
                    ESP32BLEManager.shared.setAllMotors(a: -turnPower, b: turnPower, c: -turnPower, d: turnPower, bypassObstacleFilter: true)
                }
                log("↩️ Turning \(turnRight ? "right" : "left") to avoid obstacle")
            }
            
        default:
            // Phase 1: turning away
            if elapsed >= reverseTurnDuration {
                finishReverse()
            }
        }
    }
    
    /// End the reverse maneuver and request a re-plan.
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
    
    /// Advance the waypoint index past any waypoints we've already passed.
    private func advanceWaypoint(pos: DevicePosition) {
        while currentWaypointIndex < currentPath.count - 1 {
            let wp = currentPath[currentWaypointIndex]
            let dx = wp.x - pos.x
            let dy = wp.y - pos.y
            let dist = sqrtf(dx * dx + dy * dy)
            
            // If we're close enough to the current waypoint, advance
            if dist < lookaheadDistance * 0.6 {
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
