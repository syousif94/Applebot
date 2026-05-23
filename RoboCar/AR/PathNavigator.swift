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
    case following       // actively following a tracked person
    case followPaused    // following but person is within standoff distance — idling
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
    var arrivalThreshold: Float = 0.20
    
    /// Maximum time to wait for an obstacle to clear before reversing (seconds)
    let obstaclePauseTimeout: TimeInterval = 3.0
    
    /// Grace period after resuming from a pause — ignore obstacles briefly so
    /// the new path has time to steer away (seconds).
    private let resumeGracePeriod: TimeInterval = 1.0
    
    /// Timestamp when we last resumed from paused → navigating
    private var lastResumeTime: Date = .distantPast
    
    /// True while waiting for a replan after obstacle detection.
    /// Suppresses corridor obstacle checks so the robot doesn't re-block
    /// on the stale path before the new (rerouted) path arrives.
    private var awaitingReplan: Bool = false
    
    /// Cooldown after an obstacle-triggered stop — prevents rapid re-triggers
    /// when the corridor check flickers at the boundary (seconds).
    private var lastObstacleStopTime: Date = .distantPast
    private let obstacleStopCooldown: TimeInterval = 2.0
    
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
    
    /// Whether we're spinning to face a clear heading after obstacle detection
    private var isSpinningToTarget: Bool = false
    
    /// Target heading for spin-to-target maneuver
    private var spinToTargetHeading: Float = 0
    
    /// Forced spin direction: +1 = right/CW, -1 = left/CCW
    private var spinToTargetSign: Float = 1.0
    
    /// When the spin-to-target maneuver started
    private var spinToTargetStartTime: Date?
    
    /// Maximum time to spin toward target before giving up and reversing (seconds)
    private let maxSpinToTargetDuration: TimeInterval = 3.0
    
    /// Timestamp when reverse maneuver started
    private var reverseStartTime: Date?
    
    /// Duration of the reverse nudge (seconds) — very short, just enough to unstick
    private let reverseDuration: TimeInterval = 0.25
    
    /// Duration of the pause after reversing to let sensors update (seconds)
    private let reversePauseDuration: TimeInterval = 0.3
    
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
    
    /// Minimum distance of real forward progress to reset the reverse fail counter
    private let resetProgressDistance: Float = 0.10  // 10cm
    
    /// Position when the last reverse/forward-escape cycle started
    private var escapeStartPos: (x: Float, y: Float)?
    
    /// Spinning stuck detection: timestamp when continuous spinning started
    private var spinStartTime: Date?
    
    /// Whether the robot was spinning on the previous tick (for hysteresis)
    private var wasSpinning: Bool = false
    
    /// Maximum time to spin in place before triggering an escape (seconds)
    private let maxSpinDuration: TimeInterval = 5.0
    
    /// Pulse-spin state machine: alternates between a short motor burst and
    /// a coast/stop gap so the heading can settle before re-checking.
    private enum SpinPhase { case burst, coast }
    private var spinPhase: SpinPhase = .burst
    private var spinPhaseStart: Date? = nil
    /// Duration of the motor burst (seconds)
    private let spinBurstDuration: TimeInterval = 0.15
    /// Duration of the coast gap between bursts (seconds)
    private let spinCoastDuration: TimeInterval = 0.25
    
    /// Post-spin settle: after a spin-in-place completes (heading aligned),
    /// the robot pauses briefly and replans so the path reflects the new
    /// orientation and any scans gathered during the spin.
    private var postSpinSettleStart: Date? = nil
    private let postSpinSettleDuration: TimeInterval = 1.0
    
    /// Total escape attempts since last meaningful forward progress
    private var escapeAttemptCount: Int = 0
    
    /// Ordered list of escape strategies to cycle through
    private enum EscapeMove: String {
        case reverseStraight = "reverse"
        case reverseLeft     = "reverse+left"
        case reverseRight    = "reverse+right"
        case forwardLeft     = "forward+left"
        case forwardRight    = "forward+right"
        case spinLeft        = "spin left"
        case spinRight       = "spin right"
    }
    
    private let escapeSequence: [EscapeMove] = [
        .reverseStraight,
        .spinLeft,
        .reverseStraight,
        .spinRight,
        .reverseLeft,
        .forwardLeft,
        .reverseRight,
        .forwardRight,
    ]
    
    /// Whether we're executing a forward escape maneuver (drive forward + turn)
    private var isForwardEscaping: Bool = false
    
    /// Timestamp when forward escape started
    private var forwardEscapeStartTime: Date?
    
    /// Duration of the forward escape drive (seconds)
    private let forwardEscapeDuration: TimeInterval = 0.6
    
    /// Direction to turn during forward escape: +1 right, -1 left
    private var forwardEscapeSign: Float = 1.0
    
    /// Turn direction during reverse nudge: 0 = straight, -1 = left, +1 = right
    private var reverseTurnDirection: Float = 0
    
    /// Whether we're executing a spin escape (pure rotation to break free)
    private var isSpinEscaping: Bool = false
    
    /// Timestamp when spin escape started
    private var spinEscapeStartTime: Date?
    
    /// Spin escape direction: +1 right, -1 left
    private var spinEscapeSign: Float = 1.0
    
    /// Duration of a spin escape maneuver (seconds) — roughly 90°
    private let spinEscapeDuration: TimeInterval = 0.8
    
    // MARK: - Velocity Tracking
    
    /// Previous position for velocity computation
    private var prevTickPos: (x: Float, y: Float)?
    
    /// Previous tick timestamp
    private var prevTickTime: Date?
    
    /// Current estimated ground speed (m/s)
    private(set) var currentSpeed: Float = 0
    
    /// Distance (meters) at which approach deceleration begins
    private let approachDecelerationRadius: Float = 0.60
    
    /// Minimum power during approach (just enough to crawl)
    private let approachMinPower: Float = 0.32
    
    /// Reference to the occupancy grid
    weak var occupancyGrid: OccupancyGrid?
    
    /// Whether we're in follow mode (following a person)
    private(set) var isFollowMode: Bool = false
    
    /// The world position of the followed person, continuously updated
    private(set) var followPersonPosition: (x: Float, y: Float)?
    
    /// Standoff distance — stop this far from the person (meters)
    var followStandoff: Float = 0.8
    
    /// Minimum distance the person must move before re-planning (meters)
    private let followReplanThreshold: Float = 0.3
    
    /// Last position we planned a follow path to
    private var lastFollowPlanPosition: (x: Float, y: Float)?
    
    // MARK: - Bearing Drift Correction
    
    /// Heading from the previous tick, used to detect sudden bearing changes
    private var prevTickHeading: Float?
    
    /// Whether enhanced bearing correction is active (after knockoff detected)
    private var isBearingCorrecting: Bool = false
    
    /// When the bearing correction mode started
    private var bearingCorrectionStart: Date?
    
    /// How long the enhanced correction mode lasts (seconds)
    private let bearingCorrectionDuration: TimeInterval = 2.0
    
    /// A heading change larger than this per tick indicates external force (radians).
    /// At 10 Hz, even aggressive spinning produces ~10°/tick. 15° per tick exceeds
    /// that and almost certainly means the robot was bumped.
    private let bearingKnockoffThreshold: Float = .pi / 12.0  // 15°
    
    /// Last known position of the followed person (persists after track is lost)
    private(set) var lastKnownPersonPosition: (x: Float, y: Float)?
    
    /// Whether we're actively turning toward the person's last known position
    /// after losing them (follow mode only)
    private(set) var isTurningToLastKnown: Bool = false
    
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
        
        // Clear stuck zone memory on every new navigation start
        occupancyGrid?.clearStuckZones()
        
        stopNavigation()
        
        targetPoint = target
        currentPath = path
        currentWaypointIndex = 0
        lastReplanTime = Date()
        pauseStartTime = nil
        lastStuckCheckPos = nil
        lastStuckCheckTime = nil
        spinStartTime = nil
        isSpinningToTarget = false
        spinToTargetStartTime = nil
        escapeAttemptCount = 0
        isForwardEscaping = false
        forwardEscapeStartTime = nil
        isSpinEscaping = false
        spinEscapeStartTime = nil
        wasSpinning = false
        awaitingReplan = false
        lastObstacleStopTime = .distantPast
        postSpinSettleStart = nil
        spinPhase = .burst
        spinPhaseStart = nil
        lastResumeTime = Date()  // grace period for the initial path
        prevTickHeading = nil
        isBearingCorrecting = false
        bearingCorrectionStart = nil
        isTurningToLastKnown = false
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
        isSpinningToTarget = false
        spinToTargetStartTime = nil
        reversePhase = 0
        reverseStartTime = nil
        lastStuckCheckPos = nil
        lastStuckCheckTime = nil
        spinStartTime = nil
        escapeAttemptCount = 0
        isForwardEscaping = false
        forwardEscapeStartTime = nil
        isSpinEscaping = false
        spinEscapeStartTime = nil
        wasSpinning = false
        awaitingReplan = false
        lastObstacleStopTime = .distantPast
        postSpinSettleStart = nil
        spinPhase = .burst
        spinPhaseStart = nil
        escapeStartPos = nil
        prevTickPos = nil
        prevTickTime = nil
        currentSpeed = 0
        isFollowMode = false
        followPersonPosition = nil
        lastFollowPlanPosition = nil
        prevTickHeading = nil
        isBearingCorrecting = false
        bearingCorrectionStart = nil
        lastKnownPersonPosition = nil
        isTurningToLastKnown = false
        
        if state != .idle {
            ESP32BLEManager.shared.stopAll()
            state = .idle
            log("🛑 Navigation stopped")
        }
    }
    
    // MARK: - Follow Mode
    
    /// Start follow mode — the car will continuously navigate toward the person.
    func startFollowing() {
        occupancyGrid?.clearStuckZones()
        stopNavigation()
        isFollowMode = true
        state = .following
        log("👤 Follow mode started")
        
        // Start the control loop
        navTimer = Timer.scheduledTimer(withTimeInterval: tickInterval, repeats: true) { [weak self] _ in
            self?.tick()
        }
    }
    
    /// Update the followed person's world position. Called continuously by LiDARViewController.
    func updateFollowTarget(x: Float, y: Float) {
        followPersonPosition = (x, y)
        lastKnownPersonPosition = (x, y)
        
        // Person is visible again — cancel turn-to-last-known mode
        if isTurningToLastKnown {
            isTurningToLastKnown = false
            log("👤 Person reacquired — resuming normal follow")
        }
        
        // Also update targetPoint so the existing tick() navigation logic works
        // Compute a standoff point: the point `standoff` meters away from the person,
        // toward the car's current position.
        guard let grid = occupancyGrid else { return }
        let pos = grid.devicePosition
        
        let dx = pos.x - x
        let dy = pos.y - y
        let dist = sqrtf(dx * dx + dy * dy)
        
        if dist > 0.01 {
            // Standoff point: along the line from person toward car
            let standoffX = x + (dx / dist) * followStandoff
            let standoffY = y + (dy / dist) * followStandoff
            targetPoint = (standoffX, standoffY)
        } else {
            targetPoint = (x, y)
        }
    }
    
    /// Whether a follow-mode re-plan is needed (person moved significantly).
    var needsFollowReplan: Bool {
        guard isFollowMode, let personPos = followPersonPosition else { return false }
        guard state == .following || state == .followPaused else { return false }
        
        guard let lastPlan = lastFollowPlanPosition else { return true }
        
        let dx = personPos.x - lastPlan.x
        let dy = personPos.y - lastPlan.y
        return sqrtf(dx * dx + dy * dy) >= followReplanThreshold
    }
    
    // MARK: - Lost Person Recovery
    
    /// Called when the followed person is lost (tracker → .lost state).
    /// Instead of stopping, the robot turns toward the person's last known
    /// position to attempt visual reacquisition.
    func handleFollowTargetLost() {
        guard isFollowMode else { return }
        guard let lastPos = lastKnownPersonPosition ?? followPersonPosition else {
            log("⚠️ Person lost — no known position to turn toward")
            return
        }
        
        lastKnownPersonPosition = lastPos
        isTurningToLastKnown = true
        
        // Update the target directly to the last known position (no standoff offset)
        // so the robot faces exactly where the person was last seen.
        targetPoint = lastPos
        
        // If idling near the person (followPaused), switch to following
        // so the tick loop drives heading correction toward the last known position.
        if state == .followPaused {
            state = .following
            onStateChanged?(.following)
        }
        
        // Request a replan toward the last known position
        lastReplanTime = .distantPast
        
        log("👤 Person lost — turning toward last known position (\(String(format: "%.2f", lastPos.x)), \(String(format: "%.2f", lastPos.y)))")
    }
    
    /// Update path for follow mode (called after a re-plan by LiDARViewController).
    func updateFollowPath(_ newPath: [(x: Float, y: Float)]) {
        guard isFollowMode else { return }
        guard !newPath.isEmpty else { return }
        
        currentPath = newPath
        currentWaypointIndex = 0
        lastReplanTime = Date()
        lastResumeTime = Date()
        awaitingReplan = false
        lastFollowPlanPosition = followPersonPosition
        
        if state == .followPaused {
            // Check if person has moved away — resume if beyond standoff
            if let pos = occupancyGrid?.devicePosition, let personPos = followPersonPosition {
                let dx = personPos.x - pos.x
                let dy = personPos.y - pos.y
                let dist = sqrtf(dx * dx + dy * dy)
                if dist > followStandoff * 1.2 {
                    state = .following
                    onStateChanged?(.following)
                    log("▶️ Person moved — resuming follow")
                }
            }
        }
        
        onPathUpdated?(newPath)
    }
    
    /// Update the path (called when the map changes and a re-plan produces a new route).
    func updatePath(_ newPath: [(x: Float, y: Float)]) {
        guard state == .navigating || state == .paused || state == .following || state == .followPaused else { return }
        guard !newPath.isEmpty else { return }
        
        currentPath = newPath
        currentWaypointIndex = 0
        lastReplanTime = Date()
        
        // Always grant a grace period when a new path arrives so the
        // corridor check doesn't immediately re-block the fresh route.
        lastResumeTime = Date()
        awaitingReplan = false
        
        // If we were paused and a new clear path exists, resume
        if state == .paused {
            state = .navigating
            pauseStartTime = nil
            log("▶️ Resumed navigation with new path")
        }
        
        onPathUpdated?(newPath)
    }
    
    /// Called when a replan attempt failed (no path found).
    /// Clears the awaiting-replan flag so obstacle checks resume.
    func clearAwaitingReplan() {
        awaitingReplan = false
    }
    
    /// Whether a re-plan is due (called by the view controller to decide when to re-plan).
    var needsReplan: Bool {
        guard state == .navigating || state == .paused || state == .following else { return false }
        return Date().timeIntervalSince(lastReplanTime) >= replanInterval
    }
    
    // MARK: - Control Loop
    
    private func tick() {
        guard let grid = occupancyGrid else { return }
        let pos = grid.devicePosition
        
        // --- Velocity estimation ---
        let now = Date()
        if let prev = prevTickPos, let prevTime = prevTickTime {
            let dt = Float(now.timeIntervalSince(prevTime))
            if dt > 0.001 {
                let dx = pos.x - prev.x
                let dy = pos.y - prev.y
                let dist = sqrtf(dx * dx + dy * dy)
                // Exponential smoothing to reduce noise
                let raw = dist / dt
                currentSpeed = currentSpeed * 0.6 + raw * 0.4
            }
        }
        prevTickPos = (pos.x, pos.y)
        prevTickTime = now
        
        // --- Bearing drift detection ---
        // Detect sudden heading changes that suggest the robot was bumped
        // by rough pavement or external force. Enable enhanced correction.
        if let prevHeading = prevTickHeading,
           !isReversing, !isSpinEscaping, !isForwardEscaping, !isSpinningToTarget {
            var headingDelta = pos.heading - prevHeading
            while headingDelta >  .pi { headingDelta -= 2 * .pi }
            while headingDelta < -.pi { headingDelta += 2 * .pi }
            
            if fabsf(headingDelta) > bearingKnockoffThreshold {
                if !isBearingCorrecting {
                    isBearingCorrecting = true
                    bearingCorrectionStart = Date()
                    log("⚡ Bearing knocked off by \(String(format: "%.0f°", headingDelta * 180 / .pi)) — correcting")
                }
            }
        }
        prevTickHeading = pos.heading
        
        // Clear bearing correction mode after timeout
        if isBearingCorrecting, let start = bearingCorrectionStart,
           Date().timeIntervalSince(start) > bearingCorrectionDuration {
            isBearingCorrecting = false
            bearingCorrectionStart = nil
            log("✅ Bearing correction complete")
        }
        
        guard let target = targetPoint else {
            stopNavigation()
            return
        }
        
        // Check arrival at final target
        let dxTarget = target.x - pos.x
        let dyTarget = target.y - pos.y
        let distToTarget = sqrtf(dxTarget * dxTarget + dyTarget * dyTarget)
        
        // In follow mode, "arrival" means we're within standoff distance — idle but stay ready
        if isFollowMode {
            if let personPos = followPersonPosition {
                let dxPerson = personPos.x - pos.x
                let dyPerson = personPos.y - pos.y
                let distToPerson = sqrtf(dxPerson * dxPerson + dyPerson * dyPerson)
                
                if distToPerson <= followStandoff {
                    // If turning toward last known position (person lost), don't
                    // idle — keep heading correction active so the robot faces the
                    // last known direction for visual reacquisition.
                    if isTurningToLastKnown {
                        let headingToTarget = atan2f(dxPerson, dyPerson)
                        var headingErr = headingToTarget - pos.heading
                        while headingErr >  .pi { headingErr -= 2 * .pi }
                        while headingErr < -.pi { headingErr += 2 * .pi }
                        if fabsf(headingErr) < .pi / 18.0 {  // ~10° — facing the right way
                            ESP32BLEManager.shared.stopAll()
                            if state != .followPaused {
                                state = .followPaused
                                onStateChanged?(.followPaused)
                                log("⏸️ Facing last known position — waiting for reacquisition")
                            }
                            return
                        }
                        // Fall through to let heading correction handle turning
                    } else {
                        ESP32BLEManager.shared.stopAll()
                        if state != .followPaused {
                            state = .followPaused
                            onStateChanged?(.followPaused)
                            log("⏸️ Within standoff distance — idling")
                        }
                        return
                    }
                } else if state == .followPaused {
                    state = .following
                    onStateChanged?(.following)
                    log("▶️ Person moved beyond standoff — resuming")
                }
            }
        } else if distToTarget < arrivalThreshold {
            ESP32BLEManager.shared.stopAll()
            state = .arrived
            log("✅ Arrived at target! (speed \(String(format: "%.2f", currentSpeed)) m/s)")
            navTimer?.invalidate()
            navTimer = nil
            return
        }
        
        // Approach deceleration: if close to target, compute a speed factor
        // that ramps power down linearly from full at approachDecelerationRadius
        // to approachMinPower at arrivalThreshold.
        let approachFactor: Float
        if !isFollowMode && distToTarget < approachDecelerationRadius {
            let range = approachDecelerationRadius - arrivalThreshold
            let progress = max(0, (distToTarget - arrivalThreshold) / max(range, 0.01))
            // progress: 0 at target → 1 at decel radius edge
            approachFactor = approachMinPower + (1.0 - approachMinPower) * progress
        } else {
            approachFactor = 1.0
        }
        
        // Check obstacle detector
        let detector = ObstacleDetector.shared
        
        // If currently executing a reverse nudge, finish it
        if isReversing {
            tickReverse()
            return
        }
        
        // If currently executing a forward escape, finish it
        if isForwardEscaping {
            tickForwardEscape()
            return
        }
        
        // If currently executing a spin escape, finish it
        if isSpinEscaping {
            tickSpinEscape()
            return
        }
        
        // If spinning toward a clear heading after obstacle detection
        if isSpinningToTarget {
            tickSpinToTarget(pos: pos)
            return
        }
        
        // Grace period: right after resuming from a pause, skip obstacle checks
        // so the car has a chance to follow the newly replanned path.
        let inGracePeriod = Date().timeIntervalSince(lastResumeTime) < resumeGracePeriod
        
        guard state == .navigating || state == .following else { return }
        
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
        // When close to the target, steer directly toward it instead of
        // chasing path waypoints that may loop around obstacles.
        let lookaheadPt: (x: Float, y: Float)
        if distToTarget < approachDecelerationRadius {
            lookaheadPt = target
        } else {
            lookaheadPt = findLookaheadPoint(pos: pos)
        }
        
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
        
        // ---------------------------------------------------------------
        // Corridor obstacle check: scan a car-width strip along the
        // travel heading. This is the ONLY obstacle check that controls
        // navigation. The radial bubble is display-only.
        //
        // IMPORTANT: Only check when the robot is roughly facing the
        // travel direction. When the heading error is large the robot
        // will spin in place first — obstacle data along a heading the
        // camera isn't pointed toward is unreliable and would cause
        // false obstacle-avoidance triggers instead of a simple turn.
        // Use the same threshold as the spin-in-place decision
        // so the spin always gets priority over obstacle avoidance.
        // Apply hysteresis: use the exit threshold if already spinning.
        // ---------------------------------------------------------------
        // When bearing correction is active (robot was knocked off course),
        // use tighter thresholds so the robot spins sooner and corrects faster.
        let spinEnterThreshold: Float = isBearingCorrecting ? .pi / 18.0 : .pi / 9.0   // 10° vs 20°
        let spinExitThreshold:  Float = isBearingCorrecting ? .pi / 36.0 : .pi / 18.0  // 5° vs 10°
        let activeSpinThreshold = wasSpinning ? spinExitThreshold : spinEnterThreshold
        var headingErrorForCheck = targetHeading - pos.heading
        while headingErrorForCheck >  .pi { headingErrorForCheck -= 2 * .pi }
        while headingErrorForCheck < -.pi { headingErrorForCheck += 2 * .pi }
        let needsLargeTurn = fabsf(headingErrorForCheck) > activeSpinThreshold
        
        let corridorDist: Float?
        let inObstacleCooldown = Date().timeIntervalSince(lastObstacleStopTime) < obstacleStopCooldown
        if !inGracePeriod && !needsLargeTurn && !awaitingReplan && !inObstacleCooldown {
            let d1 = detector.corridorObstacleDistance(heading: targetHeading)
            let d2 = (targetHeading == waypointHeading) ? nil
                : detector.corridorObstacleDistance(heading: waypointHeading)
            switch (d1, d2) {
            case let (a?, b?): corridorDist = min(a, b)
            case let (a?, nil): corridorDist = a
            case let (nil, b?): corridorDist = b
            case (nil, nil):    corridorDist = nil
            }
        } else {
            corridorDist = nil
        }
        
        if let dist = corridorDist, dist <= detector.corridorStopDistance {
            // Near the final destination? Declare arrived early.
            if distToTarget < arrivalThreshold * 3 {
                ESP32BLEManager.shared.stopAll()
                state = .arrived
                log("✅ Arrived near target (obstacle in final approach)")
                navTimer?.invalidate()
                navTimer = nil
                return
            }
            
            // Obstacle ahead — stop and request a replan that routes around it.
            // Do NOT spin — spinning causes feedback loops where the new heading
            // also hits obstacles. Just stop, let the planner find a clear route.
            ESP32BLEManager.shared.stopAll()
            awaitingReplan = true
            lastObstacleStopTime = Date()
            lastReplanTime = .distantPast
            
            log("⏸️ Obstacle at \(String(format: "%.0f", dist * 1000))mm — stopping for replan")
            if state != .paused {
                state = .paused
                pauseStartTime = Date()
            }
            return
        }
        
        // Slow-zone factor: 1.0 = full speed, 0.5 = half speed near obstacles
        let corridorSlowFactor: Float
        if let dist = corridorDist, dist <= detector.slowZoneRadius {
            let range = detector.slowZoneRadius - detector.corridorStopDistance
            let progress = (dist - detector.corridorStopDistance) / max(range, 0.01)
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
        
        // Threshold for "large turn" — above this we spin in place instead of arcing.
        // Uses hysteresis to prevent oscillation at the boundary.
        let sharpTurnThreshold: Float = activeSpinThreshold
        
        let isSpinning = absError > sharpTurnThreshold
        
        // Post-spin settle: when spinning just finished (was spinning, now
        // heading is aligned), stop and replan so the path accounts for
        // scans gathered during the spin and the new orientation.
        if wasSpinning && !isSpinning && postSpinSettleStart == nil {
            ESP32BLEManager.shared.stopAll()
            postSpinSettleStart = Date()
            lastReplanTime = .distantPast  // request replan
            log("🔄 Spin done — settling and replanning")
        }
        
        if let settleStart = postSpinSettleStart {
            if Date().timeIntervalSince(settleStart) < postSpinSettleDuration {
                // Still settling — hold motors stopped, wait for replan
                ESP32BLEManager.shared.stopAll()
                wasSpinning = isSpinning
                return
            } else {
                // Settle done — resume driving.
                // Reset stuck detection so the 1s of intentional stillness
                // during settle doesn't trigger a reverse escape.
                postSpinSettleStart = nil
                lastResumeTime = Date()
                lastStuckCheckPos = nil
                lastStuckCheckTime = nil
                log("▶️ Post-spin settle done — resuming")
            }
        }
        
        wasSpinning = isSpinning
        
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
                        // Record this stuck location so the planner avoids it
                        recordStuckZone(pos: pos)
                        
                        startNextEscapeMove(pos: pos)
                        state = .paused
                        pauseStartTime = Date()
                        lastStuckCheckPos = (pos.x, pos.y)
                        lastStuckCheckTime = Date()
                        return
                    }
                    
                    // Car moved (not stuck this interval) — but only reset
                    // escape counter if we've made meaningful forward progress
                    // since the last escape maneuver (prevents ARKit jitter resets).
                    if let escStart = escapeStartPos {
                        let progressDx = pos.x - escStart.x
                        let progressDy = pos.y - escStart.y
                        let progressDist = sqrtf(progressDx * progressDx + progressDy * progressDy)
                        if progressDist >= resetProgressDistance {
                            escapeAttemptCount = 0
                            escapeStartPos = nil
                        }
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
                // Record this stuck location so the planner avoids it
                recordStuckZone(pos: pos)
                
                // Spinning hasn't changed position — force a reverse first
                // to physically move before trying other escape strategies.
                if !isObstacleBehind(pos: pos) {
                    log("🔧 Spin-stuck — forcing reverse")
                    startReverseNudge(turnDirection: 0)
                    escapeAttemptCount += 1
                    if escapeStartPos == nil {
                        escapeStartPos = (pos.x, pos.y)
                    }
                } else {
                    startNextEscapeMove(pos: pos)
                }
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
            // Pulse-spin: alternate between a short motor burst and a coast
            // gap. During coast the robot decelerates and ARKit heading
            // catches up, preventing overshoot.
            let now = Date()
            if spinPhaseStart == nil {
                spinPhase = .burst
                spinPhaseStart = now
            }
            let phaseElapsed = now.timeIntervalSince(spinPhaseStart!)
            
            switch spinPhase {
            case .burst:
                if phaseElapsed >= spinBurstDuration {
                    // Switch to coast — stop motors and return
                    spinPhase = .coast
                    spinPhaseStart = now
                    ESP32BLEManager.shared.stopAll()
                    wasSpinning = isSpinning
                    return
                } else {
                    // Apply spin power — use higher power during bearing correction
                    // for a faster, more assertive recovery from knockoff.
                    let minSpin: Float = isBearingCorrecting ? 0.55 : 0.50
                    let maxSpin: Float = isBearingCorrecting ? 0.75 : 0.65
                    let errorFraction = min(absError / .pi, 1.0)
                    let spinPower = minSpin + (maxSpin - minSpin) * errorFraction
                    leftPower  =  turnSign * spinPower
                    rightPower = -turnSign * spinPower
                }
            case .coast:
                if phaseElapsed >= spinCoastDuration {
                    // Switch to burst
                    spinPhase = .burst
                    spinPhaseStart = now
                }
                // Motors off — coasting/settling. Use stopAll and return
                // early so we don't send any motor command this tick.
                ESP32BLEManager.shared.stopAll()
                wasSpinning = isSpinning
                return
            }
        } else {
            // Reset pulse-spin state when not spinning
            spinPhase = .burst
            spinPhaseStart = nil
            // Small correction: proportional differential steering while driving forward.
            // Reduce cruise speed proportionally when heading error is large to
            // avoid carrying too much speed into a turn.
            // Apply corridor slow factor when approaching obstacles ahead.
            // Use higher turn gain during bearing correction for faster recovery.
            let turnGain: Float = isBearingCorrecting ? 2.0 : 1.2
            let turn = max(-1.0, min(1.0, headingError * turnGain))
            let slowdownFactor: Float = 1.0 - 0.5 * (absError / sharpTurnThreshold)
            let basePower: Float = max(0.40, cruiseSpeed * max(0.5, slowdownFactor) * corridorSlowFactor)
            // Apply approach deceleration near target
            let approachPower: Float = max(approachMinPower, basePower * approachFactor)
            leftPower  = approachPower + turn * approachPower
            rightPower = approachPower - turn * approachPower
            // Ensure both wheels are always above the dead zone (30%).
            // If a wheel would drop below 30%, clamp it to 30% so it
            // keeps contributing torque rather than stalling.
            leftPower  = max(0.30, leftPower)
            rightPower = max(0.30, rightPower)
        }
        
        // Clamp to [-100%, 100%] and enforce minimum |30%| (dead zone avoidance)
        // Values between -30% and 30% don't have enough torque to move.
        // Zero is allowed (motor off).
        func clampPower(_ p: Float) -> Int8 {
            let clamped = max(-1.0, min(1.0, p))
            let scaled = Int(clamped * 100)
            if scaled == 0 { return 0 }
            if scaled > 0 && scaled < 30 { return 30 }
            if scaled < 0 && scaled > -30 { return -30 }
            return Int8(clamping: scaled)
        }
        
        let leftInt  = clampPower(leftPower)
        let rightInt = clampPower(rightPower)
        
        // A & C = left side, B & D = right side
        ESP32BLEManager.shared.setAllMotors(a: leftInt, b: rightInt, c: leftInt, d: rightInt)
    }
    
    // MARK: - Stuck Zone Recording
    
    /// Record the current position + travel heading as a stuck zone in the
    /// occupancy grid so future A* plans route around it.
    private func recordStuckZone(pos: DevicePosition) {
        guard let grid = occupancyGrid else { return }
        
        // Determine the heading the robot was trying to travel
        let travelHeading: Float
        if currentWaypointIndex < currentPath.count {
            let wp = currentPath[currentWaypointIndex]
            travelHeading = atan2f(wp.x - pos.x, wp.y - pos.y)
        } else if let target = targetPoint {
            travelHeading = atan2f(target.x - pos.x, target.y - pos.y)
        } else {
            travelHeading = pos.heading
        }
        
        grid.recordStuckZone(worldX: pos.x, worldY: pos.y, heading: travelHeading)
        log("📌 Recorded stuck zone at (\(String(format: "%.2f", pos.x)), \(String(format: "%.2f", pos.y))) heading \(String(format: "%.0f°", travelHeading * 180 / .pi))")
    }
    
    // MARK: - Reverse Direction Check
    
    /// Returns `true` if there's an obstacle directly behind the car
    /// (within the corridor stop distance in the reverse direction).
    private func isObstacleBehind(pos: DevicePosition) -> Bool {
        let detector = ObstacleDetector.shared
        // Reverse heading = current heading + 180°
        var reverseHeading = pos.heading + .pi
        if reverseHeading >  .pi { reverseHeading -= 2 * .pi }
        if reverseHeading < -.pi { reverseHeading += 2 * .pi }
        if let dist = detector.corridorObstacleDistance(heading: reverseHeading, corridorLength: 0.20) {
            return dist < 0.15  // wall within 15cm behind
        }
        return false
    }
    
    // MARK: - Escape Strategy Selection
    
    /// Pick the next escape move from the sequence, skipping moves that are
    /// unsafe given the current surroundings (e.g., don't reverse into a wall).
    private func pickEscapeMove(pos: DevicePosition) -> EscapeMove {
        let wallBehind = isObstacleBehind(pos: pos)
        let detector = ObstacleDetector.shared
        let wallAhead = detector.corridorObstacleDistance(heading: pos.heading)
            .map { $0 < 0.15 } ?? false
        
        let startIdx = escapeAttemptCount % escapeSequence.count
        for offset in 0..<escapeSequence.count {
            let idx = (startIdx + offset) % escapeSequence.count
            let candidate = escapeSequence[idx]
            switch candidate {
            case .reverseStraight, .reverseLeft, .reverseRight:
                if wallBehind { continue }
            case .forwardLeft, .forwardRight:
                if wallAhead { continue }
            case .spinLeft, .spinRight:
                break  // always safe
            }
            return candidate
        }
        // All filtered — default to spin
        return escapeAttemptCount % 2 == 0 ? .spinLeft : .spinRight
    }
    
    /// Execute the next escape maneuver from the diverse strategy sequence.
    private func startNextEscapeMove(pos: DevicePosition) {
        let move = pickEscapeMove(pos: pos)
        escapeAttemptCount += 1
        if escapeStartPos == nil {
            escapeStartPos = (pos.x, pos.y)
        }
        
        log("🔧 Escape #\(escapeAttemptCount) — \(move.rawValue)")
        
        switch move {
        case .reverseStraight:
            startReverseNudge(turnDirection: 0)
        case .reverseLeft:
            startReverseNudge(turnDirection: -1)
        case .reverseRight:
            startReverseNudge(turnDirection: 1)
        case .forwardLeft:
            startForwardEscape(pos: pos, forcedSign: -1)
        case .forwardRight:
            startForwardEscape(pos: pos, forcedSign: 1)
        case .spinLeft:
            startSpinEscape(direction: -1)
        case .spinRight:
            startSpinEscape(direction: 1)
        }
    }
    
    // MARK: - Reverse Nudge
    
    /// Reverse nudge with optional turn direction.
    /// `turnDirection`: 0 = straight, -1 = turn left while reversing, +1 = turn right
    private func startReverseNudge(turnDirection: Float = 0) {
        isReversing = true
        reversePhase = 0
        reverseStartTime = Date()
        reverseTurnDirection = turnDirection
        
        let basePower: Int8 = -50
        let left: Int8, right: Int8
        if turnDirection < 0 {
            // Reverse + turn left: right side reverses harder
            left = -35; right = basePower
        } else if turnDirection > 0 {
            // Reverse + turn right: left side reverses harder
            left = basePower; right = -35
        } else {
            left = basePower; right = basePower
        }
        ESP32BLEManager.shared.setAllMotors(a: left, b: right, c: left, d: right)
        
        let dirLabel = turnDirection < 0 ? "+left" : turnDirection > 0 ? "+right" : ""
        log("↩️ Reverse nudge \(dirLabel)...")
    }
    
    /// Called each tick while isReversing — manages the nudge phases.
    private func tickReverse() {
        guard let startTime = reverseStartTime else {
            finishReverseNudge()
            return
        }
        
        let elapsed = Date().timeIntervalSince(startTime)
        
        switch reversePhase {
        case 0:
            if elapsed >= reverseDuration {
                ESP32BLEManager.shared.stopAll()
                reversePhase = 1
                reverseStartTime = Date()
            }
        default:
            if elapsed >= reversePauseDuration {
                finishReverseNudge()
            }
        }
    }
    
    /// End the nudge — try to spin toward the target, then resume navigation.
    private func finishReverseNudge() {
        ESP32BLEManager.shared.stopAll()
        isReversing = false
        reversePhase = 0
        reverseStartTime = nil
        awaitingReplan = true
        lastReplanTime = .distantPast
        
        // After nudge, try to spin toward the target
        if let target = targetPoint, let grid = occupancyGrid {
            let pos = grid.devicePosition
            let goalHeading = atan2f(target.x - pos.x, target.y - pos.y)
            let detector = ObstacleDetector.shared
            if let result = detector.findClearHeading(preferred: goalHeading, currentHeading: pos.heading) {
                spinToTargetHeading = result.heading
                spinToTargetSign = result.spinSign
                isSpinningToTarget = true
                spinToTargetStartTime = Date()
                // Stay paused until replan arrives — spin will wait
                state = .paused
                pauseStartTime = Date()
                log("↩️ Nudge done — spinning toward clear heading")
                return
            }
        }
        
        // No clear heading found — wait for replan in paused state
        state = .paused
        pauseStartTime = Date()
        log("↩️ Nudge done — waiting for replan")
    }
    
    // MARK: - Forward Escape Maneuver
    
    /// Drive forward while turning — direction can be forced or auto-detected.
    private func startForwardEscape(pos: DevicePosition, forcedSign: Float? = nil) {
        isForwardEscaping = true
        forwardEscapeStartTime = Date()
        
        if let sign = forcedSign {
            forwardEscapeSign = sign
        } else {
            let detector = ObstacleDetector.shared
            if let target = targetPoint {
                let goalHeading = atan2f(target.x - pos.x, target.y - pos.y)
                forwardEscapeSign = detector.bestSpinDirection(from: pos.heading, to: goalHeading)
            } else {
                forwardEscapeSign = 1.0
            }
        }
        
        // Drive forward with a strong turn
        let innerPower: Int8 = 30   // slow inner wheel
        let outerPower: Int8 = 55   // faster outer wheel
        let left: Int8, right: Int8
        if forwardEscapeSign > 0 {
            left = outerPower; right = innerPower
        } else {
            left = innerPower; right = outerPower
        }
        ESP32BLEManager.shared.setAllMotors(a: left, b: right, c: left, d: right)
        log("🏃 Forward escape — turning \(forwardEscapeSign > 0 ? "right" : "left")")
    }
    
    /// Called each tick while forward-escaping.
    private func tickForwardEscape() {
        guard let start = forwardEscapeStartTime else {
            finishForwardEscape()
            return
        }
        if Date().timeIntervalSince(start) >= forwardEscapeDuration {
            finishForwardEscape()
        }
    }
    
    /// End forward escape, request replan.
    private func finishForwardEscape() {
        ESP32BLEManager.shared.stopAll()
        isForwardEscaping = false
        forwardEscapeStartTime = nil
        awaitingReplan = true
        lastReplanTime = .distantPast
        state = .paused
        pauseStartTime = Date()
        lastStuckCheckPos = nil
        lastStuckCheckTime = nil
        log("🏃 Forward escape done — waiting for replan")
    }
    
    // MARK: - Spin Escape Maneuver
    
    /// Pure spin in place — used when both forward and reverse are blocked.
    private func startSpinEscape(direction: Float) {
        isSpinEscaping = true
        spinEscapeStartTime = Date()
        spinEscapeSign = direction
        
        let spinPower: Float = 0.55
        let left  = Int8(clamping: Int( direction * spinPower * 100))
        let right = Int8(clamping: Int(-direction * spinPower * 100))
        ESP32BLEManager.shared.setAllMotors(a: left, b: right, c: left, d: right)
        log("🔄 Spin escape — \(direction > 0 ? "right" : "left")")
    }
    
    /// Called each tick while spin-escaping.
    private func tickSpinEscape() {
        guard let start = spinEscapeStartTime else {
            finishSpinEscape()
            return
        }
        if Date().timeIntervalSince(start) >= spinEscapeDuration {
            finishSpinEscape()
        }
    }
    
    /// End spin escape, request replan.
    private func finishSpinEscape() {
        ESP32BLEManager.shared.stopAll()
        isSpinEscaping = false
        spinEscapeStartTime = nil
        awaitingReplan = true
        lastReplanTime = .distantPast
        state = .paused
        pauseStartTime = Date()
        lastStuckCheckPos = nil
        lastStuckCheckTime = nil
        log("🔄 Spin escape done — waiting for replan")
    }
    
    // MARK: - Spin-to-Target Maneuver
    
    /// Spin in place toward the computed clear heading, then resume navigation.
    private func tickSpinToTarget(pos: DevicePosition) {
        var headingError = spinToTargetHeading - pos.heading
        // Normalize to [-π, π]
        while headingError >  .pi { headingError -= 2 * .pi }
        while headingError < -.pi { headingError += 2 * .pi }
        
        let absError = fabsf(headingError)
        
        // Close enough to target heading — done spinning
        if absError < 0.25 {  // ~14°
            ESP32BLEManager.shared.stopAll()
            isSpinningToTarget = false
            spinToTargetStartTime = nil
            if awaitingReplan {
                // New path hasn't arrived yet — wait in paused state
                // instead of navigating with the stale path.
                state = .paused
                pauseStartTime = Date()
                log("🔄 Spin complete — waiting for replan")
            } else {
                state = .navigating
                lastResumeTime = Date()
                log("🔄 Spin complete — resuming navigation")
            }
            lastReplanTime = .distantPast  // force replan from new orientation
            return
        }
        
        // Timeout — give up and let replan handle it
        if let start = spinToTargetStartTime,
           Date().timeIntervalSince(start) > maxSpinToTargetDuration {
            ESP32BLEManager.shared.stopAll()
            isSpinningToTarget = false
            spinToTargetStartTime = nil
            if awaitingReplan {
                state = .paused
                pauseStartTime = Date()
                log("🔄 Spin timed out — waiting for replan")
            } else {
                state = .navigating
                lastResumeTime = Date()
                log("🔄 Spin timed out — resuming with replan")
            }
            lastReplanTime = .distantPast
            return
        }
        
        // Spin in the predetermined direction (away from obstacle)
        let turnSign: Float = spinToTargetSign
        let spinPower: Float = 0.55
        
        func clampPower(_ p: Float) -> Int8 {
            let clamped = max(-1.0, min(1.0, p))
            let scaled = Int(clamped * 100)
            if scaled == 0 { return 0 }
            if scaled > 0 && scaled < 30 { return 30 }
            if scaled < 0 && scaled > -30 { return -30 }
            return Int8(clamping: scaled)
        }
        
        let leftPower  = clampPower( turnSign * spinPower)
        let rightPower = clampPower(-turnSign * spinPower)
        ESP32BLEManager.shared.setAllMotors(a: leftPower, b: rightPower, c: leftPower, d: rightPower)
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
