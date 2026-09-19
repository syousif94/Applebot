//
//  ExplorationController.swift
//  RoboCar
//
//  Created by Sammy Yousif on 2/17/26.
//

import Foundation
import simd

/// Autonomous exploration controller that drives the car to map
/// the entire reachable area bounded by walls, doors, and obstacles.
///
/// Algorithm:
///   1. Find frontier clusters (unknown cells adjacent to free cells)
///   2. For each frontier, snap its centroid to the nearest free cell —
///      the edge waypoint that sits in explored space with no wall between
///      it and the unknown region
///   3. Use A* (free cells only) to build a navigable path to the edge
///      waypoint; fall back to greedy A* (unknown cells traversable) when
///      no strictly free path exists
///   4. Follow each path waypoint: turn-to-face then drive
///   5. After reaching the waypoint set, pause for LiDAR to update the
///      map and replan from the new frontiers
///   6. Stop when no frontiers remain or the time limit is reached
class ExplorationController {
    
    // MARK: - Singleton
    
    static let shared = ExplorationController()
    
    // MARK: - Configuration
    
    /// Driving speed (0–100)
    let driveSpeed: Int8 = 35
    
    /// Turning speed (0–100)
    let turnSpeed: Int8 = 40
    
    /// How close to a frontier target we need to be (meters)
    let arrivalThreshold: Float = 0.30
    
    /// Angular tolerance for "facing the target" (radians, ~15°)
    let headingTolerance: Float = 0.26
    
    /// How often to re-evaluate the target (seconds)
    let replanInterval: TimeInterval = 2.0
    
    /// Max time to spend exploring before giving up (seconds)
    let maxExplorationTime: TimeInterval = 300  // 5 minutes
    
    /// Minimum time to drive forward before re-evaluating (seconds)
    let minDriveStep: TimeInterval = 0.3
    
    /// Time to wait for new map data after stopping (seconds)
    let scanPause: TimeInterval = 1.5
    
    // MARK: - State
    
    private(set) var isExploring = false
    private var explorationTask: Task<String, Error>?
    
    /// Callback for status updates
    var onStatusUpdate: ((String) -> Void)?
    
    private init() {}
    
    // MARK: - Public API
    
    /// Start autonomous exploration. Returns a summary when complete.
    func startExploration() async throws -> String {
        guard !isExploring else {
            return "Exploration is already in progress."
        }
        
        guard let grid = ObstacleDetector.shared.occupancyGrid else {
            return "No occupancy grid available. Make sure LiDAR is running."
        }
        
        isExploring = true
        defer { isExploring = false }
        
        let startTime = Date()
        var totalTurns = 0
        var totalDriveSegments = 0
        var stuckCount = 0
        let maxStuck = 5
        
        log("🗺️ Starting autonomous exploration…")
        
        // Initial 360° scan to seed the map
        log("📡 Performing initial scan…")
        try await performScan()
        
        while true {
            // Check time limit
            let elapsed = Date().timeIntervalSince(startTime)
            if elapsed > maxExplorationTime {
                log("⏱️ Time limit reached (\(Int(elapsed))s)")
                break
            }
            
            // Check cancellation
            try Task.checkCancellation()
            
            // Find frontiers (unknown cells adjacent to free cells)
            let frontiers = grid.findFrontierClusters(maxClusters: 10, minClusterSize: 2)

            if frontiers.isEmpty {
                log("✅ No more frontiers — area fully mapped!")
                break
            }

            log("🔍 Found \(frontiers.count) frontier(s), largest has \(frontiers[0].size) cells")

            let pos = grid.devicePosition
            var navPath: [(x: Float, y: Float)] = []
            var navTarget: (x: Float, y: Float)? = nil

            // Try each frontier (sorted nearest-first by findFrontierClusters)
            for frontier in frontiers.prefix(5) {
                // The frontier centroid is in unknown space. Snap it to the nearest
                // free cell — this is the edge waypoint that lies in explored,
                // wall-free space right at the boundary of the unknown region.
                guard let edgePoint = grid.nearestFreeWorldPoint(x: frontier.x, y: frontier.y) else { continue }

                let dx = edgePoint.x - pos.x
                let dy = edgePoint.y - pos.y
                let dist = sqrtf(dx * dx + dy * dy)
                if dist < arrivalThreshold { continue }

                // Prefer a path through known free cells only
                let strictPath = grid.findPath(fromX: pos.x, fromY: pos.y,
                                               toX: edgePoint.x, toY: edgePoint.y)
                if !strictPath.isEmpty {
                    navPath = strictPath
                    navTarget = (edgePoint.x, edgePoint.y)
                    log("🗺️ A* path: \(strictPath.count) waypoints, \(String(format: "%.1f", dist))m to frontier edge")
                    break
                }

                // Fall back: greedy path that may cross unknown cells to reach
                // the frontier centroid directly
                let greedyPath = grid.findPathGreedy(fromX: pos.x, fromY: pos.y,
                                                     toX: frontier.x, toY: frontier.y)
                if !greedyPath.isEmpty {
                    navPath = greedyPath
                    navTarget = (frontier.x, frontier.y)
                    log("🗺️ Greedy path: \(greedyPath.count) waypoints toward frontier")
                    break
                }
            }

            guard navTarget != nil else {
                log("📡 No reachable frontier, scanning…")
                try await performScan()
                stuckCount += 1
                if stuckCount >= maxStuck {
                    log("🔄 Stuck too many times, finishing.")
                    break
                }
                continue
            }

            stuckCount = 0

            // Follow each path waypoint: turn then drive
            for waypoint in navPath {
                try Task.checkCancellation()

                // Skip waypoints we've already passed
                let wp = grid.devicePosition
                let wdx = waypoint.x - wp.x
                let wdy = waypoint.y - wp.y
                if sqrtf(wdx * wdx + wdy * wdy) < arrivalThreshold { continue }

                let turned = try await turnToward(x: waypoint.x, y: waypoint.y, grid: grid)
                if turned { totalTurns += 1 }

                let drove = try await driveToward(x: waypoint.x, y: waypoint.y, grid: grid)
                if drove { totalDriveSegments += 1 }

                // Obstacle blocked this path — replan from current position
                if ObstacleDetector.shared.obstacleDetected { break }
            }

            // Pause for LiDAR to update the map before replanning
            try await Task.sleep(for: .seconds(scanPause))
        }
        
        // Stop motors
        await MainActor.run { ESP32BLEManager.shared.stopAll() }
        
        let elapsed = Int(Date().timeIntervalSince(startTime))
        let summary = "Exploration complete in \(elapsed)s. " +
            "\(grid.freeCount) free cells, \(grid.occupiedCount) occupied cells. " +
            "\(totalDriveSegments) drive segments, \(totalTurns) turns."
        log(summary)
        return summary
    }
    
    /// Stop exploration
    func stopExploration() {
        explorationTask?.cancel()
        explorationTask = nil
        isExploring = false
        ESP32BLEManager.shared.stopAll()
        log("🛑 Exploration stopped by user")
    }
    
    // MARK: - Navigation Primitives
    
    /// Turn in place to face a world-coordinate target. Returns true if a turn was needed.
    private func turnToward(x targetX: Float, y targetY: Float, grid: OccupancyGrid) async throws -> Bool {
        let pos = grid.devicePosition
        let dx = targetX - pos.x
        let dy = targetY - pos.y
        let targetHeading = atan2f(dx, dy)
        
        var angleDiff = targetHeading - pos.heading
        while angleDiff > .pi  { angleDiff -= 2 * .pi }
        while angleDiff < -.pi { angleDiff += 2 * .pi }
        
        // Already facing roughly the right way
        if fabsf(angleDiff) < headingTolerance {
            return false
        }
        
        log("↩️ Turning \(angleDiff > 0 ? "right" : "left") \(Int(fabsf(angleDiff) * 180 / .pi))°")
        
        let turnPower = turnSpeed
        
        // Turn until heading is close enough
        let startTime = Date()
        let maxTurnTime: TimeInterval = 5.0
        
        while true {
            try Task.checkCancellation()
            
            if Date().timeIntervalSince(startTime) > maxTurnTime { break }
            
            let currentPos = grid.devicePosition
            var currentDiff = targetHeading - currentPos.heading
            while currentDiff > .pi  { currentDiff -= 2 * .pi }
            while currentDiff < -.pi { currentDiff += 2 * .pi }
            
            if fabsf(currentDiff) < headingTolerance {
                break
            }
            
            // Turn right (positive angleDiff) or left
            if currentDiff > 0 {
                // Right: left wheels forward, right wheels backward
                await MainActor.run {
                    ESP32BLEManager.shared.setAllMotors(a: turnPower, b: -turnPower, c: turnPower, d: -turnPower)
                }
            } else {
                // Left: left wheels backward, right wheels forward
                await MainActor.run {
                    ESP32BLEManager.shared.setAllMotors(a: -turnPower, b: turnPower, c: -turnPower, d: turnPower)
                }
            }
            
            try await Task.sleep(for: .milliseconds(50))
        }
        
        await MainActor.run { ESP32BLEManager.shared.stopAll() }
        try await Task.sleep(for: .milliseconds(200))
        return true
    }
    
    /// Drive forward toward a target point. Stops if obstacle detected or arrived.
    /// Returns true if any forward progress was made.
    private func driveToward(x targetX: Float, y targetY: Float, grid: OccupancyGrid) async throws -> Bool {
        let startPos = grid.devicePosition
        let dx = targetX - startPos.x
        let dy = targetY - startPos.y
        let totalDist = sqrtf(dx * dx + dy * dy)
        
        if totalDist < arrivalThreshold {
            return false
        }
        
        log("🚗 Driving \(String(format: "%.1f", totalDist))m toward frontier")
        
        let startTime = Date()
        let maxDriveTime: TimeInterval = min(Double(totalDist / (Float(driveSpeed) * 0.003)), 8.0)
        var madeProgress = false
        
        // Start driving forward
        await MainActor.run {
            ESP32BLEManager.shared.setAllMotors(a: driveSpeed, b: driveSpeed, c: driveSpeed, d: driveSpeed)
        }
        
        while true {
            try Task.checkCancellation()
            
            let elapsed = Date().timeIntervalSince(startTime)
            if elapsed > maxDriveTime { break }
            
            let pos = grid.devicePosition
            let remainDX = targetX - pos.x
            let remainDY = targetY - pos.y
            let remainDist = sqrtf(remainDX * remainDX + remainDY * remainDY)
            
            // Check if we've arrived
            if remainDist < arrivalThreshold {
                madeProgress = true
                break
            }
            
            // Check if heading drifted too much (need to re-aim)
            let targetHeading = atan2f(remainDX, remainDY)
            var headingErr = targetHeading - pos.heading
            while headingErr > .pi  { headingErr -= 2 * .pi }
            while headingErr < -.pi { headingErr += 2 * .pi }
            
            if fabsf(headingErr) > headingTolerance * 2 {
                // Heading drifted — stop and re-turn
                break
            }
            
            // Check if obstacle detector is blocking us
            if ObstacleDetector.shared.obstacleDetected {
                log("🛑 Obstacle detected, stopping drive")
                break
            }
            
            // Track progress
            let movedDX = pos.x - startPos.x
            let movedDY = pos.y - startPos.y
            if sqrtf(movedDX * movedDX + movedDY * movedDY) > 0.05 {
                madeProgress = true
            }
            
            try await Task.sleep(for: .milliseconds(50))
        }
        
        await MainActor.run { ESP32BLEManager.shared.stopAll() }
        try await Task.sleep(for: .milliseconds(200))
        
        // If obstacle blocked us, try to back up slightly and turn
        if ObstacleDetector.shared.obstacleDetected {
            try await avoidObstacle()
        }
        
        return madeProgress
    }
    
    /// Back up slightly and turn to escape an obstacle
    private func avoidObstacle() async throws {
        log("↩️ Avoiding obstacle — backing up")
        
        // Reverse briefly
        await MainActor.run {
            ESP32BLEManager.shared.setAllMotors(a: -driveSpeed, b: -driveSpeed, c: -driveSpeed, d: -driveSpeed)
        }
        try await Task.sleep(for: .seconds(0.5))
        await MainActor.run { ESP32BLEManager.shared.stopAll() }
        try await Task.sleep(for: .milliseconds(200))
        
        // Turn 90° in a random direction
        let turnRight = Bool.random()
        let tp = turnSpeed
        log("↩️ Turning \(turnRight ? "right" : "left") to find new path")
        
        await MainActor.run {
            if turnRight {
                ESP32BLEManager.shared.setAllMotors(a: tp, b: -tp, c: tp, d: -tp)
            } else {
                ESP32BLEManager.shared.setAllMotors(a: -tp, b: tp, c: -tp, d: tp)
            }
        }
        try await Task.sleep(for: .seconds(0.8))
        await MainActor.run { ESP32BLEManager.shared.stopAll() }
        try await Task.sleep(for: .milliseconds(300))
    }
    
    /// Perform a slow 360° scan to gather map data
    private func performScan() async throws {
        let tp = Int8(25)  // Slow turn for scanning
        
        await MainActor.run {
            ESP32BLEManager.shared.setAllMotors(a: tp, b: -tp, c: tp, d: -tp)
        }
        
        // Turn for ~4 seconds (roughly 360° at low speed)
        try await Task.sleep(for: .seconds(4.0))
        
        await MainActor.run { ESP32BLEManager.shared.stopAll() }
        try await Task.sleep(for: .seconds(1.0))
    }
    
    // MARK: - Logging
    
    private func log(_ message: String) {
        print("[Explore] \(message)")
        onStatusUpdate?(message)
        
        // Telemetry: emit exploration event
        let grid = ObstacleDetector.shared.occupancyGrid
        let cameraTransform = TelemetryService.shared.lastCameraTransform
        let status: String
        if message.contains("Starting") { status = "started" }
        else if message.contains("scan") || message.contains("Scan") { status = "scanning" }
        else if message.contains("frontier") || message.contains("Frontier") { status = "target_selected" }
        else if message.contains("Turning") { status = "turning" }
        else if message.contains("Driving") { status = "driving" }
        else if message.contains("Stuck") || message.contains("stuck") { status = "stuck" }
        else if message.contains("complete") || message.contains("fully mapped") { status = "completed" }
        else if message.contains("stopped") || message.contains("Stopped") { status = "stopped" }
        else if message.contains("Time limit") { status = "time_limit" }
        else { status = "info" }
        
        TelemetryService.shared.logExplorationEvent(
            status: status,
            message: message,
            occupancyGrid: grid,
            cameraTransform: cameraTransform
        )
    }
}
