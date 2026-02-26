//
//  TelemetryService.swift
//  RoboCar
//
//  WebSocket client + local JSONL persistence for navigation telemetry.
//  Uses URLSessionWebSocketTask (built into Foundation, zero dependencies).
//
//  Usage:
//    TelemetryService.shared.start(serverURL: "ws://192.168.1.100:8765")
//    // called automatically from LiDARViewController.updateFrame()
//    TelemetryService.shared.tick(pose:cameraTransform:)
//

import Foundation
import ARKit
import simd
import UIKit

// MARK: - TelemetryService

final class TelemetryService {

    // MARK: - Singleton

    static let shared = TelemetryService()

    // MARK: - Configuration

    /// UserDefaults key for server URL
    static let serverURLKey = "telemetryServerURL"
    static let defaultServerURL = "ws://192.168.1.100:8765"

    /// Server WebSocket URL — persisted in UserDefaults
    var serverURL: String {
        get { UserDefaults.standard.string(forKey: Self.serverURLKey) ?? Self.defaultServerURL }
        set { UserDefaults.standard.set(newValue, forKey: Self.serverURLKey) }
    }

    /// How many nav_frames per second (~10 Hz)
    private let framesPerSecond = 10

    /// How many nav_frames between mesh snapshots (~1 Hz → every 10 frames)
    private let meshSnapshotInterval = 10

    /// Maximum recent logs to keep for crash breadcrumb
    private let maxRecentLogs = 5

    /// Maximum lines buffered in memory before dropping oldest
    private let maxBufferedLines = 5000

    /// Flush file every N events or this interval, whichever first
    private let flushFileInterval: TimeInterval = 1.0
    private let flushFileCount = 100

    // MARK: - State

    private(set) var sessionId = UUID().uuidString
    private var seq: UInt64 = 0
    private(set) var isRunning = false
    private var framesSinceLastMesh = 0
    private var totalFrames: UInt64 = 0
    private var totalRoutes = 0
    private var totalObstacleEvents = 0
    private var totalReplans = 0
    private var sessionStartDate = Date()

    /// Active route ID (set by logRoutePlanned, cleared on nav idle)
    private(set) var currentRouteId: String?

    /// Recent log messages for crash breadcrumb
    private var recentLogs: [String] = []

    // MARK: - Last known steering state (set by PathNavigator integration)

    struct LastSteeringSnapshot {
        var mode: String = "stopped"
        var headingError: Float = 0
        var speedFactor: Float = 1
        var depthEvasionBias: Float = 0
        var spinPower: Float = 0
        var arcTurn: Float = 0
        var lookaheadDist: Float = 0
        var lookaheadPoint: Vec2 = Vec2(x: 0, y: 0)
        var targetHeading: Float = 0
        var stuckDetected: Bool = false
        var distSinceStuckCheck: Float? = nil
        var motorCmdRaw: MotorSnapshot = .zero
    }

    /// Updated by PathNavigator each tick (before obstacle filtering)
    var lastSteering = LastSteeringSnapshot()

    // MARK: - Camera transform (set by LiDARViewController each frame)

    var lastCameraTransform: simd_float4x4 = matrix_identity_float4x4

    // MARK: - WebSocket

    private var session: URLSession?
    private var wsTask: URLSessionWebSocketTask?
    private var isConnected = false
    private var reconnectTimer: Timer?
    private let reconnectInterval: TimeInterval = 3.0

    // MARK: - File Persistence

    private var fileHandle: FileHandle?
    private var filePath: URL?
    private var linesSinceFlush = 0
    private var lastFlushTime = Date()

    // MARK: - Breadcrumb

    private var breadcrumbPath: URL?
    private var breadcrumbFd: Int32 = -1

    // MARK: - Thread Safety

    /// Dispatch queue for all telemetry work (serial)
    private let queue = DispatchQueue(label: "com.robocar.telemetry", qos: .utility)

    // MARK: - Callbacks

    /// Called on main thread with connection status for UI display
    var onConnectionStatusChanged: ((Bool, String) -> Void)?

    private init() {}

    // MARK: - Lifecycle

    /// Start the telemetry service. Creates session file, connects WebSocket.
    func start(serverURL: String? = nil) {
        if let url = serverURL {
            self.serverURL = url
        }

        queue.async { [weak self] in
            guard let self = self, !self.isRunning else { return }
            self.isRunning = true
            self.sessionId = UUID().uuidString
            self.seq = 0
            self.totalFrames = 0
            self.totalRoutes = 0
            self.totalObstacleEvents = 0
            self.totalReplans = 0
            self.sessionStartDate = Date()
            self.recentLogs = []
            self.currentRouteId = nil

            // Check for previous crash
            self.checkForCrash()

            // Create session file
            self.createSessionFile()

            // Create breadcrumb file
            self.createBreadcrumbFile()

            // Connect WebSocket
            self.connectWebSocket()

            // Send session_start
            let event = SessionStartEvent(
                sessionId: self.sessionId,
                gridCellSize: 0.05,
                gridRadius: 500,
                arTrackingState: "normal",
                bleState: ESP32BLEManager.shared.connectionState.displayText,
                appVersion: Bundle.main.infoDictionary?["CFBundleShortVersionString"] as? String ?? "unknown",
                deviceModel: self.deviceModel(),
                osVersion: UIDevice.current.systemVersion
            )
            self.send(type: "session_start", payload: event)
        }
    }

    /// Stop the telemetry service. Writes session_end, closes connections.
    func stop(reason: String = "clean") {
        queue.async { [weak self] in
            guard let self = self, self.isRunning else { return }
            self.sendSessionEnd(reason: reason)
            self.isRunning = false
            self.disconnectWebSocket()
            self.closeSessionFile()
            self.closeBreadcrumbFile()
        }
    }

    /// Reset for a new session (e.g. after grid reset / relocalization)
    func reset() {
        queue.async { [weak self] in
            guard let self = self else { return }
            if self.isRunning {
                self.sendSessionEnd(reason: "user_stop")
                self.closeSessionFile()
            }
            self.sessionId = UUID().uuidString
            self.seq = 0
            self.totalFrames = 0
            self.totalRoutes = 0
            self.totalObstacleEvents = 0
            self.totalReplans = 0
            self.sessionStartDate = Date()
            self.recentLogs = []
            self.currentRouteId = nil
            self.createSessionFile()
            self.createBreadcrumbFile()

            let event = SessionStartEvent(
                sessionId: self.sessionId,
                gridCellSize: 0.05,
                gridRadius: 500,
                arTrackingState: "normal",
                bleState: ESP32BLEManager.shared.connectionState.displayText,
                appVersion: Bundle.main.infoDictionary?["CFBundleShortVersionString"] as? String ?? "unknown",
                deviceModel: self.deviceModel(),
                osVersion: UIDevice.current.systemVersion
            )
            self.send(type: "session_start", payload: event)
        }
    }

    // MARK: - Nav Frame Tick (called from LiDARViewController.updateFrame ~10 Hz)

    /// Sample all singletons and emit a nav_frame + optional mesh_snapshot.
    func tick(occupancyGrid: OccupancyGrid, cameraTransform: simd_float4x4) {
        // Capture everything we need on the calling thread (main)
        let pos = occupancyGrid.devicePosition
        let pose = Pose(from: pos, arTransform: cameraTransform)

        let navigator = PathNavigator.shared
        let detector = ObstacleDetector.shared
        let ble = ESP32BLEManager.shared

        let motorCmd = MotorSnapshot(fromBLEData: ble.lastMotorDataPublic)
        let motorRaw = lastSteering.motorCmdRaw
        let bleConnected = ble.connectionState == .connected

        // Capture depth obstacle snapshot on main thread for obstacle_map
        let depthDist = detector.depthObstacleDistance
        let depthWorldPos = detector.depthObstacleWorldPosition

        // Steering
        let steeringSnap = lastSteering
        let steering = SteeringState(
            mode: steeringSnap.mode,
            headingError: steeringSnap.headingError,
            headingErrorDeg: steeringSnap.headingError * 180 / .pi,
            speedFactor: steeringSnap.speedFactor,
            depthEvasionBias: steeringSnap.depthEvasionBias,
            spinPower: steeringSnap.spinPower,
            arcTurn: steeringSnap.arcTurn
        )

        // Waypoint
        let wpIndex = navigator.currentWaypointIndex
        let wpCount = navigator.currentPath.count
        let target = navigator.targetPoint
        let currentWP: Vec2?
        if wpIndex < navigator.currentPath.count {
            let wp = navigator.currentPath[wpIndex]
            currentWP = Vec2(x: wp.x, y: wp.y)
        } else {
            currentWP = nil
        }
        let distToTarget: Float?
        if let t = target {
            let dx = t.x - pos.x
            let dy = t.y - pos.y
            distToTarget = sqrtf(dx * dx + dy * dy)
        } else {
            distToTarget = nil
        }
        let distToWP: Float?
        if let wp = currentWP {
            let dx = wp.x - pos.x
            let dy = wp.y - pos.y
            distToWP = sqrtf(dx * dx + dy * dy)
        } else {
            distToWP = nil
        }

        let waypoint = WaypointState(
            index: wpIndex,
            count: wpCount,
            current: currentWP,
            lookahead: steeringSnap.lookaheadPoint,
            target: target.map { Vec2(x: $0.x, y: $0.y) },
            distToTarget: distToTarget,
            distToWaypoint: distToWP
        )

        // Obstacle
        let nearbyObs: [ObstacleRecord] = detector.nearbyObstacles.map { obs in
            ObstacleRecord(
                distance: obs.distance,
                classification: obs.classification.label,
                angleDeg: obs.angleDegrees,
                direction: obs.directionLabel,
                elevationDeg: obs.elevationDegrees,
                isDepthBased: obs.isDepthBased,
                worldPos: nil
            )
        }

        let blockedLocal: Vec2?
        if let bl = detector.blockedLocalDirection {
            blockedLocal = Vec2(x: bl.x, y: bl.y)
        } else {
            blockedLocal = nil
        }
        let blockedWorld: Vec2?
        if let bw = detector.blockedWorldDirection {
            blockedWorld = Vec2(x: bw.x, y: bw.y)
        } else {
            blockedWorld = nil
        }
        let depthPos: Vec3?
        if let dp = detector.depthObstacleWorldPosition {
            depthPos = Vec3(x: dp.x, y: dp.y, z: dp.z)
        } else {
            depthPos = nil
        }

        let obstacle = ObstacleSnapshot(
            detected: detector.obstacleDetected,
            nearestDist: detector.nearestObstacleDistance,
            nearestClassification: detector.nearestClassification.label.isEmpty ? nil : detector.nearestClassification.label,
            blockedLocalDir: blockedLocal,
            blockedWorldDir: blockedWorld,
            blockingTravel: detector.obstacleDetected, // simplified
            depthDist: detector.depthObstacleDistance,
            depthWorldPos: depthPos,
            nearbyCount: nearbyObs.count,
            nearby: nearbyObs
        )

        let pursuit = PurePursuitState(
            lookaheadDist: steeringSnap.lookaheadDist,
            lookaheadPoint: steeringSnap.lookaheadPoint,
            targetHeading: steeringSnap.targetHeading,
            stuckDetected: steeringSnap.stuckDetected,
            distSinceStuckCheck: steeringSnap.distSinceStuckCheck
        )

        let navState: String
        switch navigator.state {
        case .idle: navState = "idle"
        case .navigating: navState = "navigating"
        case .paused: navState = "paused"
        case .arrived: navState = "arrived"
        }

        let frame = NavFrame(
            pose: pose,
            motorCmd: motorCmd,
            motorCmdRaw: motorRaw,
            steering: steering,
            waypoint: waypoint,
            obstacle: obstacle,
            pursuit: pursuit,
            bleConnected: bleConnected,
            motorWriteInFlight: false
        )

        // Send on background queue
        queue.async { [weak self] in
            guard let self = self, self.isRunning else { return }
            self.totalFrames += 1
            self.send(type: "nav_frame", payload: frame)

            // Write crash breadcrumb
            let breadcrumb = CrashBreadcrumb(
                seq: self.seq,
                ts: Date().timeIntervalSince1970,
                sessionId: self.sessionId,
                pose: pose,
                motor: motorCmd,
                navState: navState,
                waypointIndex: wpIndex,
                headingError: steeringSnap.headingError,
                obstacleDetected: detector.obstacleDetected,
                nearestObstacleDist: detector.nearestObstacleDistance,
                steeringMode: steeringSnap.mode,
                recentLogs: Array(self.recentLogs.suffix(self.maxRecentLogs))
            )
            self.writeBreadcrumb(breadcrumb)

            // Grid-only mesh snapshot + obstacle map at lower rate
            self.framesSinceLastMesh += 1
            if self.framesSinceLastMesh >= self.meshSnapshotInterval {
                self.framesSinceLastMesh = 0
                self.emitMeshSnapshot(occupancyGrid: occupancyGrid, pose: pose)
                self.emitObstacleMap(occupancyGrid: occupancyGrid, pose: pose, depthDist: depthDist, depthWorldPos: depthWorldPos)
            }
        }
    }

    // MARK: - Event Logging API

    /// Log a route plan/replan. Called from LiDARViewController.planPath and replan logic.
    func logRoutePlanned(
        target: (x: Float, y: Float),
        origin: (x: Float, y: Float),
        waypoints: [(x: Float, y: Float)],
        algorithm: String,
        reason: String,
        planDurationMs: Float,
        occupancyGrid: OccupancyGrid,
        cameraTransform: simd_float4x4,
        includeGrid: Bool = false
    ) {
        let routeId = UUID().uuidString
        let pose = Pose(from: occupancyGrid.devicePosition, arTransform: cameraTransform)

        var pathLength: Float = 0
        for i in 1..<waypoints.count {
            let dx = waypoints[i].x - waypoints[i-1].x
            let dy = waypoints[i].y - waypoints[i-1].y
            pathLength += sqrtf(dx * dx + dy * dy)
        }

        let previousRouteId = currentRouteId

        let grid: GridSnapshot?
        if includeGrid {
            grid = buildGridSnapshot(occupancyGrid: occupancyGrid)
        } else {
            grid = nil
        }

        let event = RoutePlannedEvent(
            routeId: routeId,
            pose: pose,
            target: Vec2(x: target.x, y: target.y),
            origin: Vec2(x: origin.x, y: origin.y),
            waypoints: waypoints.map { Vec2(x: $0.x, y: $0.y) },
            waypointCount: waypoints.count,
            pathLengthMeters: pathLength,
            planDurationMs: planDurationMs,
            algorithm: algorithm,
            reason: reason,
            replacesRouteId: previousRouteId,
            gridAtPlanTime: grid
        )

        queue.async { [weak self] in
            guard let self = self else { return }
            self.currentRouteId = routeId
            self.totalRoutes += 1
            if reason.hasPrefix("replan") {
                self.totalReplans += 1
            }
            self.send(type: "route_planned", payload: event)
        }
    }

    /// Log a navigation state change. Called from PathNavigator.
    func logNavStateChange(
        from: NavigationState,
        to: NavigationState,
        reason: String,
        occupancyGrid: OccupancyGrid?,
        cameraTransform: simd_float4x4
    ) {
        let pos = occupancyGrid?.devicePosition ?? .zero
        let pose = Pose(from: pos, arTransform: cameraTransform)
        let motor = MotorSnapshot(fromBLEData: ESP32BLEManager.shared.lastMotorDataPublic)

        func stateString(_ s: NavigationState) -> String {
            switch s {
            case .idle: return "idle"
            case .navigating: return "navigating"
            case .paused: return "paused"
            case .arrived: return "arrived"
            }
        }

        let distToTarget: Float?
        if let target = PathNavigator.shared.targetPoint, let grid = occupancyGrid {
            let dx = target.x - grid.devicePosition.x
            let dy = target.y - grid.devicePosition.y
            distToTarget = sqrtf(dx * dx + dy * dy)
        } else {
            distToTarget = nil
        }

        let event = NavStateChangeEvent(
            from: stateString(from),
            to: stateString(to),
            reason: reason,
            pose: pose,
            routeId: currentRouteId,
            distToTarget: distToTarget,
            motor: motor
        )

        addRecentLog("[NavState] \(stateString(from)) → \(stateString(to)): \(reason)")

        if stateString(to) == "idle" {
            currentRouteId = nil
        }

        queue.async { [weak self] in
            self?.send(type: "nav_state_change", payload: event)
        }
    }

    /// Log an obstacle event. Called from ObstacleDetector when state changes.
    func logObstacleEvent(
        action: String,
        obstacles: [NearbyObstacle],
        motorBefore: MotorSnapshot,
        motorAfter: MotorSnapshot,
        triggerSource: String,
        replanAttempt: Int? = nil,
        occupancyGrid: OccupancyGrid?,
        cameraTransform: simd_float4x4
    ) {
        let pos = occupancyGrid?.devicePosition ?? .zero
        let pose = Pose(from: pos, arTransform: cameraTransform)

        let records: [ObstacleRecord] = obstacles.map { obs in
            ObstacleRecord(
                distance: obs.distance,
                classification: obs.classification.label,
                angleDeg: obs.angleDegrees,
                direction: obs.directionLabel,
                elevationDeg: obs.elevationDegrees,
                isDepthBased: obs.isDepthBased,
                worldPos: nil
            )
        }

        let event = ObstacleEventPayload(
            pose: pose,
            action: action,
            obstacles: records,
            motorBefore: motorBefore,
            motorAfter: motorAfter,
            triggerSource: triggerSource,
            replanAttempt: replanAttempt,
            routeId: currentRouteId
        )

        addRecentLog("[Obstacle] \(action) (\(obstacles.count) nearby)")

        queue.async { [weak self] in
            guard let self = self else { return }
            self.totalObstacleEvents += 1
            self.send(type: "obstacle_event", payload: event)
        }
    }

    /// Log an exploration event.
    func logExplorationEvent(
        status: String,
        frontierCount: Int = 0,
        nearestFrontierDist: Float? = nil,
        selectedTarget: (x: Float, y: Float)? = nil,
        totalTurns: Int = 0,
        totalDriveSegments: Int = 0,
        elapsedSec: Float = 0,
        message: String = "",
        occupancyGrid: OccupancyGrid?,
        cameraTransform: simd_float4x4
    ) {
        let pos = occupancyGrid?.devicePosition ?? .zero
        let pose = Pose(from: pos, arTransform: cameraTransform)

        let event = ExplorationEventPayload(
            status: status,
            pose: pose,
            frontierCount: frontierCount,
            nearestFrontierDist: nearestFrontierDist,
            selectedTarget: selectedTarget.map { Vec2(x: $0.x, y: $0.y) },
            totalTurns: totalTurns,
            totalDriveSegments: totalDriveSegments,
            elapsedSec: elapsedSec,
            message: message
        )

        addRecentLog("[Explore] \(status): \(message)")

        queue.async { [weak self] in
            self?.send(type: "exploration_event", payload: event)
        }
    }

    /// Log a calibration sample or result.
    func logCalibrationEvent(_ event: CalibrationEventPayload) {
        queue.async { [weak self] in
            self?.send(type: "calibration_event", payload: event)
        }
    }

    // MARK: - Mesh Snapshot

    private func emitMeshSnapshot(occupancyGrid: OccupancyGrid, pose: Pose) {
        let grid = buildGridSnapshot(occupancyGrid: occupancyGrid)

        // We send anchor data from the main ARSession in a separate path;
        // here we just send the grid snapshot without full mesh geometry
        // (mesh geometry is only needed occasionally or on demand).
        let event = MeshSnapshotEvent(
            pose: pose,
            anchors: [],  // populated via emitMeshAnchors when available
            grid: grid
        )
        send(type: "mesh_snapshot", payload: event)
    }

    /// Send a mesh snapshot with anchor geometry. Call from main thread with AR frame data.
    func emitMeshAnchors(_ anchors: [MeshAnchorSnapshot], occupancyGrid: OccupancyGrid, cameraTransform: simd_float4x4) {
        let pose = Pose(from: occupancyGrid.devicePosition, arTransform: cameraTransform)
        let grid = buildGridSnapshot(occupancyGrid: occupancyGrid)

        let event = MeshSnapshotEvent(pose: pose, anchors: anchors, grid: grid)
        queue.async { [weak self] in
            self?.send(type: "mesh_snapshot", payload: event)
        }
    }

    // MARK: - Obstacle Map

    /// Obstacle map query radius in meters — all occupied cells within this distance are sent
    private let obstacleMapRadius: Float = 3.0

    /// Emit obstacle positions from the occupancy grid so the web client can visualize
    /// which mesh regions are considered obstacles. Called at ~1 Hz alongside mesh snapshots.
    private func emitObstacleMap(
        occupancyGrid: OccupancyGrid,
        pose: Pose,
        depthDist: Float?,
        depthWorldPos: (x: Float, y: Float, z: Float)?
    ) {
        let pos = occupancyGrid.devicePosition
        let detailed = occupancyGrid.getOccupiedCellsDetailed(
            aroundX: pos.x, aroundY: pos.y, radius: obstacleMapRadius
        )

        // Filter out floor/ceiling — same as ObstacleDetector
        let obstacleCells: [ObstacleCell] = detailed.compactMap { cell in
            guard cell.classification != .floor && cell.classification != .ceiling else { return nil }
            return ObstacleCell(
                x: cell.worldX,
                y: cell.worldY,
                height: cell.height,
                classification: cell.classification.label,
                distance: cell.distance
            )
        }

        let depthPoint: DepthObstaclePoint?
        if let dist = depthDist, let wp = depthWorldPos {
            depthPoint = DepthObstaclePoint(x: wp.x, y: wp.y, z: wp.z, distance: dist)
        } else {
            depthPoint = nil
        }

        let event = ObstacleMapEvent(
            pose: pose,
            radius: obstacleMapRadius,
            cellCount: obstacleCells.count,
            cells: obstacleCells,
            depthObstacle: depthPoint
        )
        send(type: "obstacle_map", payload: event)
    }

    // MARK: - Grid Encoding

    private func buildGridSnapshot(occupancyGrid: OccupancyGrid) -> GridSnapshot {
        let pos = occupancyGrid.devicePosition
        let radiusMeters: Float = 10.0  // 10m radius around device
        let region = occupancyGrid.getRegion(centerX: pos.x, centerY: pos.y, radiusMeters: radiusMeters)

        let size = region.cells.count
        guard size > 0 else {
            return GridSnapshot(
                originX: pos.x, originY: pos.y, cellSize: occupancyGrid.cellSize,
                width: 0, height: 0, cellsRleB64: "", classificationsRleB64: "",
                occupiedCount: 0, freeCount: 0, floorHeight: 0
            )
        }

        // Flatten to row-major
        var cellValues: [UInt8] = []
        var classValues: [UInt8] = []
        cellValues.reserveCapacity(size * size)
        classValues.reserveCapacity(size * size)

        for y in 0..<size {
            for x in 0..<size {
                if x < region.cells.count && y < region.cells[x].count {
                    cellValues.append(region.cells[x][y].rawValue)
                    classValues.append(region.classifications[x][y].rawValue)
                } else {
                    cellValues.append(0)
                    classValues.append(0)
                }
            }
        }

        return GridSnapshot(
            originX: region.originX,
            originY: region.originY,
            cellSize: region.cellSize,
            width: size,
            height: size,
            cellsRleB64: rleEncodeBase64(cellValues),
            classificationsRleB64: rleEncodeBase64(classValues),
            occupiedCount: occupancyGrid.occupiedCount,
            freeCount: occupancyGrid.freeCount,
            floorHeight: region.minHeight
        )
    }

    // MARK: - Send

    private func send<T: Encodable>(type: String, payload: T) {
        seq += 1
        let message = TelemetryMessage(
            type: type,
            ts: Date().timeIntervalSince1970,
            seq: seq,
            sessionId: sessionId,
            payload: payload
        )

        guard let data = try? telemetryEncoder.encode(message) else { return }

        // Write to file
        writeToFile(data)

        // Send over WebSocket
        if isConnected, let ws = wsTask {
            let string = String(data: data, encoding: .utf8) ?? ""
            ws.send(.string(string)) { [weak self] error in
                if let error = error {
                    print("[Telemetry] WS send error: \(error.localizedDescription)")
                    self?.queue.async { self?.handleDisconnect() }
                }
            }
        }
    }

    // MARK: - WebSocket

    private func connectWebSocket() {
        guard let url = URL(string: "\(serverURL)/ws/ingest/\(sessionId)") else {
            print("[Telemetry] Invalid server URL: \(serverURL)")
            return
        }

        let config = URLSessionConfiguration.default
        config.waitsForConnectivity = false
        config.timeoutIntervalForRequest = 10
        config.timeoutIntervalForResource = 10
        session = URLSession(configuration: config)
        wsTask = session?.webSocketTask(with: url)
        wsTask?.resume()

        print("[Telemetry] Connecting to \(url)…")

        // Verify the connection is actually open before marking connected
        wsTask?.sendPing { [weak self] error in
            guard let self = self else { return }
            self.queue.async {
                if let error = error {
                    print("[Telemetry] Connection failed: \(error.localizedDescription)")
                    self.wsTask?.cancel(with: .goingAway, reason: nil)
                    self.wsTask = nil
                    self.session?.invalidateAndCancel()
                    self.session = nil
                    self.isConnected = false
                    self.notifyConnectionStatus(false, self.serverURL)
                    // Schedule reconnect
                    self.scheduleReconnect()
                } else {
                    print("[Telemetry] Connected to \(url)")
                    self.isConnected = true
                    self.notifyConnectionStatus(true, self.serverURL)
                    // Start listening for ACKs
                    self.listenForMessages()
                }
            }
        }
    }

    private func listenForMessages() {
        wsTask?.receive { [weak self] result in
            guard let self = self else { return }
            switch result {
            case .success(let message):
                switch message {
                case .string(let text):
                    // Parse ACK: {"ack": seq}
                    if let data = text.data(using: .utf8),
                       let json = try? JSONSerialization.jsonObject(with: data) as? [String: Any],
                       let _ = json["ack"] as? UInt64 {
                        // Could track upload cursor here for resume support
                    }
                    // Parse commands: {"cmd": "request_mesh_snapshot"}
                    if let data = text.data(using: .utf8),
                       let json = try? JSONSerialization.jsonObject(with: data) as? [String: Any],
                       let cmd = json["cmd"] as? String {
                        self.handleServerCommand(cmd)
                    }
                default:
                    break
                }
                // Continue listening
                self.listenForMessages()

            case .failure(let error):
                print("[Telemetry] WS receive error: \(error.localizedDescription)")
                self.queue.async { self.handleDisconnect() }
            }
        }
    }

    private func handleServerCommand(_ cmd: String) {
        switch cmd {
        case "request_mesh_snapshot":
            // Emit a full mesh snapshot on next opportunity
            framesSinceLastMesh = meshSnapshotInterval
        default:
            break
        }
    }

    private func handleDisconnect() {
        guard isConnected else { return }
        isConnected = false
        wsTask?.cancel(with: .goingAway, reason: nil)
        wsTask = nil
        session?.invalidateAndCancel()
        session = nil

        notifyConnectionStatus(false, serverURL)

        scheduleReconnect()
    }

    private func scheduleReconnect() {
        DispatchQueue.main.async { [weak self] in
            guard let self = self else { return }
            self.reconnectTimer?.invalidate()
            self.reconnectTimer = Timer.scheduledTimer(withTimeInterval: self.reconnectInterval, repeats: false) { [weak self] _ in
                self?.queue.async {
                    guard let self = self, self.isRunning, !self.isConnected else { return }
                    self.connectWebSocket()
                }
            }
        }
    }

    private func disconnectWebSocket() {
        reconnectTimer?.invalidate()
        reconnectTimer = nil
        isConnected = false
        wsTask?.cancel(with: .normalClosure, reason: nil)
        wsTask = nil
        session?.invalidateAndCancel()
        session = nil
    }

    // MARK: - File Persistence

    private var telemetryDirectory: URL {
        let docs = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask).first!
        return docs.appendingPathComponent("telemetry", isDirectory: true)
    }

    private func createSessionFile() {
        let dir = telemetryDirectory
        try? FileManager.default.createDirectory(at: dir, withIntermediateDirectories: true)

        let path = dir.appendingPathComponent("\(sessionId).jsonl")
        FileManager.default.createFile(atPath: path.path, contents: nil)
        fileHandle = try? FileHandle(forWritingTo: path)
        fileHandle?.seekToEndOfFile()
        filePath = path
        linesSinceFlush = 0
        lastFlushTime = Date()
    }

    private func writeToFile(_ data: Data) {
        guard let fh = fileHandle else { return }
        var line = data
        line.append(0x0A)  // newline
        fh.write(line)

        linesSinceFlush += 1
        if linesSinceFlush >= flushFileCount || Date().timeIntervalSince(lastFlushTime) >= flushFileInterval {
            fh.synchronizeFile()
            linesSinceFlush = 0
            lastFlushTime = Date()
        }
    }

    private func closeSessionFile() {
        fileHandle?.synchronizeFile()
        fileHandle?.closeFile()
        fileHandle = nil
    }

    // MARK: - Crash Breadcrumb

    private func createBreadcrumbFile() {
        let path = telemetryDirectory.appendingPathComponent("breadcrumb.json")
        breadcrumbPath = path

        // Create or truncate
        FileManager.default.createFile(atPath: path.path, contents: nil)
    }

    private func writeBreadcrumb(_ breadcrumb: CrashBreadcrumb) {
        guard let path = breadcrumbPath else { return }
        guard let data = try? telemetryEncoder.encode(breadcrumb) else { return }
        try? data.write(to: path, options: .atomic)
    }

    private func closeBreadcrumbFile() {
        // Remove breadcrumb on clean shutdown
        if let path = breadcrumbPath {
            try? FileManager.default.removeItem(at: path)
        }
    }

    /// Check if the previous session crashed (breadcrumb exists without session_end)
    private func checkForCrash() {
        let path = telemetryDirectory.appendingPathComponent("breadcrumb.json")
        guard FileManager.default.fileExists(atPath: path.path),
              let data = try? Data(contentsOf: path),
              let breadcrumb = try? telemetryDecoder.decode(CrashBreadcrumb.self, from: data) else {
            return
        }

        // Previous session crashed — send the breadcrumb as an event
        print("[Telemetry] Previous session \(breadcrumb.sessionId) crashed at seq \(breadcrumb.seq)")
        send(type: "crash_breadcrumb", payload: breadcrumb)

        // Clean up
        try? FileManager.default.removeItem(at: path)
    }

    // MARK: - Session End

    private func sendSessionEnd(reason: String) {
        guard let occupancyGrid = ObstacleDetector.shared.occupancyGrid else { return }
        let grid = buildGridSnapshot(occupancyGrid: occupancyGrid)

        let event = SessionEndEvent(
            sessionId: sessionId,
            durationSec: Float(Date().timeIntervalSince(sessionStartDate)),
            totalFrames: totalFrames,
            totalRoutes: totalRoutes,
            totalObstacleEvents: totalObstacleEvents,
            totalReplans: totalReplans,
            finalGrid: grid,
            endReason: reason
        )
        send(type: "session_end", payload: event)
    }

    // MARK: - Helpers

    private func addRecentLog(_ message: String) {
        queue.async { [weak self] in
            guard let self = self else { return }
            self.recentLogs.append(message)
            if self.recentLogs.count > self.maxRecentLogs * 2 {
                self.recentLogs = Array(self.recentLogs.suffix(self.maxRecentLogs))
            }
        }
    }

    private func notifyConnectionStatus(_ connected: Bool, _ url: String) {
        DispatchQueue.main.async { [weak self] in
            self?.onConnectionStatusChanged?(connected, url)
        }
    }

    private func deviceModel() -> String {
        var systemInfo = utsname()
        uname(&systemInfo)
        return withUnsafePointer(to: &systemInfo.machine) {
            $0.withMemoryRebound(to: CChar.self, capacity: 1) {
                String(validatingUTF8: $0) ?? "unknown"
            }
        }
    }
}
