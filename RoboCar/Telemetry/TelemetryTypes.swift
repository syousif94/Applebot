//
//  TelemetryTypes.swift
//  RoboCar
//
//  Wire format types for streaming navigation telemetry to the Rust server.
//  Every type is Codable and serialized as a single JSON line.
//  Field names use camelCase in Swift but encode as snake_case on the wire.
//

import Foundation
import simd

// MARK: - Snake Case Coding

/// Shared encoder configured for the wire format.
let telemetryEncoder: JSONEncoder = {
    let e = JSONEncoder()
    e.keyEncodingStrategy = .convertToSnakeCase
    e.dateEncodingStrategy = .millisecondsSince1970
    e.outputFormatting = []  // compact, single line
    return e
}()

/// Shared decoder for server messages.
let telemetryDecoder: JSONDecoder = {
    let d = JSONDecoder()
    d.keyDecodingStrategy = .convertFromSnakeCase
    return d
}()

// MARK: - Envelope

/// Every message sent over the WebSocket. Serialized as one JSON line.
/// The `payload` is encoded inline (flattened) alongside the envelope fields.
struct TelemetryMessage: Encodable {
    let type: String
    let ts: Double       // unix epoch seconds, ms precision
    let seq: UInt64
    let sessionId: String

    private let _payload: AnyEncodable

    init<T: Encodable>(type: String, ts: Double, seq: UInt64, sessionId: String, payload: T) {
        self.type = type
        self.ts = ts
        self.seq = seq
        self.sessionId = sessionId
        self._payload = AnyEncodable(payload)
    }

    func encode(to encoder: Encoder) throws {
        var container = encoder.container(keyedBy: DynamicCodingKey.self)
        try container.encode(type, forKey: .init("type"))
        try container.encode(ts, forKey: .init("ts"))
        try container.encode(seq, forKey: .init("seq"))
        try container.encode(sessionId, forKey: .init("session_id"))
        // Flatten payload fields into the same object
        try _payload.encode(to: encoder)
    }
}

// MARK: - Shared Primitives

struct Vec2: Codable {
    let x: Float
    let y: Float
}

struct Vec3: Codable {
    let x: Float
    let y: Float
    let z: Float
}

/// Full 6-DOF device pose. Included in every event.
struct Pose: Codable {
    let x: Float
    let y: Float
    let z: Float
    let heading: Float
    let headingDeg: Float
    /// Raw ARKit 4×4 column-major transform (16 floats)
    let transform: [Float]

    init(from devicePosition: DevicePosition, arTransform: simd_float4x4) {
        self.x = devicePosition.x
        self.y = devicePosition.y
        self.z = devicePosition.z
        self.heading = devicePosition.heading
        self.headingDeg = devicePosition.heading * 180 / .pi
        // Column-major: col0, col1, col2, col3
        let m = arTransform
        self.transform = [
            m.columns.0.x, m.columns.0.y, m.columns.0.z, m.columns.0.w,
            m.columns.1.x, m.columns.1.y, m.columns.1.z, m.columns.1.w,
            m.columns.2.x, m.columns.2.y, m.columns.2.z, m.columns.2.w,
            m.columns.3.x, m.columns.3.y, m.columns.3.z, m.columns.3.w,
        ]
    }
}

struct MotorSnapshot: Codable {
    let a: Int8
    let b: Int8
    let c: Int8
    let d: Int8

    static let zero = MotorSnapshot(a: 0, b: 0, c: 0, d: 0)

    /// Decode from ESP32BLEManager.lastMotorDataPublic (4 bytes, already negated on write)
    init(fromBLEData data: Data?) {
        guard let data = data, data.count >= 4 else {
            a = 0; b = 0; c = 0; d = 0
            return
        }
        // Values stored in BLE data are post-negate (positive = physical forward)
        a = Int8(bitPattern: data[0])
        b = Int8(bitPattern: data[1])
        c = Int8(bitPattern: data[2])
        d = Int8(bitPattern: data[3])
    }

    init(a: Int8, b: Int8, c: Int8, d: Int8) {
        self.a = a; self.b = b; self.c = c; self.d = d
    }
}

// MARK: - 1. NavFrame (type: "nav_frame", ~10 Hz)

struct NavFrame: Codable {
    let pose: Pose
    let motorCmd: MotorSnapshot
    let motorCmdRaw: MotorSnapshot
    let steering: SteeringState
    let waypoint: WaypointState
    let obstacle: ObstacleSnapshot
    let pursuit: PurePursuitState
    let bleConnected: Bool
    let motorWriteInFlight: Bool
}

struct SteeringState: Codable {
    /// "point_turn" | "arc" | "reverse_phase0" | "reverse_phase1" | "stopped"
    let mode: String
    /// Heading error to target waypoint (radians, signed: >0 = need right)
    let headingError: Float
    let headingErrorDeg: Float
    /// Effective speed multiplier (approach slowdown × depth evasion)
    let speedFactor: Float
    /// Radians added to heading by depth-based evasion
    let depthEvasionBias: Float
    /// Point-turn spin power (0 if not in point-turn mode)
    let spinPower: Float
    /// Proportional arc steering value (-1..1)
    let arcTurn: Float
}

struct WaypointState: Codable {
    let index: Int
    let count: Int
    let current: Vec2?
    let lookahead: Vec2?
    let target: Vec2?
    let distToTarget: Float?
    let distToWaypoint: Float?
}

struct ObstacleSnapshot: Codable {
    let detected: Bool
    let nearestDist: Float?
    let nearestClassification: String?
    let blockedLocalDir: Vec2?
    let blockedWorldDir: Vec2?
    let blockingTravel: Bool
    let depthDist: Float?
    let depthWorldPos: Vec3?
    let nearbyCount: Int
    let nearby: [ObstacleRecord]
}

struct ObstacleRecord: Codable {
    let distance: Float
    let classification: String
    let angleDeg: Int
    let direction: String
    let elevationDeg: Int
    let isDepthBased: Bool
    let worldPos: Vec2?
}

struct PurePursuitState: Codable {
    let lookaheadDist: Float
    let lookaheadPoint: Vec2
    let targetHeading: Float
    let stuckDetected: Bool
    let distSinceStuckCheck: Float?
}

// MARK: - 2. ObstacleMap (type: "obstacle_map", ~1 Hz)

struct ObstacleMapEvent: Codable {
    let pose: Pose
    let radius: Float              // query radius in meters
    let cellCount: Int             // total obstacle cells returned
    let cells: [ObstacleCell]      // obstacle cell positions within radius
    let depthObstacle: DepthObstaclePoint?  // current LiDAR depth-map obstacle, if any
}

struct ObstacleCell: Codable {
    let x: Float                   // world X (meters)
    let y: Float                   // world Y (meters, ARKit Z)
    let height: Float              // max height above floor (meters)
    let classification: String     // mesh classification label
    let distance: Float            // distance from device (meters)
}

struct DepthObstaclePoint: Codable {
    let x: Float                   // world X
    let y: Float                   // world Y (ARKit Z)
    let z: Float                   // height (ARKit Y)
    let distance: Float            // distance from device (meters)
}

// MARK: - 3. MeshSnapshot (type: "mesh_snapshot", ~0.5-1 Hz)

struct MeshSnapshotEvent: Codable {
    let pose: Pose
    let anchors: [MeshAnchorSnapshot]
    let grid: GridSnapshot
}

struct MeshAnchorSnapshot: Codable {
    let anchorId: String
    let transform: [Float]       // 16 floats, column-major
    let verticesB64: String      // base64(f32[]) — [x,y,z, x,y,z, ...]
    let vertexCount: Int
    let indicesB64: String       // base64(u32[]) — triangle indices
    let triangleCount: Int
    let classificationsB64: String // base64(u8[]) — one per face
    let generation: Int
}

struct GridSnapshot: Codable {
    let originX: Float
    let originY: Float
    let cellSize: Float
    let width: Int
    let height: Int
    let cellsRleB64: String             // RLE-encoded cell states, base64
    let classificationsRleB64: String   // RLE-encoded classifications, base64
    let occupiedCount: Int
    let freeCount: Int
    let floorHeight: Float
}

// MARK: - 4. RoutePlanned (type: "route_planned")

struct RoutePlannedEvent: Codable {
    let routeId: String
    let pose: Pose
    let target: Vec2
    let origin: Vec2
    let waypoints: [Vec2]
    let waypointCount: Int
    let pathLengthMeters: Float
    let planDurationMs: Float
    /// "astar" | "greedy"
    let algorithm: String
    /// "user_tap" | "replan_periodic" | "replan_obstacle" | "replan_stuck" | "replan_path_exhausted" | "exploration" | "voice" | "server"
    let reason: String
    let replacesRouteId: String?
    let gridAtPlanTime: GridSnapshot?
}

// MARK: - 4b. RoutePreviewSegment (type: "route_preview_segment")

/// Emitted for each leg of a multi-waypoint route as its path is computed.
struct RoutePreviewSegmentEvent: Codable {
    let pose: Pose
    /// Shared across all segments of this preview computation
    let previewId: String
    /// 0-based index of this segment in the multi-waypoint route
    let segmentIndex: Int
    /// Total number of segments being planned
    let segmentCount: Int
    /// Origin of this segment
    let origin: Vec2
    /// Destination of this segment (the user-placed waypoint)
    let target: Vec2
    /// The computed A*/greedy path for this segment
    let waypoints: [Vec2]
    let waypointCount: Int
    let pathLengthMeters: Float
    /// "astar" | "greedy" | "greedy_shorter" | "direct_fallback"
    let algorithm: String
    /// Total number of user-placed waypoints in the route
    let routeWaypointCount: Int
    /// All user-placed waypoint destinations (for context)
    let routeWaypoints: [Vec2]
}

// MARK: - 5. NavStateChange (type: "nav_state_change")

struct NavStateChangeEvent: Codable {
    /// "idle" | "navigating" | "paused" | "arrived"
    let from: String
    let to: String
    /// "started" | "arrived" | "user_stop" | "obstacle_blocking" | "obstacle_cleared" |
    /// "replan_resume" | "stuck" | "reverse_start" | "reverse_complete" | "path_exhausted" | "cancelled"
    let reason: String
    let pose: Pose
    let routeId: String?
    let distToTarget: Float?
    let motor: MotorSnapshot
}

// MARK: - 6. ObstacleEvent (type: "obstacle_event")

struct ObstacleEventPayload: Codable {
    let pose: Pose
    /// "detected" | "cleared" | "replan_requested" | "reverse_initiated" | "evasion_steering" | "motor_filtered"
    let action: String
    let obstacles: [ObstacleRecord]
    let motorBefore: MotorSnapshot
    let motorAfter: MotorSnapshot
    /// "mesh_grid" | "depth_map" | "both"
    let triggerSource: String
    let replanAttempt: Int?
    let routeId: String?
}

// MARK: - 7. ExplorationEvent (type: "exploration_event")

struct ExplorationEventPayload: Codable {
    /// "started" | "scanning" | "target_selected" | "turning" | "driving" | "stuck" | "completed" | "stopped" | "time_limit"
    let status: String
    let pose: Pose
    let frontierCount: Int
    let nearestFrontierDist: Float?
    let selectedTarget: Vec2?
    let totalTurns: Int
    let totalDriveSegments: Int
    let elapsedSec: Float
    let message: String
}

// MARK: - 8. CalibrationEvent (type: "calibration_event")

struct CalibrationEventPayload: Codable {
    /// "sample" | "result"
    let phase: String
    // Sample fields
    let testIndex: Int?
    let totalTests: Int?
    let leftPower: Int8?
    let rightPower: Int8?
    let linearSpeed: Float?
    let angularVelocity: Float?
    let duration: Float?
    let distance: Float?
    let angleTurned: Float?
    // Result fields
    let speedPerPowerUnit: Float?
    let maxTurnRate: Float?
    let leftRightBias: Float?
}

// MARK: - 9. SessionStart (type: "session_start")

struct SessionStartEvent: Codable {
    let sessionId: String
    let gridCellSize: Float
    let gridRadius: Int
    let arTrackingState: String
    let bleState: String
    let appVersion: String
    let deviceModel: String
    let osVersion: String
}

// MARK: - 10. SessionEnd (type: "session_end")

struct SessionEndEvent: Codable {
    let sessionId: String
    let durationSec: Float
    let totalFrames: UInt64
    let totalRoutes: Int
    let totalObstacleEvents: Int
    let totalReplans: Int
    let finalGrid: GridSnapshot
    /// "clean" | "crash" | "background" | "user_stop"
    let endReason: String
}

// MARK: - 11. CrashBreadcrumb (type: "crash_breadcrumb")

struct CrashBreadcrumb: Codable {
    let seq: UInt64
    let ts: Double
    let sessionId: String
    let pose: Pose
    let motor: MotorSnapshot
    let navState: String
    let waypointIndex: Int
    let headingError: Float
    let obstacleDetected: Bool
    let nearestObstacleDist: Float?
    let steeringMode: String
    let recentLogs: [String]
}

// MARK: - 12. NavCommandAck (type: "nav_command_ack")

struct NavCommandAckEvent: Codable {
    /// The command that was received: "set_nav_target" | "start_navigation" | "stop_navigation" | "add_route_point" | "run_route" | "pause_route" | "clear_route"
    let cmd: String
    let success: Bool
    let message: String
    /// Echo back the target point (present for set_nav_target)
    let target: Vec2?
    /// Number of waypoints in the planned path (present for set_nav_target on success)
    let waypointCount: Int?
}

// MARK: - 13. PersonTracking (type: "person_tracking", ~2 Hz during scanning/tracking)

/// Emitted periodically while the PersonTracker is scanning or tracking.
/// Contains all currently identified people, who is gesturing, and who is active.
struct PersonTrackingEvent: Codable {
    let pose: Pose
    /// "idle" | "scanning" | "tracking" | "reacquiring" | "lost"
    let trackingState: String
    /// Total people currently visible
    let peopleCount: Int
    /// Per-person snapshots
    let people: [PersonSnapshot]
    /// The UUID (short prefix) of the person being actively followed, if any
    let activePersonId: String?
    /// World position of the actively tracked person
    let activePersonPos: Vec2?
    /// Optional event that triggered this emission
    /// "scan_update" | "gesture_detected" | "person_activated" | "tracking_update" | "person_lost" | "state_change"
    let event: String
    /// Human-readable message
    let message: String
}

/// Snapshot of a single detected person, included in PersonTrackingEvent.
struct PersonSnapshot: Codable {
    /// Stable session-level ID (UUID, first 8 chars)
    let id: String
    /// World position (ARKit X, ARKit Z) — matches occupancy grid convention
    let worldPos: Vec2?
    /// Bounding box in normalized Vision coordinates (origin bottom-left)
    let bbox: BBoxRect
    /// Whether this person is currently showing the activation gesture
    let isGesturing: Bool
    /// How many consecutive detection cycles they have been gesturing
    let gestureStreak: Int
    /// Whether this is the actively followed person
    let isActive: Bool
    /// Seconds since this person was last detected
    let lastSeenAgo: Float
}

struct BBoxRect: Codable {
    let x: Float
    let y: Float
    let width: Float
    let height: Float
}

// MARK: - Encoding Helpers

/// Type-erased Encodable wrapper for flattening payloads into the envelope.
private struct AnyEncodable: Encodable {
    private let _encode: (Encoder) throws -> Void

    init<T: Encodable>(_ wrapped: T) {
        _encode = { encoder in
            try wrapped.encode(to: encoder)
        }
    }

    func encode(to encoder: Encoder) throws {
        try _encode(encoder)
    }
}

/// Dynamic coding key for flattening.
private struct DynamicCodingKey: CodingKey {
    var stringValue: String
    var intValue: Int?

    init(_ string: String) {
        self.stringValue = string
        self.intValue = nil
    }

    init?(stringValue: String) {
        self.stringValue = stringValue
        self.intValue = nil
    }

    init?(intValue: Int) {
        self.stringValue = "\(intValue)"
        self.intValue = intValue
    }
}

// MARK: - RLE Grid Encoding

/// Run-length encode an array of UInt8 values.
/// Output format: sequence of (value: UInt8, count: UInt16LE) = 3 bytes each.
/// Returns base64-encoded string.
func rleEncodeBase64(_ values: [UInt8]) -> String {
    guard !values.isEmpty else { return "" }

    var data = Data()
    data.reserveCapacity(values.count)  // worst case ~same size, typically much smaller

    var currentValue = values[0]
    var count: UInt16 = 1

    for i in 1..<values.count {
        if values[i] == currentValue && count < UInt16.max {
            count += 1
        } else {
            data.append(currentValue)
            var le = count.littleEndian
            data.append(contentsOf: withUnsafeBytes(of: &le) { Array($0) })
            currentValue = values[i]
            count = 1
        }
    }
    // Flush last run
    data.append(currentValue)
    var le = count.littleEndian
    data.append(contentsOf: withUnsafeBytes(of: &le) { Array($0) })

    return data.base64EncodedString()
}

// MARK: - Base64 Helpers for Mesh Data

/// Encode a [Float] array as base64.
func floatArrayToBase64(_ values: [Float]) -> String {
    let data = values.withUnsafeBufferPointer { buffer in
        Data(bytes: buffer.baseAddress!, count: buffer.count * MemoryLayout<Float>.size)
    }
    return data.base64EncodedString()
}

/// Encode a [UInt32] array as base64.
func uint32ArrayToBase64(_ values: [UInt32]) -> String {
    let data = values.withUnsafeBufferPointer { buffer in
        Data(bytes: buffer.baseAddress!, count: buffer.count * MemoryLayout<UInt32>.size)
    }
    return data.base64EncodedString()
}

/// Encode a [UInt8] array as base64.
func uint8ArrayToBase64(_ values: [UInt8]) -> String {
    return Data(values).base64EncodedString()
}
