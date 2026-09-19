//
//  RemoteControlProtocol.swift
//  RoboCar
//

import Foundation

struct RemotePoint: Codable {
    let x: Float
    let y: Float
}

struct RemotePose: Codable {
    let x: Float
    let y: Float
    let z: Float
    let heading: Float
    let headingDeg: Float
    /// True compass bearing in degrees (0 = North, CW positive). Nil when not yet available.
    let compassBearingDeg: Double?

    init(devicePosition: DevicePosition, compassBearingDeg: Double? = nil) {
        x = devicePosition.x
        y = devicePosition.y
        z = devicePosition.z
        heading = devicePosition.heading
        headingDeg = devicePosition.heading * 180 / .pi
        self.compassBearingDeg = compassBearingDeg
    }
}

struct RemoteCameraFrame: Codable {
    let width: Int
    let height: Int
    let jpegBase64: String
}

struct RemoteGridCell: Codable {
    let x: Float
    let y: Float
    let state: UInt8
    let classification: UInt8
    let height: Float
}

struct RemoteGridUpdate: Codable {
    let radius: Float
    let cellSize: Float
    let cells: [RemoteGridCell]
}

/// A detected person sent to the remote controller for drawing tappable
/// outlines over the video stream. Bounding box is in Vision-normalised
/// coordinates (origin bottom-left, values 0-1), matching the host overlay.
struct RemotePersonBox: Codable {
    let id: String
    let x: Float
    let y: Float
    let width: Float
    let height: Float
    let isActive: Bool
    let label: String
    let name: String?
}

struct RemoteServoState: Codable {
    let id: UInt8
    let error: UInt8
    let position: UInt16
    let load: UInt16
    let voltage: UInt8
    let temperature: UInt8
    let torqueEnabled: Bool?

    init(_ state: ServoState) {
        id = state.id
        error = state.error
        position = state.position
        load = state.load
        voltage = state.voltage
        temperature = state.temperature
        torqueEnabled = state.torqueEnabled
    }

    var asServoState: ServoState {
        ServoState(id: id, error: error, position: position, load: load, voltage: voltage, temperature: temperature, torqueEnabled: torqueEnabled)
    }
}

/// A single live servo position sample (host → controller).
struct RemoteServoPosition: Codable {
    let id: UInt8
    let position: UInt16
}

/// Multi-turn axis status mirror of `ServoAxisStatus` (host → controller).
struct RemoteServoAxisStatus: Codable {
    let id: UInt8
    let isTracked: Bool
    let hasMin: Bool
    let hasMax: Bool
    let hasZero: Bool
    let isMoving: Bool
    let isError: Bool
    let cumulativeTicks: Int32
    let angleDegrees: Double?
    let percent: Double?
    let totalDegrees: Double

    init(_ status: ServoAxisStatus) {
        id = status.id
        isTracked = status.isTracked
        hasMin = status.hasMin
        hasMax = status.hasMax
        hasZero = status.hasZero
        isMoving = status.isMoving
        isError = status.isError
        cumulativeTicks = status.cumulativeTicks
        angleDegrees = status.angleDegrees
        percent = status.percent
        totalDegrees = status.totalDegrees
    }

    var asServoAxisStatus: ServoAxisStatus {
        ServoAxisStatus(
            id: id,
            isTracked: isTracked,
            hasMin: hasMin,
            hasMax: hasMax,
            hasZero: hasZero,
            isMoving: isMoving,
            isError: isError,
            cumulativeTicks: cumulativeTicks,
            angleDegrees: angleDegrees,
            percent: percent,
            totalDegrees: totalDegrees
        )
    }
}

struct RemoteMessage: Codable {
    var type: String
    var seq: UInt64?
    var ts: Double?
    var x: Float?
    var y: Float?
    var pose: RemotePose?
    var camera: RemoteCameraFrame?
    var grid: RemoteGridUpdate?
    var routeWaypoints: [RemotePoint]?
    var plannedPath: [RemotePoint]?
    var routePreviewPaths: [[RemotePoint]]?
    var activeRouteWaypointIndex: Int?
    var bleConnected: Bool?
    var navState: String?
    var message: String?
    var servoIDs: [UInt8]?
    var servoState: RemoteServoState?
    /// Live servo positions streamed from the robot (host → controller).
    var servoPositions: [RemoteServoPosition]?
    /// Multi-turn axis status pushed from the robot (host → controller).
    var servoAxisStatus: RemoteServoAxisStatus?
    var id: UInt8?
    var from: UInt8?
    var to: UInt8?
    var position: UInt16?
    var speed: UInt16?
    var wheelSpeed: Int16?
    var acceleration: UInt8?
    var enabled: Bool?
    /// Jog/goto angle in degrees (controller → host).
    var degrees: Double?
    /// Jog direction: -1 counterclockwise, 1 clockwise (controller → host).
    var direction: Int8?
    /// Travel percent 0-100 (controller → host).
    var percent: Double?
    /// Travel mark: 0 = min, 1 = max (controller → host).
    var mark: UInt8?
    var signalType: String?
    var sdp: String?
    var candidate: String?
    var sdpMid: String?
    var sdpMLineIndex: Int32?
    var nlCommand: String?
    var isLocalConnection: Bool?
    var meshAnchors: [MeshAnchorSnapshot]?

    /// Detected people broadcast to the controller (host → controller).
    var personBoxes: [RemotePersonBox]?
    /// Stable UUID string of a person targeted by a controller command.
    var personID: String?
    /// Name supplied by the controller for naming a person, or the name to
    /// follow/delete by name.
    var personName: String?

    init(type: String) {
        self.type = type
        self.ts = Date().timeIntervalSince1970
    }
}

extension RemotePoint {
    init(_ tuple: (x: Float, y: Float)) {
        x = tuple.x
        y = tuple.y
    }
}
