//
//  RemoteControlProtocol.swift
//  RoboCar
//

import Foundation

enum RemoteControlProtocol {
    static let serviceType = "_robocar-remote._tcp"
    static let defaultPort: UInt16 = 8787
}

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

    init(devicePosition: DevicePosition) {
        x = devicePosition.x
        y = devicePosition.y
        z = devicePosition.z
        heading = devicePosition.heading
        headingDeg = devicePosition.heading * 180 / .pi
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

struct RemoteServoState: Codable {
    let id: UInt8
    let error: UInt8
    let position: UInt16
    let load: UInt16
    let voltage: UInt8
    let temperature: UInt8

    init(_ state: ServoState) {
        id = state.id
        error = state.error
        position = state.position
        load = state.load
        voltage = state.voltage
        temperature = state.temperature
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
    var id: UInt8?
    var from: UInt8?
    var to: UInt8?
    var position: UInt16?
    var speed: UInt16?
    var wheelSpeed: Int16?
    var acceleration: UInt8?
    var enabled: Bool?
    var signalType: String?
    var sdp: String?
    var candidate: String?
    var sdpMid: String?
    var sdpMLineIndex: Int32?

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
