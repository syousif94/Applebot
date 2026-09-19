//
//  RemoteControlHostService.swift
//  RoboCar
//

import Foundation
import UIKit

final class RemoteControlHostService {
    static let shared = RemoteControlHostService()

    var onStatusChanged: ((String) -> Void)?


    private let queue = DispatchQueue.main
    private let encoder = JSONEncoder()
    private let decoder = JSONDecoder()
    private let transport = RemoteControlIrohSession.shared
    private let video = RemoteVideoCodec()
    private var latestStatusMessage: RemoteMessage?
    private var latestMapStateMessage: RemoteMessage?
    private var latestGridUpdateMessage: RemoteMessage?
    private var latestMeshMessage: RemoteMessage?
    private var latestServoIDsMessage: RemoteMessage?
    private var latestServoStateMessage: RemoteMessage?
    private var latestServoPositionsMessage: RemoteMessage?
    private var latestServoAxisStatusMessages: [UInt8: RemoteMessage] = [:]
    private var seq: UInt64 = 0
    private var staleDriveTimer: DispatchSourceTimer?
    private var lastDriveCommandDate: Date?
    private let driveTimeout: TimeInterval = 0.75

    private init() {}

    var isRunning: Bool { transport.role == .robot && transport.isRunning }
    var clientCount: Int { transport.role == .robot && transport.isConnected ? 1 : 0 }
    var isLocalConnection: Bool { false }

    func start() {
        guard !isRunning else { return }
        transport.start(role: .robot)
        transport.onMessage = { [weak self] data in
            guard let self, let message = try? self.decoder.decode(RemoteMessage.self, from: data) else { return }
            if message.type == "restartVideo" { self.video.requestKeyframe(); return }
            if message.type == "drive" { self.lastDriveCommandDate = Date() }
            if message.type == "stopDrive" { self.lastDriveCommandDate = nil }
            let sessionID = self.transport.sessionID
            RemoteRobotCommandDispatcher.dispatch(message) { [weak self] in
                self?.transport.sessionID == sessionID && self?.transport.isConnected == true
            }
        }
        transport.onConnected = { [weak self] in self?.replayLatestState() }
        transport.onDisconnected = { [weak self] in
            self?.lastDriveCommandDate = nil
            self?.video.reset()
            NLNavigator.shared.stop()
            NotificationCenter.default.post(name: .serverStopNavigation, object: nil)
            NotificationCenter.default.post(name: .stopFollowing, object: nil)
            ESP32BLEManager.shared.stopAll()
        }
        transport.onNeedsKeyframe = { [weak self] in self?.video.requestKeyframe() }
        video.onEncoded = { [weak self] data, keyframe in self?.transport.sendVideo(data, keyframe: keyframe) }
        startDriveWatchdog()
    }

    func stop() {
        guard transport.role == .robot else { return }
        transport.stop()
        video.reset()
        staleDriveTimer?.cancel()
        staleDriveTimer = nil
        lastDriveCommandDate = nil
        ESP32BLEManager.shared.stopAll()
    }

    func broadcastStatus() {
        var message = RemoteMessage(type: "status")
        message.bleConnected = ESP32BLEManager.shared.connectionState == .connected
        message.message = "\(clientCount) remote client(s)"
        message.isLocalConnection = isLocalConnection
        latestStatusMessage = message
        broadcast(message)
    }

    func broadcastMapState(occupancyGrid: OccupancyGrid, routeWaypoints: [(x: Float, y: Float)], plannedPath: [(x: Float, y: Float)], routePreviewPaths: [[(x: Float, y: Float)]], activeRouteWaypointIndex: Int, navState: String, compassBearingDeg: Double? = nil) {
        var message = RemoteMessage(type: "mapState")
        message.pose = RemotePose(devicePosition: occupancyGrid.devicePosition, compassBearingDeg: compassBearingDeg)
        message.routeWaypoints = routeWaypoints.map(RemotePoint.init)
        message.plannedPath = plannedPath.map(RemotePoint.init)
        message.routePreviewPaths = routePreviewPaths.map { $0.map(RemotePoint.init) }
        message.activeRouteWaypointIndex = activeRouteWaypointIndex
        message.navState = navState
        message.bleConnected = ESP32BLEManager.shared.connectionState == .connected
        latestMapStateMessage = message
        broadcast(message)
    }

    func broadcastGridUpdate(occupancyGrid: OccupancyGrid, radiusMeters: Float = 6.0) {
        let pos = occupancyGrid.devicePosition
        let region = occupancyGrid.getRegion(centerX: pos.x, centerY: pos.y, radiusMeters: radiusMeters)
        var cells: [RemoteGridCell] = []
        cells.reserveCapacity(1200)

        for (xi, column) in region.cells.enumerated() {
            let worldX = region.originX + Float(xi) * region.cellSize
            for (yi, state) in column.enumerated() where state != .unknown {
                let worldY = region.originY + Float(yi) * region.cellSize
                cells.append(RemoteGridCell(
                    x: worldX,
                    y: worldY,
                    state: state.rawValue,
                    classification: region.classifications[xi][yi].rawValue,
                    height: region.heights[xi][yi]
                ))
            }
        }

        var message = RemoteMessage(type: "gridUpdate")
        message.grid = RemoteGridUpdate(radius: radiusMeters, cellSize: region.cellSize, cells: cells)
        latestGridUpdateMessage = message
        broadcast(message)
    }

    func broadcastCameraFrame(image: UIImage) {
        guard clientCount > 0 else { return }
        video.encode(image)
    }

    func broadcastServoIDs(_ ids: [UInt8]) {
        var message = RemoteMessage(type: "servoList")
        message.servoIDs = ids
        latestServoIDsMessage = message
        broadcast(message)
    }

    func broadcastServoState(_ state: ServoState) {
        var message = RemoteMessage(type: "servoState")
        message.servoState = RemoteServoState(state)
        latestServoStateMessage = message
        broadcast(message)
    }

    func broadcastServoPositions(_ positions: [UInt8: UInt16]) {
        var message = RemoteMessage(type: "servoPositions")
        message.servoPositions = positions
            .sorted { $0.key < $1.key }
            .map { RemoteServoPosition(id: $0.key, position: $0.value) }
        latestServoPositionsMessage = message
        broadcast(message)
    }

    func broadcastServoAxisStatus(_ status: ServoAxisStatus) {
        var message = RemoteMessage(type: "servoAxisStatus")
        message.servoAxisStatus = RemoteServoAxisStatus(status)
        latestServoAxisStatusMessages[status.id] = message
        broadcast(message)
    }

    func broadcastMeshAnchors(_ snapshots: [MeshAnchorSnapshot]) {
        guard !snapshots.isEmpty else { return }
        var message = RemoteMessage(type: "meshAnchors")
        message.meshAnchors = snapshots
        latestMeshMessage = message
        broadcast(message)
    }

    /// Broadcasts the currently detected people so the controller can draw
    /// tappable outlines over the video stream.
    func broadcastPersonBoxes(_ boxes: [RemotePersonBox]) {
        var message = RemoteMessage(type: "personBoxes")
        message.personBoxes = boxes
        broadcast(message)
    }

    func broadcastGridReset() {
        latestMapStateMessage = nil
        latestGridUpdateMessage = nil
        latestMeshMessage = nil
        broadcast(RemoteMessage(type: "gridReset"))
    }

    private func broadcast(_ message: RemoteMessage) {
        guard clientCount > 0 else { return }
        var outbound = message
        outbound.seq = seq
        outbound.ts = Date().timeIntervalSince1970
        seq += 1
        guard let data = try? encoder.encode(outbound) else { return }
        transport.sendMessage(data)
    }

    private func replayLatestState() {
        queue.async { [weak self] in
            guard let self else { return }

            var status = RemoteMessage(type: "status")
            status.bleConnected = ESP32BLEManager.shared.connectionState == .connected
            status.message = "\(self.clientCount) remote client(s)"
            self.latestStatusMessage = status

            self.broadcast(RemoteMessage(type: "gridReset"))

            ([
                self.latestStatusMessage,
                self.latestMapStateMessage,
                self.latestGridUpdateMessage,
                self.latestMeshMessage,
                self.latestServoIDsMessage,
                self.latestServoStateMessage,
                self.latestServoPositionsMessage
            ].compactMap { $0 } + Array(self.latestServoAxisStatusMessages.values)).forEach { self.broadcast($0) }
        }
    }

    private func startDriveWatchdog() {
        staleDriveTimer?.cancel()
        let timer = DispatchSource.makeTimerSource(queue: queue)
        timer.schedule(deadline: .now() + driveTimeout, repeating: 0.25)
        timer.setEventHandler { [weak self] in
            guard let self, let lastDriveCommandDate = self.lastDriveCommandDate else { return }
            if Date().timeIntervalSince(lastDriveCommandDate) > self.driveTimeout {
                self.lastDriveCommandDate = nil
                DispatchQueue.main.async { ESP32BLEManager.shared.stopAll() }
            }
        }
        staleDriveTimer = timer
        timer.resume()
    }

    private func publishStatus(_ status: String) {
        print("[RemoteHost] \(status)")
        DispatchQueue.main.async { [weak self] in
            self?.onStatusChanged?(status)
        }
    }
}
