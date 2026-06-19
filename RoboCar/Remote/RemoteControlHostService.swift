//
//  RemoteControlHostService.swift
//  RoboCar
//

import Foundation
import Network
import UIKit
import WebRTC

final class RemoteControlHostService {
    static let shared = RemoteControlHostService()

    var onStatusChanged: ((String) -> Void)?


    private let queue = DispatchQueue(label: "com.robocar.remote.host", qos: .userInitiated)
    private let encoder = JSONEncoder()
    private let decoder = JSONDecoder()
    private var listener: NWListener?
    private var signalingConnection: NWConnection?
    private var receiveBuffer = ""
    private var webRTCSession: RemoteControlWebRTCSession?
    private var latestStatusMessage: RemoteMessage?
    private var latestMapStateMessage: RemoteMessage?
    private var latestGridUpdateMessage: RemoteMessage?
    private var latestServoIDsMessage: RemoteMessage?
    private var latestServoStateMessage: RemoteMessage?
    private var seq: UInt64 = 0
    private var staleDriveTimer: DispatchSourceTimer?
    private var motorStopTimer: DispatchSourceTimer?
    private var lastDriveCommandDate: Date?
    private let driveTimeout: TimeInterval = 0.75
    private var attemptedVideoFrameCount: UInt64 = 0

    private init() {}

    var isRunning: Bool { listener != nil }
    var clientCount: Int { signalingConnection == nil ? 0 : 1 }
    private var isLocalConnectionDetected = false
    var isLocalConnection: Bool { isLocalConnectionDetected }

    func start(port: UInt16 = RemoteControlProtocol.defaultPort) {
        guard listener == nil else { return }

        do {
            let parameters = NWParameters.tcp
            parameters.includePeerToPeer = true
            let listener = try NWListener(using: parameters, on: NWEndpoint.Port(rawValue: port)!)
            listener.service = NWListener.Service(name: UIDevice.current.name, type: RemoteControlProtocol.serviceType)
            listener.stateUpdateHandler = { [weak self] state in
                self?.handleListenerState(state)
            }
            listener.newConnectionHandler = { [weak self] connection in
                self?.accept(connection)
            }
            self.listener = listener
            startDriveWatchdog()
            listener.start(queue: queue)
        } catch {
            publishStatus("Remote host failed: \(error.localizedDescription)")
        }
    }

    func stop() {
        queue.async { [weak self] in
            guard let self else { return }
            guard self.listener != nil || self.signalingConnection != nil || self.webRTCSession != nil else { return }
            self.listener?.cancel()
            self.listener = nil
            self.signalingConnection?.cancel()
            self.signalingConnection = nil
            self.receiveBuffer = ""
            self.webRTCSession?.stop()
            self.webRTCSession = nil
            self.staleDriveTimer?.cancel()
            self.staleDriveTimer = nil
            self.motorStopTimer?.cancel()
            self.motorStopTimer = nil
            self.lastDriveCommandDate = nil
            DispatchQueue.main.async { ESP32BLEManager.shared.stopAll() }
            self.publishStatus("Remote host stopped")
        }
    }

    func broadcastStatus() {
        var message = RemoteMessage(type: "status")
        message.bleConnected = ESP32BLEManager.shared.connectionState == .connected
        message.message = "\(clientCount) remote client(s)"
        message.isLocalConnection = isLocalConnection
        latestStatusMessage = message
        broadcast(message)
    }

    func broadcastMapState(occupancyGrid: OccupancyGrid, routeWaypoints: [(x: Float, y: Float)], plannedPath: [(x: Float, y: Float)], routePreviewPaths: [[(x: Float, y: Float)]], activeRouteWaypointIndex: Int, navState: String) {
        var message = RemoteMessage(type: "mapState")
        message.pose = RemotePose(devicePosition: occupancyGrid.devicePosition)
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
        attemptedVideoFrameCount += 1
        guard let webRTCSession else {
            if attemptedVideoFrameCount == 1 || attemptedVideoFrameCount % 30 == 0 {
                print("[RemoteWebRTC] host: dropped view frame #\(attemptedVideoFrameCount), no session")
            }
            return
        }
        if !webRTCSession.canAcceptVideoFrame,
           attemptedVideoFrameCount == 1 || attemptedVideoFrameCount % 30 == 0 {
            print("[RemoteWebRTC] host: view frame #\(attemptedVideoFrameCount) before local video track ready")
        }
        webRTCSession.sendVideoFrame(image)
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

    func broadcastGridReset() {
        broadcast(RemoteMessage(type: "gridReset"))
    }

    private func handleListenerState(_ state: NWListener.State) {
        switch state {
        case .ready:
            publishStatus("Remote host available on Bonjour")
        case .failed(let error):
            publishStatus("Remote host failed: \(error.localizedDescription)")
            stop()
        case .cancelled:
            publishStatus("Remote host stopped")
        default:
            break
        }
    }

    private func accept(_ connection: NWConnection) {
        motorStopTimer?.cancel()
        motorStopTimer = nil
        signalingConnection?.cancel()
        signalingConnection = connection
        receiveBuffer = ""
        let oldSession = webRTCSession
        let session = RemoteControlWebRTCSession(role: .host)
        session.onSignal = { [weak self] message in
            self?.sendSignal(message)
        }
        session.onStatusChanged = { [weak self] status in
            self?.publishStatus(status)
        }
        session.onLocalFrameSent = { [weak self] width, height, count in
            guard count == 1 || count % 30 == 0 else { return }
            self?.publishStatus("WebRTC sent view frame #\(count) (\(width)x\(height))")
        }
        webRTCSession = session
        oldSession?.stop()
        connection.stateUpdateHandler = { [weak self, weak connection] state in
            guard let self, let connection else { return }
            switch state {
            case .ready:
                self.publishStatus("WebRTC signaling connected")
                print("[RemoteWebRTC] host: TCP signaling ready, starting peer connection")
                session.start()
                self.broadcastStatus()
                self.replayLatestState()
                self.receive(on: connection)
            case .failed, .cancelled:
                self.remove(connection)
            default:
                break
            }
        }
        connection.start(queue: queue)
    }

    private func remove(_ connection: NWConnection) {
        guard signalingConnection === connection else { return }
        signalingConnection = nil
        receiveBuffer = ""
        webRTCSession?.stop()
        webRTCSession = nil
        DispatchQueue.main.async { [weak self] in self?.isLocalConnectionDetected = false }
        scheduleMotorStop()
        publishStatus("Remote client disconnected")
        broadcastStatus()
    }

    private func scheduleMotorStop() {
        motorStopTimer?.cancel()
        let timer = DispatchSource.makeTimerSource(queue: queue)
        timer.schedule(deadline: .now() + 2.0)
        timer.setEventHandler { [weak self] in
            guard let self, self.signalingConnection == nil else { return }
            self.motorStopTimer = nil
            DispatchQueue.main.async { ESP32BLEManager.shared.stopAll() }
        }
        motorStopTimer = timer
        timer.resume()
    }

    private func receive(on connection: NWConnection) {
        connection.receive(minimumIncompleteLength: 1, maximumLength: 64 * 1024) { [weak self, weak connection] data, _, isComplete, error in
            guard let self, let connection else { return }
            if let data, !data.isEmpty, let chunk = String(data: data, encoding: .utf8) {
                self.handle(chunk: chunk, from: connection)
            }
            if isComplete || error != nil {
                self.remove(connection)
                return
            }
            self.receive(on: connection)
        }
    }

    private func handle(chunk: String, from connection: NWConnection) {
        receiveBuffer += chunk
        while let newline = receiveBuffer.firstIndex(of: "\n") {
            let line = String(receiveBuffer[..<newline])
            receiveBuffer.removeSubrange(...newline)
            guard let data = line.data(using: .utf8), let message = try? decoder.decode(RemoteMessage.self, from: data) else { continue }
            if message.type == "webrtcSignal" {
                print("[RemoteWebRTC] host: received signal \(message.signalType ?? "unknown")")
                webRTCSession?.handleSignal(message)
                continue
            }
            if message.type == "connectionInfo" {
                let isLocal = message.isLocalConnection ?? false
                webRTCSession?.configureForLocalConnection(isLocal)
                DispatchQueue.main.async { [weak self] in
                    self?.isLocalConnectionDetected = isLocal
                }
                publishStatus("WebRTC connection: \(isLocal ? "local" : "remote")")
                broadcastStatus()
                continue
            }
            if message.type == "drive" {
                lastDriveCommandDate = Date()
            }
            if message.type == "stopDrive" {
                lastDriveCommandDate = nil
            }
            RemoteRobotCommandDispatcher.dispatch(message)
        }
    }

    private func broadcast(_ message: RemoteMessage) {
        queue.async { [weak self] in
            guard let self, let signalingConnection else { return }
            var outbound = message
            outbound.seq = self.seq
            outbound.ts = Date().timeIntervalSince1970
            self.seq += 1
            guard let data = try? self.encoder.encode(outbound) else { return }
            var framed = data
            framed.append(0x0A)
            signalingConnection.send(content: framed, completion: .contentProcessed { _ in })
        }
    }

    private func replayLatestState() {
        queue.async { [weak self] in
            guard let self else { return }

            var status = RemoteMessage(type: "status")
            status.bleConnected = ESP32BLEManager.shared.connectionState == .connected
            status.message = "\(self.clientCount) remote client(s)"
            self.latestStatusMessage = status

            [
                self.latestStatusMessage,
                self.latestMapStateMessage,
                self.latestGridUpdateMessage,
                self.latestServoIDsMessage,
                self.latestServoStateMessage
            ].compactMap { $0 }.forEach { self.broadcast($0) }
        }
    }

    private func sendSignal(_ message: RemoteMessage) {
        queue.async { [weak self] in
            guard let self, let signalingConnection else { return }
            guard let data = try? self.encoder.encode(message) else { return }
            var framed = data
            framed.append(0x0A)
            print("[RemoteWebRTC] host: sending signal \(message.signalType ?? "unknown")")
            signalingConnection.send(content: framed, completion: .contentProcessed { _ in })
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
        print("[RemoteWebRTC] host status: \(status)")
        DispatchQueue.main.async { [weak self] in
            self?.onStatusChanged?(status)
        }
    }
}
