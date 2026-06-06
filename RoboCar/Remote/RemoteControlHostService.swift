//
//  RemoteControlHostService.swift
//  RoboCar
//

import Foundation
import Network
import UIKit

final class RemoteControlHostService {
    static let shared = RemoteControlHostService()

    var onStatusChanged: ((String) -> Void)?


    private let queue = DispatchQueue(label: "com.robocar.remote.host", qos: .userInitiated)
    private let encoder = JSONEncoder()
    private let decoder = JSONDecoder()
    private var listener: NWListener?
    private var connections: [ObjectIdentifier: NWConnection] = [:]
    private var receiveBuffers: [ObjectIdentifier: String] = [:]
    private var seq: UInt64 = 0
    private var staleDriveTimer: DispatchSourceTimer?
    private var lastDriveCommandDate: Date?
    private let driveTimeout: TimeInterval = 0.75

    private init() {}

    var isRunning: Bool { listener != nil }
    var clientCount: Int { connections.count }

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
            self.listener?.cancel()
            self.listener = nil
            self.connections.values.forEach { $0.cancel() }
            self.connections.removeAll()
            self.receiveBuffers.removeAll()
            self.staleDriveTimer?.cancel()
            self.staleDriveTimer = nil
            self.lastDriveCommandDate = nil
            DispatchQueue.main.async { ESP32BLEManager.shared.stopAll() }
            self.publishStatus("Remote host stopped")
        }
    }

    func broadcastStatus() {
        var message = RemoteMessage(type: "status")
        message.bleConnected = ESP32BLEManager.shared.connectionState == .connected
        message.message = "\(connections.count) remote client(s)"
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
        broadcast(message)
    }

    func broadcastGridUpdate(occupancyGrid: OccupancyGrid, radiusMeters: Float = 6.0) {
        guard !connections.isEmpty else { return }
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
        broadcast(message)
    }

    func broadcastCameraFrame(jpegData: Data, width: Int, height: Int) {
        guard !connections.isEmpty else { return }
        var message = RemoteMessage(type: "cameraFrame")
        message.camera = RemoteCameraFrame(width: width, height: height, jpegBase64: jpegData.base64EncodedString())
        broadcast(message)
    }

    func broadcastServoIDs(_ ids: [UInt8]) {
        var message = RemoteMessage(type: "servoList")
        message.servoIDs = ids
        broadcast(message)
    }

    func broadcastServoState(_ state: ServoState) {
        var message = RemoteMessage(type: "servoState")
        message.servoState = RemoteServoState(state)
        broadcast(message)
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
        let key = ObjectIdentifier(connection)
        connections[key] = connection
        receiveBuffers[key] = ""
        connection.stateUpdateHandler = { [weak self, weak connection] state in
            guard let self, let connection else { return }
            switch state {
            case .ready:
                self.publishStatus("Remote client connected")
                self.broadcastStatus()
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
        let key = ObjectIdentifier(connection)
        connections.removeValue(forKey: key)
        receiveBuffers.removeValue(forKey: key)
        DispatchQueue.main.async { ESP32BLEManager.shared.stopAll() }
        publishStatus("Remote client disconnected")
        broadcastStatus()
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
        let key = ObjectIdentifier(connection)
        var buffer = (receiveBuffers[key] ?? "") + chunk
        while let newline = buffer.firstIndex(of: "\n") {
            let line = String(buffer[..<newline])
            buffer.removeSubrange(...newline)
            guard let data = line.data(using: .utf8), let message = try? decoder.decode(RemoteMessage.self, from: data) else { continue }
            if message.type == "drive" {
                lastDriveCommandDate = Date()
            }
            if message.type == "stopDrive" {
                lastDriveCommandDate = nil
            }
            RemoteRobotCommandDispatcher.dispatch(message)
        }
        receiveBuffers[key] = buffer
    }

    private func broadcast(_ message: RemoteMessage) {
        queue.async { [weak self] in
            guard let self, !self.connections.isEmpty else { return }
            var outbound = message
            outbound.seq = self.seq
            outbound.ts = Date().timeIntervalSince1970
            self.seq += 1
            guard let data = try? self.encoder.encode(outbound) else { return }
            var framed = data
            framed.append(0x0A)
            for connection in self.connections.values {
                connection.send(content: framed, completion: .contentProcessed { _ in })
            }
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
        DispatchQueue.main.async { [weak self] in
            self?.onStatusChanged?(status)
        }
    }
}
