//
//  RemoteRobotCommandDispatcher.swift
//  RoboCar
//

import Foundation

enum RemoteRobotCommandDispatcher {
    static func dispatch(_ message: RemoteMessage) {
        DispatchQueue.main.async {
            switch message.type {
            case "drive":
                if PathNavigator.shared.state != .idle {
                    NotificationCenter.default.post(name: .serverStopNavigation, object: nil)
                }
                ESP32BLEManager.shared.drive(x: message.x ?? 0, y: message.y ?? 0)

            case "stopDrive":
                ESP32BLEManager.shared.stopAll()

            case "setNavTarget":
                guard let x = message.x, let y = message.y else { return }
                NotificationCenter.default.post(name: .serverSetNavTarget, object: nil, userInfo: ["x": x, "y": y])

            case "addRoutePoint":
                guard let x = message.x, let y = message.y else { return }
                NotificationCenter.default.post(name: .serverAddRoutePoint, object: nil, userInfo: ["x": x, "y": y])

            case "runRoute":
                NotificationCenter.default.post(name: .serverRunRoute, object: nil)

            case "pauseRoute":
                NotificationCenter.default.post(name: .serverPauseRoute, object: nil)

            case "clearRoute":
                NotificationCenter.default.post(name: .serverClearRoute, object: nil)

            case "startNavigation":
                NotificationCenter.default.post(name: .serverStartNavigation, object: nil)

            case "stopNavigation":
                NotificationCenter.default.post(name: .serverStopNavigation, object: nil)

            case "nlCommand":
                guard let command = message.nlCommand, !command.isEmpty else { return }
                NLNavigator.shared.start(command: command)

            case "stopNLCommand":
                NLNavigator.shared.stop()

            case "followPerson":
                if let personID = message.personID {
                    NotificationCenter.default.post(name: .remoteFollowPerson, object: nil,
                                                    userInfo: ["personID": personID])
                } else if let personName = message.personName {
                    NotificationCenter.default.post(name: .remoteFollowPersonByName, object: nil,
                                                    userInfo: ["personName": personName])
                }

            case "stopFollowing":
                NotificationCenter.default.post(name: .stopFollowing, object: nil)

            case "namePerson":
                guard let personID = message.personID, let personName = message.personName else { return }
                NotificationCenter.default.post(name: .remoteNamePerson, object: nil,
                                                userInfo: ["personID": personID, "personName": personName])

            case "deleteNamedPerson":
                guard let personName = message.personName else { return }
                NotificationCenter.default.post(name: .remoteDeleteNamedPerson, object: nil,
                                                userInfo: ["personName": personName])

            case "scanServos":
                ESP32BLEManager.shared.rescanServos(from: message.from ?? 1, to: message.to ?? 20)

            case "moveServo":
                guard let id = message.id, let position = message.position else { return }
                ESP32BLEManager.shared.moveServo(id: id, position: position, speed: message.speed ?? 1000)

            case "stopServo":
                guard let id = message.id else { return }
                ESP32BLEManager.shared.stopServo(id: id)

            case "moveAllServos":
                guard let position = message.position else { return }
                ESP32BLEManager.shared.moveDiscoveredServos(position: position, speed: message.speed ?? 1000, acceleration: message.acceleration ?? 50)

            case "setServoTorque":
                guard let id = message.id, let enabled = message.enabled else { return }
                ESP32BLEManager.shared.setServoTorque(id: id, enabled: enabled)

            case "calibrateServoZero":
                guard let id = message.id else { return }
                ESP32BLEManager.shared.calibrateServoZero(id: id)

            case "driveServoWheel":
                guard let id = message.id else { return }
                ESP32BLEManager.shared.driveServoWheel(id: id, speed: message.wheelSpeed ?? 0, acceleration: message.acceleration ?? 50)

            case "setServoPositionMode":
                guard let id = message.id else { return }
                ESP32BLEManager.shared.setServoPositionMode(id: id)

            case "refreshServoState":
                guard let id = message.id else { return }
                ESP32BLEManager.shared.refreshServoState(id: id)

            default:
                break
            }
        }
    }
}
