//
//  FoundationModelService.swift
//  RoboCar
//
//  Created by Sammy Yousif on 2/17/26.
//

import Foundation
import FoundationModels

// MARK: - Movement Tools

/// Drive the car forward or backward a precise distance using ARKit closed-loop tracking.
struct DriveDistanceTool: Tool {
    let name = "driveDistance"
    let description = "Drive the car forward or backward a precise distance. Convert feet to meters first (1 ft = 0.3048 m, 1 yard = 0.914 m)."

    @Generable
    struct Arguments {
        @Guide(description: "Distance in meters to travel", .range(0.05...15.0))
        var distanceMeters: Double

        @Guide(description: "Direction to drive: 'forward' or 'backward'")
        var direction: String

        @Guide(description: "Speed percentage (10–80 for normal indoor use)", .range(10...80))
        var speed: Int
    }

    func call(arguments: Arguments) async throws -> String {
        guard let grid = ObstacleDetector.shared.occupancyGrid else {
            return "No occupancy grid available — LiDAR must be running for precise distance driving."
        }
        let target = Float(max(0.05, arguments.distanceMeters))
        let power = Int8(min(80, max(10, arguments.speed)))
        let isForward = arguments.direction.lowercased() != "backward"
        let motorPower = isForward ? power : -power

        let start = grid.devicePosition

        await MainActor.run {
            ESP32BLEManager.shared.setAllMotors(a: motorPower, b: motorPower, c: motorPower, d: motorPower)
        }

        // Timeout: at minimum speed ~5 cm/s, add 5 s buffer
        let deadline = Date().addingTimeInterval(Double(target) / 0.05 + 5.0)
        while Date() < deadline {
            try Task.checkCancellation()
            try await Task.sleep(for: .milliseconds(50))
            let pos = grid.devicePosition
            let dx = pos.x - start.x
            let dy = pos.y - start.y
            if sqrtf(dx * dx + dy * dy) >= target { break }
        }

        await MainActor.run { ESP32BLEManager.shared.stopAll() }

        let end = grid.devicePosition
        let dx = end.x - start.x
        let dy = end.y - start.y
        let actual = sqrtf(dx * dx + dy * dy)
        let dir = isForward ? "forward" : "backward"
        return String(format: "Drove %@ %.2f m (target %.2f m).", dir, actual, target)
    }
}

/// Spin the car left or right by a precise number of degrees using ARKit heading tracking.
struct TurnTool: Tool {
    let name = "turnDegrees"
    let description = "Spin the car precisely left or right by a given number of degrees in place."

    @Generable
    struct Arguments {
        @Guide(description: "Degrees to turn, must be positive", .range(1.0...360.0))
        var degrees: Double

        @Guide(description: "Direction to turn: 'left' or 'right'")
        var direction: String

        @Guide(description: "Speed percentage (20–50 for accurate turns)", .range(20...50))
        var speed: Int
    }

    func call(arguments: Arguments) async throws -> String {
        guard let grid = ObstacleDetector.shared.occupancyGrid else {
            return "No occupancy grid available — LiDAR must be running for precise turns."
        }
        let targetRads = Float(arguments.degrees) * .pi / 180.0
        let power = Int8(min(50, max(20, arguments.speed)))
        let isLeft = arguments.direction.lowercased() != "right"
        // Left turn: A,C backward; B,D forward. Right turn: opposite.
        let lPow: Int8 = isLeft ? -power : power
        let rPow: Int8 = isLeft ? power : -power

        var totalTurned: Float = 0
        var lastHeading = grid.devicePosition.heading

        await MainActor.run {
            ESP32BLEManager.shared.setAllMotors(a: lPow, b: rPow, c: lPow, d: rPow)
        }

        let deadline = Date().addingTimeInterval(10.0)
        while Date() < deadline && totalTurned < targetRads {
            try Task.checkCancellation()
            try await Task.sleep(for: .milliseconds(50))
            let h = grid.devicePosition.heading
            var delta = h - lastHeading
            while delta >  .pi { delta -= 2 * .pi }
            while delta < -.pi { delta += 2 * .pi }
            totalTurned += fabsf(delta)
            lastHeading = h
        }

        await MainActor.run { ESP32BLEManager.shared.stopAll() }

        let actualDeg = totalTurned * 180 / .pi
        let dir = isLeft ? "left" : "right"
        return String(format: "Turned %@ %.0f° (target %.0f°).", dir, actualDeg, arguments.degrees)
    }
}

/// Tool that stops all motors immediately
struct StopMotorsTool: Tool {
    let name = "stopMotors"
    let description = "Immediately stop all motors on the car."

    @Generable
    struct Arguments {}

    func call(arguments: Arguments) async throws -> String {
        await MainActor.run {
            ESP32BLEManager.shared.stopAll()
        }
        return "All motors stopped."
    }
}

/// Tool that moves an ST3215 servo
struct SetServoTool: Tool {
    let name = "setServo"
    let description = "Move an ST3215 servo to a position. Use servo ID 1 unless the user names a different ID."

    @Generable
    struct Arguments {
        @Guide(description: "Servo ID from 1 to 253.", .range(1...253))
        var id: Int

        @Guide(description: "Servo angle from 0 to 180 degrees. 90 is center.", .range(0...180))
        var angle: Int
    }

    func call(arguments: Arguments) async throws -> String {
        let angle = UInt8(min(180, max(0, arguments.angle)))
        let id = UInt8(min(253, max(1, arguments.id)))
        let position = UInt16((Double(angle) / 180.0 * 4095.0).rounded())
        await MainActor.run {
            ESP32BLEManager.shared.moveServo(id: id, position: position, speed: 1000)
        }
        return "Servo \(id) set to \(angle) degrees."
    }
}

/// Tool that starts autonomous exploration to map the entire area
struct ExploreAreaTool: Tool {
    let name = "exploreArea"
    let description = "Start autonomous exploration. The car will drive around by itself to build a complete 2D map of the room, stopping when the area is fully bounded by walls and obstacles. This takes a few minutes."

    @Generable
    struct Arguments {}

    func call(arguments: Arguments) async throws -> String {
        return try await ExplorationController.shared.startExploration()
    }
}

/// Tool that stops autonomous exploration
struct StopExplorationTool: Tool {
    let name = "stopExploration"
    let description = "Stop the autonomous exploration that is currently in progress."

    @Generable
    struct Arguments {}

    func call(arguments: Arguments) async throws -> String {
        await MainActor.run {
            ExplorationController.shared.stopExploration()
        }
        return "Exploration stopped."
    }
}

/// Tool that runs motor calibration to measure speed and turn rates
struct CalibrateMotorsTool: Tool {
    let name = "calibrateMotors"
    let description = "Run a motor calibration sequence. The car will test various power levels and measure actual speed and turn rates using iPhone sensors. Needs clear space around the car. Takes about 30 seconds."

    @Generable
    struct Arguments {}

    func call(arguments: Arguments) async throws -> String {
        return try await MotorCalibrator.shared.runCalibration()
    }
}

/// Tool that starts following a person — either a specific saved person by name,
/// or the nearest visible person when no name is given.
struct FollowPersonTool: Tool {
    let name = "followPerson"
    let description = "Follow a person with the camera. If the user names a specific person (e.g. 'follow Alex'), pass that name and the car will follow that saved person — following immediately if they're visible, or watching for them to appear if they're not. If no name is given (e.g. 'follow me' or 'follow that person'), the car follows the nearest visible person. Once locked on, the car continuously drives to stay behind them at a safe distance."

    @Generable
    struct Arguments {
        @Guide(description: "Optional name of the specific saved person to follow. Omit to follow the nearest visible person.")
        var personName: String?
    }

    func call(arguments: Arguments) async throws -> String {
        let requestedName = arguments.personName?.trimmingCharacters(in: .whitespacesAndNewlines)
        await MainActor.run {
            var userInfo: [AnyHashable: Any] = [:]
            if let requestedName, !requestedName.isEmpty {
                userInfo["name"] = requestedName
            }
            NotificationCenter.default.post(name: .startFollowing, object: nil, userInfo: userInfo)
        }
        try await Task.sleep(for: .seconds(0.5))
        let state = await MainActor.run { PersonTracker.shared.state }
        if state == .tracking {
            if let requestedName, !requestedName.isEmpty {
                return "Found \(requestedName)! Following them now."
            }
            return "Got it — following the nearest person now. I'll stay about a meter behind."
        } else if let requestedName, !requestedName.isEmpty {
            return "I don't see \(requestedName) yet. I'll keep watching and start following the moment they appear."
        } else {
            return "I don't see anyone yet. I'll start following as soon as someone steps into view."
        }
    }
}

/// Tool that stops following the person
struct StopFollowingTool: Tool {
    let name = "stopFollowing"
    let description = "Stop following the person and return to idle mode."

    @Generable
    struct Arguments {}

    func call(arguments: Arguments) async throws -> String {
        await MainActor.run {
            NotificationCenter.default.post(name: .stopFollowing, object: nil)
        }
        return "Stopped following. Standing by."
    }
}

/// Navigate to a named scene object using stored landmarks or vision-guided search.
struct NavigateToObjectTool: Tool {
    let name = "navigateToObject"
    let description = "Navigate to a named real-world object or place (e.g. 'fridge', 'couch', 'door', 'table', 'window', 'kitchen'). Uses stored landmark positions when available; otherwise starts a vision-guided search."

    @Generable
    struct Arguments {
        @Guide(description: "Name of the object or place to navigate to")
        var objectName: String
    }

    func call(arguments: Arguments) async throws -> String {
        let name = arguments.objectName.trimmingCharacters(in: .whitespaces)

        // Refresh mesh-based landmarks (door, table, couch, window) in background
        if let grid = ObstacleDetector.shared.occupancyGrid {
            let g = grid
            await withCheckedContinuation { (cont: CheckedContinuation<Void, Never>) in
                DispatchQueue.global(qos: .utility).async {
                    SceneLandmarkStore.shared.updateMeshClassifications(from: g)
                    cont.resume()
                }
            }
        }

        // Navigate directly to a stored landmark if one matches
        if let landmark = SceneLandmarkStore.shared.findBestMatch(for: name) {
            let tx = landmark.worldX
            let ty = landmark.worldY
            await MainActor.run {
                NotificationCenter.default.post(
                    name: .serverSetNavTarget,
                    object: nil,
                    userInfo: ["x": tx, "y": ty]
                )
            }
            return "Navigating to '\(landmark.name)' at (\(String(format: "%.1f", tx)), \(String(format: "%.1f", ty)))."
        }

        // Fall back to vision-guided NL navigation
        await MainActor.run {
            NLNavigator.shared.start(command: "Navigate to the \(name)")
        }
        return "Starting vision-guided navigation to '\(name)'. I'll use the camera to find it."
    }
}

// MARK: - Foundation Model Service

/// Manages the Apple Foundation Models session with tool calling for car control
class FoundationModelService {

    static let shared = FoundationModelService()

    private var session: LanguageModelSession?

    private let instructions = """
    You are a helpful voice assistant that controls a robotic car via Bluetooth. \
    You can drive with precision, navigate to scene objects, follow people, map the room, and more.

    Movement:
    - Use driveDistance to move forward or backward by exact distance. \
      Convert feet/yards to meters (1 ft = 0.3048 m). Default speed is 50.
    - Use turnDegrees to rotate left or right by exact degrees. Default speed is 30.
    - Use stopMotors to halt immediately.

    Navigation:
    - Use navigateToObject to go to a named object or place ("fridge", "door", "couch", etc.). \
      The robot uses stored landmark positions when available, or switches to vision-guided search.

    Other:
    - Use exploreArea to autonomously map the entire room.
    - Use calibrateMotors to measure actual speed and turn rates.
    - Use followPerson / stopFollowing to track a person.
    - Use setServo to move a servo arm.

    Keep spoken responses very brief and conversational — they are read aloud. \
    If the user gives a distance in feet or yards, convert to meters before calling driveDistance. \
    Default speed is 50 for driving and 30 for turns unless the user specifies.
    """

    private init() {
        createSession()
    }

    private func createSession() {
        session = LanguageModelSession(
            tools: [
                DriveDistanceTool(),
                TurnTool(),
                StopMotorsTool(),
                SetServoTool(),
                NavigateToObjectTool(),
                ExploreAreaTool(),
                StopExplorationTool(),
                CalibrateMotorsTool(),
                FollowPersonTool(),
                StopFollowingTool(),
            ],
            instructions: instructions
        )
    }

    /// Send a command to the model and get a text response
    func sendCommand(_ command: String) async throws -> String {
        guard let session else {
            createSession()
            return try await sendCommand(command)
        }

        do {
            let response = try await session.respond {
                command
            }
            return response.content
        } catch {
            print("[Model] Error: \(error)")
            createSession()
            throw error
        }
    }

    /// Reset the session (clears conversation history)
    func resetSession() {
        createSession()
    }
}
