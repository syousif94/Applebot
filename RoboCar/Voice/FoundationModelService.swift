//
//  FoundationModelService.swift
//  RoboCar
//
//  Created by Sammy Yousif on 2/17/26.
//

import Foundation
import FoundationModels

// MARK: - Tools

/// Tool that drives the car forward at a given speed for a duration
struct DriveForwardTool: Tool {
    let name = "driveForward"
    let description = "Drive the car forward at a given speed for a number of seconds."
    
    @Generable
    struct Arguments {
        @Guide(description: "Speed percentage from 1 to 100", .range(1...100))
        var speed: Int
        
        @Guide(description: "Duration in seconds to drive forward", .range(0.5...10.0))
        var duration: Double
    }
    
    func call(arguments: Arguments) async throws -> String {
        let power = Int8(min(100, max(1, arguments.speed)))
        await MainActor.run {
            ESP32BLEManager.shared.setAllMotors(a: power, b: power, c: power, d: power)
        }
        
        try await Task.sleep(for: .seconds(arguments.duration))
        
        await MainActor.run {
            ESP32BLEManager.shared.stopAll()
        }
        
        return "Drove forward at \(arguments.speed)% speed for \(String(format: "%.1f", arguments.duration)) seconds, then stopped."
    }
}

/// Tool that drives the car backward at a given speed for a duration
struct DriveBackwardTool: Tool {
    let name = "driveBackward"
    let description = "Drive the car backward at a given speed for a number of seconds."
    
    @Generable
    struct Arguments {
        @Guide(description: "Speed percentage from 1 to 100", .range(1...100))
        var speed: Int
        
        @Guide(description: "Duration in seconds to drive backward", .range(0.5...10.0))
        var duration: Double
    }
    
    func call(arguments: Arguments) async throws -> String {
        let power = -Int8(min(100, max(1, arguments.speed)))
        await MainActor.run {
            ESP32BLEManager.shared.setAllMotors(a: power, b: power, c: power, d: power)
        }
        
        try await Task.sleep(for: .seconds(arguments.duration))
        
        await MainActor.run {
            ESP32BLEManager.shared.stopAll()
        }
        
        return "Drove backward at \(arguments.speed)% speed for \(String(format: "%.1f", arguments.duration)) seconds, then stopped."
    }
}

/// Tool that turns the car left (spins in place or arcs)
struct TurnLeftTool: Tool {
    let name = "turnLeft"
    let description = "Turn the car to the left by spinning in place for a duration."
    
    @Generable
    struct Arguments {
        @Guide(description: "Speed percentage from 1 to 100", .range(1...100))
        var speed: Int
        
        @Guide(description: "Duration in seconds to turn", .range(0.2...5.0))
        var duration: Double
    }
    
    func call(arguments: Arguments) async throws -> String {
        let power = Int8(min(100, max(1, arguments.speed)))
        // Left wheels backward, right wheels forward = turn left
        await MainActor.run {
            ESP32BLEManager.shared.setAllMotors(a: -power, b: power, c: -power, d: power)
        }
        
        try await Task.sleep(for: .seconds(arguments.duration))
        
        await MainActor.run {
            ESP32BLEManager.shared.stopAll()
        }
        
        return "Turned left at \(arguments.speed)% speed for \(String(format: "%.1f", arguments.duration)) seconds, then stopped."
    }
}

/// Tool that turns the car right (spins in place or arcs)
struct TurnRightTool: Tool {
    let name = "turnRight"
    let description = "Turn the car to the right by spinning in place for a duration."
    
    @Generable
    struct Arguments {
        @Guide(description: "Speed percentage from 1 to 100", .range(1...100))
        var speed: Int
        
        @Guide(description: "Duration in seconds to turn", .range(0.2...5.0))
        var duration: Double
    }
    
    func call(arguments: Arguments) async throws -> String {
        let power = Int8(min(100, max(1, arguments.speed)))
        // Left wheels forward, right wheels backward = turn right
        await MainActor.run {
            ESP32BLEManager.shared.setAllMotors(a: power, b: -power, c: power, d: -power)
        }
        
        try await Task.sleep(for: .seconds(arguments.duration))
        
        await MainActor.run {
            ESP32BLEManager.shared.stopAll()
        }
        
        return "Turned right at \(arguments.speed)% speed for \(String(format: "%.1f", arguments.duration)) seconds, then stopped."
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
        // Brief delay for initial detection / activation
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

// MARK: - Foundation Model Service

/// Manages the Apple Foundation Models session with tool calling for car control
class FoundationModelService {
    
    static let shared = FoundationModelService()
    
    private var session: LanguageModelSession?
    
    private let instructions = """
    You are a helpful voice assistant that controls a robotic car via Bluetooth. \
    You can drive the car forward, backward, turn left, turn right, stop, set the steering servo, \
    start autonomous exploration to map the entire room, calibrate the motors to measure \
    actual speed and turn rates, and follow a person around. \
    When the user asks you to move the car, use the appropriate tool. \
    When the user asks to map the room or explore the area, use the exploreArea tool. \
    When the user asks to calibrate, test the motors, or measure speed, use the calibrateMotors tool. \
    When the user asks to follow them or follow a person, use the followPerson tool. \
    If they name a specific person (e.g. "follow Alex"), pass that name to the tool; \
    otherwise it follows the nearest visible person. \
    When the user asks to stop following, use the stopFollowing tool. \
    Keep your spoken responses very brief and conversational — you are being read aloud. \
    If the user asks something unrelated to the car, answer briefly. \
    Default to 50% speed and 1 second duration if the user doesn't specify.
    """
    
    private init() {
        createSession()
    }
    
    private func createSession() {
        session = LanguageModelSession(
            tools: [
                DriveForwardTool(),
                DriveBackwardTool(),
                TurnLeftTool(),
                TurnRightTool(),
                StopMotorsTool(),
                SetServoTool(),
                ExploreAreaTool(),
                StopExplorationTool(),
                CalibrateMotorsTool(),
                FollowPersonTool(),
                StopFollowingTool()
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
