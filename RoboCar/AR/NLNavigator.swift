//
//  NLNavigator.swift
//  RoboCar
//

import Foundation
import FoundationModels
import ARKit
import CoreImage
import UIKit

// MARK: - Navigation Tools

/// Moves the robot toward a world coordinate chosen by the model.
struct NavigateToGridPointTool: Tool {
    let name = "navigateToGridPoint"
    let description = "Navigate the robot toward a target world coordinate. Choose a point 1–3 meters ahead in the direction of the goal."

    @Generable
    struct Arguments {
        @Guide(description: "World X coordinate to navigate toward, in meters from the AR origin")
        var x: Double

        @Guide(description: "World Y coordinate to navigate toward, in meters from the AR origin")
        var y: Double

        @Guide(description: "Brief explanation of why this point was chosen and what you see in the camera")
        var reason: String
    }

    func call(arguments: Arguments) async throws -> String {
        let tx = Float(arguments.x)
        let ty = Float(arguments.y)
        await MainActor.run {
            NotificationCenter.default.post(
                name: .serverSetNavTarget,
                object: nil,
                userInfo: ["x": tx, "y": ty]
            )
        }
        return "Navigating to (\(String(format: "%.2f", arguments.x)), \(String(format: "%.2f", arguments.y))): \(arguments.reason)"
    }
}

/// Called by the model when it believes the robot has reached the destination.
struct DeclareArrivedTool: Tool {
    let name = "declareArrived"
    let description = "Declare that the robot has arrived at the destination. Only call this when the camera view confirms you are at the goal location."

    @Generable
    struct Arguments {
        @Guide(description: "Brief statement confirming what you see that indicates arrival")
        var message: String
    }

    func call(arguments: Arguments) async throws -> String {
        await MainActor.run {
            NLNavigator.shared.arrivalDeclared = true
        }
        return arguments.message
    }
}

// MARK: - NL Navigator

/// Runs a vision-guided navigation loop on the robot side.
/// Receives a natural language goal from the remote controller, then iteratively
/// prompts the on-device model with the current camera image and grid context
/// to plan and execute navigation steps until arrival or user stop.
@MainActor
final class NLNavigator {

    static let shared = NLNavigator()

    // Set by LiDARViewController at startup
    weak var occupancyGrid: OccupancyGrid?
    weak var arSession: ARSession?

    /// Called with progress/status text for display. nil means hide the status.
    var onStatusChanged: ((String?) -> Void)?

    private(set) var isRunning = false

    /// Set to true by DeclareArrivedTool during a respond() call
    var arrivalDeclared = false

    private var navigationTask: Task<Void, Never>?
    private let ciContext = CIContext()

    private init() {}

    // MARK: - Public API

    func start(command: String) {
        stop()
        isRunning = true
        arrivalDeclared = false
        onStatusChanged?("NL nav: \"\(command)\"")
        navigationTask = Task { [weak self] in
            await self?.runLoop(goal: command)
        }
    }

    func stop() {
        navigationTask?.cancel()
        navigationTask = nil
        isRunning = false
        arrivalDeclared = false
        PathNavigator.shared.stopNavigation()
        onStatusChanged?(nil)
    }

    // MARK: - Navigation Loop

    private func runLoop(goal: String) async {
        let session = LanguageModelSession(
            tools: [NavigateToGridPointTool(), DeclareArrivedTool()],
            instructions: """
            You are the navigation brain of a robotic car equipped with a LiDAR/camera iPhone. \
            Your job is to navigate the car to the described destination by issuing a series of \
            short navigation moves.

            At each step you receive:
            - The navigation goal (natural language description)
            - Your current world position (x, y in meters) and heading (degrees, 0 = +Y, 90 = +X)
            - A summary of the occupancy grid near you
            - The current camera view labeled "camera"

            Rules:
            - Call navigateToGridPoint with a point 1–3 meters ahead, in the direction of the goal.
            - Use the camera image to identify the destination (furniture, doors, rooms, etc.).
            - After each move you will be re-evaluated with a fresh camera image.
            - When the camera view shows you are at the destination, call declareArrived.
            - If unsure, make a reasonable forward progress toward where you think the goal is.
            """
        )

        var step = 0
        let maxSteps = 20

        while !Task.isCancelled && step < maxSteps {
            guard let grid = occupancyGrid else { break }
            let pos = grid.devicePosition
            let gridDesc = buildGridDescription(grid: grid)
            let cameraImage = captureCurrentCameraFrame()

            let promptText = """
            Goal: \(goal)
            Step: \(step + 1) of \(maxSteps)
            Position: x=\(String(format: "%.2f", pos.x))m, y=\(String(format: "%.2f", pos.y))m
            Heading: \(String(format: "%.0f", pos.heading * 180 / .pi))°
            \(gridDesc)

            Look at the camera image and decide your next navigation action.
            """

            onStatusChanged?("NL step \(step + 1): thinking…")

            do {
                if #available(iOS 27, *), let cgImage = cameraImage {
                    try await session.respond {
                        promptText
                        Attachment(cgImage).label("camera")
                    }
                } else {
                    try await session.respond(to: promptText)
                }
            } catch {
                onStatusChanged?("NL nav error: \(error.localizedDescription)")
                break
            }

            if arrivalDeclared || Task.isCancelled { break }

            // Wait for PathNavigator to start and complete navigation
            onStatusChanged?("NL step \(step + 1): navigating…")
            await waitForNavigation()

            if Task.isCancelled { break }
            step += 1
        }

        isRunning = false
        if arrivalDeclared {
            onStatusChanged?("NL nav: arrived!")
        } else if Task.isCancelled {
            onStatusChanged?(nil)
        } else {
            onStatusChanged?("NL nav: complete")
        }
        arrivalDeclared = false
    }

    // MARK: - Helpers

    private func captureCurrentCameraFrame() -> CGImage? {
        guard let pixelBuffer = arSession?.currentFrame?.capturedImage else { return nil }
        let ciImage = CIImage(cvPixelBuffer: pixelBuffer)
        return ciContext.createCGImage(ciImage, from: ciImage.extent)
    }

    private func buildGridDescription(grid: OccupancyGrid) -> String {
        let pos = grid.devicePosition
        let heading = pos.heading
        let step: Float = 0.15
        let reach: Float = 2.5

        var forwardFree = 0
        var forwardBlocked = false
        var dist: Float = step
        while dist <= reach {
            let wx = pos.x + sin(heading) * dist
            let wy = pos.y + cos(heading) * dist
            let state = grid.getState(worldX: wx, worldY: wy)
            if state == .occupied { forwardBlocked = true; break }
            if state == .free { forwardFree += 1 }
            dist += step
        }

        let obstacleStatus = forwardBlocked ? "obstacle \(String(format: "%.1f", dist))m ahead" : "path clear"
        return "Grid: \(obstacleStatus), \(forwardFree) free cells forward, total mapped: \(grid.freeCount) free + \(grid.occupiedCount) occupied"
    }

    private func waitForNavigation() async {
        // Give LiDARViewController time to plan and start the path
        let deadline = Date().addingTimeInterval(3.0)
        while PathNavigator.shared.state == .idle, Date() < deadline {
            if Task.isCancelled { return }
            try? await Task.sleep(for: .milliseconds(100))
        }
        // Now wait for the active navigation to finish
        while PathNavigator.shared.state == .navigating || PathNavigator.shared.state == .paused {
            if Task.isCancelled { return }
            try? await Task.sleep(for: .milliseconds(200))
        }
    }
}
