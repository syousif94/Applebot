//
//  MotorCalibrator.swift
//  RoboCar
//
//  Created by Sammy Yousif on 2/17/26.
//

import Foundation
import simd

/// Records a single measurement of motor power → observed velocity/turn-rate
struct CalibrationSample: CustomStringConvertible {
    /// Left-side motor power (-100…100)
    let leftPower: Int8
    /// Right-side motor power (-100…100)
    let rightPower: Int8
    /// Measured linear speed (m/s) — positive = forward in device frame
    let linearSpeed: Float
    /// Measured angular velocity (rad/s) — positive = turning right
    let angularVelocity: Float
    /// Duration of the measurement window (seconds)
    let duration: TimeInterval
    /// Total distance traveled (m)
    let distance: Float
    /// Total angle turned (rad)
    let angleTurned: Float
    
    var description: String {
        String(format: "L:%+4d R:%+4d → %.3f m/s, %.1f°/s (%.2fs, %.3fm, %.1f°)",
               Int(leftPower), Int(rightPower),
               linearSpeed, angularVelocity * 180 / .pi,
               duration, distance, angleTurned * 180 / .pi)
    }
}

/// Result summary for a full calibration run
struct CalibrationResult: CustomStringConvertible {
    /// All individual samples
    let samples: [CalibrationSample]
    
    /// Estimated meters-per-second per unit of motor power (forward)
    let speedPerPowerUnit: Float
    
    /// Estimated radians-per-second when doing differential turn at power 100
    let maxTurnRate: Float
    
    /// Left-side speed bias (>1 means left side is faster)
    let leftRightBias: Float
    
    /// Date of calibration
    let timestamp: Date
    
    var description: String {
        var lines = ["═══ Motor Calibration Results ═══"]
        lines.append(String(format: "Speed/power unit: %.4f m/s per unit", speedPerPowerUnit))
        lines.append(String(format: "Max turn rate:    %.1f °/s (at power 100)", maxTurnRate * 180 / .pi))
        lines.append(String(format: "Left/Right bias:  %.3f (1.0 = balanced)", leftRightBias))
        lines.append("─── Samples ───")
        for s in samples {
            lines.append("  \(s)")
        }
        lines.append("════════════════════════════════")
        return lines.joined(separator: "\n")
    }
}

// MARK: - Motor Calibrator

/// Runs a calibration sequence to measure the relationship between
/// motor power commands and actual speed/direction using ARKit tracking.
///
/// Procedure:
///   1. Disable obstacle avoidance temporarily
///   2. Run a series of power levels for both straight and turning
///   3. At each level, drive for a short time, sample position/heading
///   4. Compute velocity and turn rate from position deltas
///   5. Restore obstacle avoidance
class MotorCalibrator {
    
    // MARK: - Singleton
    
    static let shared = MotorCalibrator()
    
    // MARK: - Configuration
    
    /// Power levels to test for straight-line driving
    let straightPowers: [Int8] = [25, 35, 50, 65, 80, 100]
    
    /// Power differentials to test for turning (left, right)
    let turnTests: [(left: Int8, right: Int8)] = [
        ( 40, -40),   // spin right at 40
        (-40,  40),   // spin left at 40
        ( 70, -70),   // spin right at 70
        (-70,  70),   // spin left at 70
        ( 50,  20),   // arc right
        ( 20,  50),   // arc left
        ( 80,  40),   // wide arc right
        ( 40,  80),   // wide arc left
    ]
    
    /// How long to drive at each power level (seconds)
    let sampleDuration: TimeInterval = 1.5
    
    /// Settle time after stopping before next test (seconds)
    let settleTime: TimeInterval = 0.8
    
    /// Sampling rate for position tracking (Hz)
    let samplingRate: Double = 30.0
    
    // MARK: - State
    
    private(set) var isCalibrating = false
    
    /// Most recent calibration result
    private(set) var lastResult: CalibrationResult?
    
    /// Callback for status updates
    var onStatusUpdate: ((String) -> Void)?
    
    /// Callback for progress (0.0 – 1.0)
    var onProgress: ((Float) -> Void)?
    
    /// Reference to occupancy grid for position tracking
    weak var occupancyGrid: OccupancyGrid?
    
    private init() {}
    
    // MARK: - Public API
    
    /// Run the full calibration sequence. Returns a human-readable summary.
    func runCalibration() async throws -> String {
        guard !isCalibrating else {
            return "Calibration is already in progress."
        }
        
        guard let grid = occupancyGrid ?? ObstacleDetector.shared.occupancyGrid else {
            return "No occupancy grid available. Make sure LiDAR is running."
        }
        
        isCalibrating = true
        defer { isCalibrating = false }
        
        // Temporarily disable obstacle avoidance during calibration
        let wasEnabled = ObstacleDetector.shared.isEnabled
        ObstacleDetector.shared.isEnabled = false
        defer { ObstacleDetector.shared.isEnabled = wasEnabled }
        
        var allSamples: [CalibrationSample] = []
        let totalTests = straightPowers.count + turnTests.count
        var completedTests = 0
        
        log("🔧 Starting motor calibration (\(totalTests) tests)…")
        log("⚠️ Obstacle avoidance disabled during calibration")
        log("   Make sure the car has space to move!")
        
        // Wait a moment before starting
        try await Task.sleep(for: .seconds(1.0))
        
        // ── Phase 1: Straight-line tests ──
        log("── Phase 1: Straight-line speed tests ──")
        
        for power in straightPowers {
            try Task.checkCancellation()
            
            log("  Testing forward power: \(power)")
            
            let sample = try await measureMotors(
                left: power, right: power, grid: grid
            )
            allSamples.append(sample)
            log("  → \(sample)")
            
            completedTests += 1
            onProgress?(Float(completedTests) / Float(totalTests))
        }
        
        // ── Phase 2: Turn tests ──
        log("── Phase 2: Turn / arc tests ──")
        
        for test in turnTests {
            try Task.checkCancellation()
            
            log("  Testing L:\(test.left) R:\(test.right)")
            
            let sample = try await measureMotors(
                left: test.left, right: test.right, grid: grid
            )
            allSamples.append(sample)
            log("  → \(sample)")
            
            completedTests += 1
            onProgress?(Float(completedTests) / Float(totalTests))
        }
        
        // ── Build calibration result ──
        let result = buildResult(from: allSamples)
        lastResult = result
        
        log("\n\(result)")
        
        return formatSummary(result)
    }
    
    /// Cancel an in-progress calibration
    func cancelCalibration() {
        isCalibrating = false
        ESP32BLEManager.shared.stopAll()
        log("🛑 Calibration cancelled")
    }
    
    // MARK: - Measurement
    
    /// Drive at the given motor powers, sample position, compute velocity.
    private func measureMotors(
        left: Int8, right: Int8, grid: OccupancyGrid
    ) async throws -> CalibrationSample {
        
        // Record start position
        let startPos = grid.devicePosition
        let startTime = Date()
        
        // Collect position samples during the drive
        var positions: [(x: Float, y: Float, heading: Float, time: TimeInterval)] = []
        positions.append((startPos.x, startPos.y, startPos.heading, 0))
        
        // Start motors: A,C = left side; B,D = right side
        await MainActor.run {
            ESP32BLEManager.shared.setAllMotors(a: left, b: right, c: left, d: right)
        }
        
        let sampleInterval = 1.0 / samplingRate
        let sampleCount = Int(sampleDuration * samplingRate)
        
        for _ in 0..<sampleCount {
            try Task.checkCancellation()
            try await Task.sleep(for: .seconds(sampleInterval))
            
            let pos = grid.devicePosition
            let elapsed = Date().timeIntervalSince(startTime)
            positions.append((pos.x, pos.y, pos.heading, elapsed))
        }
        
        // Stop motors
        await MainActor.run {
            ESP32BLEManager.shared.stopAll()
        }
        
        // Wait for the car to settle
        try await Task.sleep(for: .seconds(settleTime))
        
        // ── Compute metrics from position samples ──
        
        let endPos = grid.devicePosition
        let totalDuration = Date().timeIntervalSince(startTime) - settleTime
        
        // Total displacement
        let dx = endPos.x - startPos.x
        let dy = endPos.y - startPos.y
        let totalDistance = sqrtf(dx * dx + dy * dy)
        
        // Total angle turned (unwrapped)
        var totalAngle: Float = 0
        for i in 1..<positions.count {
            var dh = positions[i].heading - positions[i - 1].heading
            while dh > .pi  { dh -= 2 * .pi }
            while dh < -.pi { dh += 2 * .pi }
            totalAngle += dh
        }
        
        // Compute instantaneous velocities from the middle portion
        // (skip first 20% and last 10% to avoid ramp-up/deceleration)
        let skipStart = max(1, positions.count / 5)
        let skipEnd = max(skipStart + 1, positions.count - positions.count / 10)
        
        var speeds: [Float] = []
        var turnRates: [Float] = []
        
        for i in skipStart..<skipEnd {
            let dt = Float(positions[i].time - positions[i - 1].time)
            guard dt > 0.001 else { continue }
            
            let vx = (positions[i].x - positions[i - 1].x) / dt
            let vy = (positions[i].y - positions[i - 1].y) / dt
            
            // Speed in the forward direction of the device at that instant
            let heading = positions[i].heading
            let forwardX = sinf(heading)
            let forwardY = cosf(heading)
            let forwardSpeed = vx * forwardX + vy * forwardY
            
            speeds.append(forwardSpeed)
            
            // Angular velocity
            var dh = positions[i].heading - positions[i - 1].heading
            while dh > .pi  { dh -= 2 * .pi }
            while dh < -.pi { dh += 2 * .pi }
            turnRates.append(dh / dt)
        }
        
        // Use median to be robust against outlier frames
        let medianSpeed = median(speeds)
        let medianTurnRate = median(turnRates)
        
        return CalibrationSample(
            leftPower: left,
            rightPower: right,
            linearSpeed: medianSpeed,
            angularVelocity: medianTurnRate,
            duration: totalDuration,
            distance: totalDistance,
            angleTurned: totalAngle
        )
    }
    
    // MARK: - Analysis
    
    /// Build a CalibrationResult from the raw samples
    private func buildResult(from samples: [CalibrationSample]) -> CalibrationResult {
        // Extract straight-line samples for speed-per-power calculation
        let straightSamples = samples.filter { $0.leftPower == $0.rightPower && $0.leftPower > 0 }
        
        var speedPerUnit: Float = 0
        if !straightSamples.isEmpty {
            // Linear regression: speed = k * power
            var sumPV: Float = 0  // sum of power * velocity
            var sumPP: Float = 0  // sum of power * power
            for s in straightSamples {
                let p = Float(s.leftPower)
                sumPV += p * s.linearSpeed
                sumPP += p * p
            }
            speedPerUnit = sumPP > 0 ? sumPV / sumPP : 0
        }
        
        // Estimate max turn rate from spin-in-place tests
        let spinSamples = samples.filter {
            $0.leftPower == -$0.rightPower && $0.leftPower != 0
        }
        var maxTurnRate: Float = 0
        if !spinSamples.isEmpty {
            // Find the highest-power spin and extrapolate to power 100
            var sumTR_P: Float = 0
            var sumP_P: Float = 0
            for s in spinSamples {
                let p = Float(abs(Int(s.leftPower)))
                let tr = fabsf(s.angularVelocity)
                sumTR_P += p * tr
                sumP_P += p * p
            }
            let turnRatePerUnit = sumP_P > 0 ? sumTR_P / sumP_P : 0
            maxTurnRate = turnRatePerUnit * 100
        }
        
        // Compute left/right bias from arc tests
        // If car curves right when both sides are set equal, left side is faster
        var leftRightBias: Float = 1.0
        if !straightSamples.isEmpty {
            var totalDrift: Float = 0
            var count: Float = 0
            for s in straightSamples {
                // Positive angular velocity during straight driving = curving right = left faster
                totalDrift += s.angularVelocity
                count += 1
            }
            let avgDrift = totalDrift / max(count, 1)
            // Express as a ratio: if drift is positive (curving right), left is faster
            // A rough conversion: bias ≈ 1 + drift / (2 * speed)
            let avgSpeed = straightSamples.map { fabsf($0.linearSpeed) }.reduce(0, +) / max(count, 1)
            if avgSpeed > 0.01 {
                leftRightBias = 1.0 + avgDrift / (2 * avgSpeed)
            }
        }
        
        return CalibrationResult(
            samples: samples,
            speedPerPowerUnit: speedPerUnit,
            maxTurnRate: maxTurnRate,
            leftRightBias: leftRightBias,
            timestamp: Date()
        )
    }
    
    // MARK: - Formatting
    
    private func formatSummary(_ result: CalibrationResult) -> String {
        var lines: [String] = []
        lines.append("Motor calibration complete!")
        lines.append("")
        
        // Straight-line results
        let straightSamples = result.samples.filter { $0.leftPower == $0.rightPower && $0.leftPower > 0 }
        if !straightSamples.isEmpty {
            lines.append("Forward speed by power level:")
            for s in straightSamples {
                let cmPerSec = s.linearSpeed * 100
                lines.append(String(format: "  Power %3d: %.1f cm/s", Int(s.leftPower), cmPerSec))
            }
            lines.append("")
        }
        
        // Turn results
        let spinSamples = result.samples.filter { $0.leftPower == -$0.rightPower }
        if !spinSamples.isEmpty {
            lines.append("Spin-in-place turn rates:")
            for s in spinSamples {
                let degPerSec = fabsf(s.angularVelocity) * 180 / .pi
                let direction = s.angularVelocity > 0 ? "right" : "left"
                lines.append(String(format: "  Power %3d: %.0f°/s (%@)",
                                    abs(Int(s.leftPower)), degPerSec, direction))
            }
            lines.append("")
        }
        
        // Arc results
        let arcSamples = result.samples.filter {
            $0.leftPower != $0.rightPower &&
            $0.leftPower != -$0.rightPower &&
            ($0.leftPower != 0 && $0.rightPower != 0)
        }
        if !arcSamples.isEmpty {
            lines.append("Arc driving (mixed power):")
            for s in arcSamples {
                let cmPerSec = s.linearSpeed * 100
                let degPerSec = s.angularVelocity * 180 / .pi
                lines.append(String(format: "  L:%+4d R:%+4d → %.1f cm/s, %.1f°/s",
                                    Int(s.leftPower), Int(s.rightPower), cmPerSec, degPerSec))
            }
            lines.append("")
        }
        
        lines.append(String(format: "Speed per power unit: %.4f m/s", result.speedPerPowerUnit))
        lines.append(String(format: "Max spin rate (at 100): %.0f°/s", result.maxTurnRate * 180 / .pi))
        lines.append(String(format: "Left/Right bias: %.3f (1.0 = balanced)", result.leftRightBias))
        
        if fabsf(result.leftRightBias - 1.0) > 0.05 {
            if result.leftRightBias > 1.0 {
                lines.append("⚠️ Left side is \(Int((result.leftRightBias - 1.0) * 100))% faster — car pulls right")
            } else {
                lines.append("⚠️ Right side is \(Int((1.0 - result.leftRightBias) * 100))% faster — car pulls left")
            }
        }
        
        return lines.joined(separator: "\n")
    }
    
    // MARK: - Helpers
    
    private func median(_ values: [Float]) -> Float {
        guard !values.isEmpty else { return 0 }
        let sorted = values.sorted()
        let mid = sorted.count / 2
        if sorted.count % 2 == 0 {
            return (sorted[mid - 1] + sorted[mid]) / 2
        }
        return sorted[mid]
    }
    
    private func log(_ message: String) {
        print("[Calibrate] \(message)")
        onStatusUpdate?(message)
    }
}
