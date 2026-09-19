import Foundation

@main
struct ServoTelemetryChecks {
    static func main() throws {
        let root = URL(fileURLWithPath: #filePath).deletingLastPathComponent().deletingLastPathComponent()
        let view = try String(contentsOf: root.appendingPathComponent("RoboCar/Views/ServoControlListView.swift"), encoding: .utf8)
        let manager = try String(contentsOf: root.appendingPathComponent("RoboCar/BLE/ESP32BLEManager.swift"), encoding: .utf8)

        func section(_ source: String, from start: String, to end: String) -> String {
            guard let startRange = source.range(of: start),
                  let endRange = source.range(of: end, range: startRange.upperBound..<source.endIndex) else {
                fatalError("Production method boundary not found: \(start)")
            }
            return String(source[startRange.lowerBound..<endRange.lowerBound])
        }

        let summary = section(view, from: "    private func updatePositionSummary()", to: "    func setControlsEnabled")
        let observer = section(manager, from: "    func addServoListObserver(", to: "    @discardableResult")
        let axis = section(manager, from: "struct ServoAxisStatus {", to: "/// Manages the BLE connection")
        let harness = #"""
        import Foundation

        \#(axis)

        final class Label {
            var text: String?
        }

        final class Card {
            var liveRawPosition: UInt16?
            var telemetryFresh = false
            var latestAxisStatus: ServoAxisStatus?
            let positionLabel = Label()
            let summaryLabel = Label()

        \#(summary)

            func render() { updatePositionSummary() }
        }

        final class ListSource {
            var latestServoIDs: [UInt8] = []
            var servoListObservers: [UUID: ([UInt8]) -> Void] = [:]

        \#(observer)
        }

        func status(angle: Double?, ticks: Int32, moving: Bool) -> ServoAxisStatus {
            ServoAxisStatus(id: 1, isTracked: true, hasMin: false, hasMax: false,
                            hasZero: angle != nil, isMoving: moving, isError: false,
                            cumulativeTicks: ticks, angleDegrees: angle, percent: nil,
                            totalDegrees: Double(ticks) * 360 / 4096)
        }

        let card = Card()
        card.telemetryFresh = true
        for (angle, ticks) in [(450.0, Int32(5120)), (-450.0, Int32(-5120))] {
            card.latestAxisStatus = status(angle: angle, ticks: ticks, moving: true)
            card.liveRawPosition = 0
            card.render()
            let jogging = card.positionLabel.text
            precondition(jogging?.hasPrefix(String(format: "%.1f", angle)) == true)
            card.latestAxisStatus = status(angle: angle, ticks: ticks, moving: false)
            card.liveRawPosition = 3072
            card.render()
            precondition(card.positionLabel.text == jogging, "Raw mode changes altered the tracked angle")
            precondition(card.summaryLabel.text?.contains("Raw: 3072") == true)
            precondition(card.summaryLabel.text?.hasPrefix(String(format: "%.1f", angle)) == true)
        }
        card.latestAxisStatus = status(angle: 90, ticks: 1024, moving: true)
        card.render()
        precondition(card.positionLabel.text?.hasPrefix("90.0") == true, "Real tracked movement was not displayed")
        card.telemetryFresh = false
        card.render()
        let stale = card.positionLabel.text
        card.liveRawPosition = 2048
        card.render()
        precondition(card.positionLabel.text == stale)
        precondition(stale?.contains("90.0") == false, "Stale angle remained visible")
        card.telemetryFresh = true
        card.latestAxisStatus = status(angle: nil, ticks: 0, moving: false)
        card.render()
        precondition(card.positionLabel.text?.contains("0.0") == false, "Unknown angle fell back to raw position")
        print("PASS: tracked motion, positive/negative turns, stop continuity, raw diagnostics, stale and missing angle")

        let source = ListSource()
        source.latestServoIDs = [1, 7]
        var local: [[UInt8]] = []
        let localID = source.addServoListObserver { local.append($0) }
        precondition(local == [[1, 7]], "Late observer did not receive cached IDs")
        source.latestServoIDs = []
        var remote: [[UInt8]] = []
        let remoteID = source.addServoListObserver { remote.append($0) }
        precondition(remote == [[]], "Empty cache replay retained old IDs")
        source.latestServoIDs = [2]
        source.servoListObservers.values.forEach { $0(source.latestServoIDs) }
        precondition(local == [[1, 7], [2]] && remote == [[], [2]])
        source.servoListObservers.removeValue(forKey: localID)
        source.servoListObservers.removeValue(forKey: remoteID)
        precondition(source.servoListObservers.isEmpty)
        print("PASS: late observer replay, empty cache, subsequent list updates, observer removal")
        """#

        let temporary = FileManager.default.temporaryDirectory.appendingPathComponent(UUID().uuidString)
        try FileManager.default.createDirectory(at: temporary, withIntermediateDirectories: true)
        defer { try? FileManager.default.removeItem(at: temporary) }
        let script = temporary.appendingPathComponent("checks.swift")
        try harness.write(to: script, atomically: true, encoding: .utf8)
        let process = Process()
        process.executableURL = URL(fileURLWithPath: "/usr/bin/xcrun")
        process.arguments = ["swift", script.path]
        try process.run()
        process.waitUntilExit()
        precondition(process.terminationStatus == 0, "Servo telemetry regression checks failed")
    }
}