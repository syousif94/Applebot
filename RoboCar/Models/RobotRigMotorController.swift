import UIKit

final class RobotRigMotorController {
    let commander: ServoCommanding
    var connectionAvailable: () -> Bool
    var robotIdentity: () -> String?
    var connectionIdentity: () -> String?
    var onChange: (() -> Void)?
    private(set) var servoIDs: [UInt8] = []
    private(set) var isArmed = false
    private(set) var status: String?
    private var states: [UInt8: (ServoState, Date)] = [:]
    private var axes: [UInt8: (ServoAxisStatus, Date)] = [:]
    private var binding: RobotRigMotorBinding?
    private var active = false
    private var visible = false
    private var timer: Timer?
    private var cursor = 0
    private var ownedMotion: UInt8?
    private var sentAt: Date?
    private var armedConnection: String?
    private var observerTokens: [UUID] = []
    private var notifications: [NSObjectProtocol] = []
    private weak var ble: ESP32BLEManager?

    init(commander: ServoCommanding, connectionAvailable: @escaping () -> Bool,
         robotIdentity: @escaping () -> String? = { "test" },
         connectionIdentity: @escaping () -> String? = { "test-session" }) {
        self.commander = commander
        self.connectionAvailable = connectionAvailable
        self.robotIdentity = robotIdentity
        self.connectionIdentity = connectionIdentity
        notifications.append(NotificationCenter.default.addObserver(forName: UIApplication.willResignActiveNotification, object: nil, queue: .main) { [weak self] _ in
            MainActor.assumeIsolated { self?.suspend(); self?.onChange?() }
        })
        notifications.append(NotificationCenter.default.addObserver(forName: UIApplication.didBecomeActiveNotification, object: nil, queue: .main) { [weak self] _ in
            MainActor.assumeIsolated {
                if self?.visible == true { self?.setActive(true) }
            }
        })
    }

    static func local() -> RobotRigMotorController {
        let ble = ESP32BLEManager.shared
        let result = RobotRigMotorController(commander: ble, connectionAvailable: { ble.connectionState == .connected },
                             robotIdentity: { ble.connectedPeripheralID.map { "ble:\($0)" } },
                             connectionIdentity: { ble.connectionGeneration.uuidString })
        result.ble = ble
        result.observerTokens = [
            ble.addServoListObserver { [weak result] ids in result?.updateIDs(ids) },
            ble.addServoStateObserver { [weak result] state in result?.receive(state) },
            ble.addServoAxisStatusObserver { [weak result] axis in result?.receive(axis) }
        ]
        return result
    }

    deinit {
        timer?.invalidate()
        for token in notifications { NotificationCenter.default.removeObserver(token) }
        let tokens = observerTokens
        let manager = ble
        Task { @MainActor in for token in tokens { manager?.removeServoObserver(token) } }
    }

    func updateIDs(_ ids: [UInt8]) {
        servoIDs = Array(Set(ids.filter { (1...253).contains($0) })).sorted()
        if let binding, !servoIDs.contains(UInt8(binding.servoID)) { disarm() }
        onChange?()
    }

    func receive(_ state: ServoState) {
        guard active else { return }
        states[state.id] = (state, Date())
        checkArmed()
        onChange?()
    }

    func receive(_ axis: ServoAxisStatus) {
        guard active else { return }
        axes[axis.id] = (axis, Date())
        if ownedMotion == axis.id, !axis.isMoving, let sentAt, Date().timeIntervalSince(sentAt) > 1 {
            ownedMotion = nil
            self.sentAt = nil
        }
        checkArmed()
        onChange?()
    }

    func setActive(_ value: Bool) {
        visible = value
        suspend()
        active = value
        guard value else { return }
        let timer = Timer(timeInterval: 0.25, repeats: true) { [weak self] _ in
            MainActor.assumeIsolated { self?.poll() }
        }
        self.timer = timer
        RunLoop.main.add(timer, forMode: .common)
        poll()
    }

    private func suspend() {
        active = false
        timer?.invalidate()
        timer = nil
        disarm()
        states.removeAll()
        axes.removeAll()
    }

    func connectionChanged() {
        disarm()
        states.removeAll()
        axes.removeAll()
        onChange?()
    }

    private func poll() {
        guard active, UIApplication.shared.applicationState == .active, connectionAvailable() else {
            disarm()
            states.removeAll()
            axes.removeAll()
            onChange?()
            return
        }
        checkArmed()
        let ids = servoIDs
        guard !ids.isEmpty else { return }
        let id = ids[(cursor / 2) % ids.count]
        if cursor % 2 == 0 { commander.refreshServoState(id: id) }
        else { commander.refreshServoAxisStatus(id: id) }
        cursor = (cursor + 1) % max(2, ids.count * 2)
    }

    private var freshnessLimit: TimeInterval { max(3, Double(servoIDs.count) * 0.75) }

    private func validateLive(_ binding: RobotRigMotorBinding) throws {
        try binding.validate()
        guard active, UIApplication.shared.applicationState == .active, connectionAvailable() else { throw RobotRigError.invalid("Robot connection is not active") }
        guard let robot = robotIdentity(), binding.robotID == robot else { throw RobotRigError.invalid("Rebind this group to the connected robot before enabling Live") }
        if isArmed, armedConnection != connectionIdentity() { throw RobotRigError.invalid("Connection changed; re-enable Live after checking calibration") }
        guard binding.mode == .multiTurn else { throw RobotRigError.invalid("Position mode is preview-only until a reliable stop path is available") }
        let id = UInt8(binding.servoID)
        guard servoIDs.contains(id), let (state, stateTime) = states[id], let (axis, axisTime) = axes[id],
              Date().timeIntervalSince(stateTime) < freshnessLimit, Date().timeIntervalSince(axisTime) < freshnessLimit else {
            throw RobotRigError.invalid("Waiting for fresh motor telemetry")
        }
        guard !state.isReadFailure, state.error == 0, state.torqueEnabled == true,
              axis.isTracked, axis.hasZero, axis.hasMin, axis.hasMax, !axis.isError,
              let angle = axis.angleDegrees, angle.isFinite, axis.percent != nil else {
            throw RobotRigError.invalid("Motor requires torque, tracking, zero and both travel marks; configure these in Servos")
        }
    }

    func arm(_ binding: RobotRigMotorBinding) throws {
        try validateLive(binding)
        guard axes[UInt8(binding.servoID)]?.0.isMoving == false else { throw RobotRigError.invalid("Wait for the motor to stop before arming") }
        self.binding = binding
        armedConnection = connectionIdentity()
        isArmed = true
        status = "Live: servo \(binding.servoID)"
        onChange?()
    }

    func measuredAngle(_ binding: RobotRigMotorBinding) -> Double? {
        let id = UInt8(clamping: binding.servoID)
        if binding.mode == .multiTurn {
            guard let (axis, time) = axes[id], Date().timeIntervalSince(time) < freshnessLimit,
                  axis.isTracked, axis.hasZero, !axis.isError, let angle = axis.angleDegrees else { return nil }
            return binding.jointAngle(for: angle)
        }
        guard let (state, time) = states[id], Date().timeIntervalSince(time) < freshnessLimit, !state.isReadFailure else { return nil }
        return binding.jointAngle(for: Double(state.position) * 360 / 4096)
    }

    func move(_ binding: RobotRigMotorBinding, to target: Double) throws {
        guard isArmed, self.binding == binding else { throw RobotRigError.invalid("Enable Live control first") }
        try validateLive(binding)
        guard target.isFinite, (binding.minimum...binding.maximum).contains(target) else { throw RobotRigError.invalid("Target exceeds joint limits") }
        guard ownedMotion == nil else { throw RobotRigError.invalid("Wait for the previous move or press Stop") }
        ownedMotion = UInt8(binding.servoID)
        sentAt = Date()
        commander.moveServoToAngle(id: UInt8(binding.servoID), degrees: binding.motorAngle(for: target))
    }

    func disarm() {
        if let id = ownedMotion, connectionAvailable(), armedConnection == connectionIdentity() { commander.stopServoMotion(id: id) }
        ownedMotion = nil
        sentAt = nil
        binding = nil
        armedConnection = nil
        isArmed = false
        status = "Preview"
    }

    private func checkArmed() {
        guard isArmed, let binding else { return }
        do { try validateLive(binding) }
        catch { disarm(); status = error.localizedDescription }
    }
}