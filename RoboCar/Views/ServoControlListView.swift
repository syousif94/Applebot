//
//  ServoControlListView.swift
//  RoboCar
//
//  Collapsible per-servo control cards shared by the local BLE control
//  panel and the remote controller. Commands go through ServoCommanding
//  so the same UI can drive BLE directly or relay over the network.
//

import UIKit

/// Xcode 27 beta links UIFontTextStyle constants against AppKit on Catalyst,
/// but macOS 26 does not export them there. Avoid those symbol references on
/// Catalyst while retaining Dynamic Type scaling and the normal iOS styles.
private enum ServoFont {
    case position, caption, button, holdButton

    var font: UIFont {
        #if targetEnvironment(macCatalyst)
        let base: UIFont
        switch self {
        case .position: base = .monospacedSystemFont(ofSize: 20, weight: .semibold)
        case .caption: base = .systemFont(ofSize: 12)
        case .button: base = .systemFont(ofSize: 14, weight: .semibold)
        case .holdButton: base = .systemFont(ofSize: 17, weight: .semibold)
        }
        return UIFontMetrics.default.scaledFont(for: base)
        #else
        switch self {
        case .position:
            return UIFontMetrics(forTextStyle: .title3).scaledFont(for: .monospacedSystemFont(ofSize: 20, weight: .semibold))
        case .caption:
            return .preferredFont(forTextStyle: .caption1)
        case .button:
            return UIFontMetrics(forTextStyle: .subheadline).scaledFont(for: .systemFont(ofSize: 14, weight: .semibold))
        case .holdButton:
            return .preferredFont(forTextStyle: .headline)
        }
        #endif
    }
}

// MARK: - Command abstraction

/// Everything the servo UI can ask a robot to do.
protocol ServoCommanding: AnyObject {
    func rescanServos(from: UInt8, to: UInt8)
    func moveServo(id: UInt8, position: UInt16, speed: UInt16)
    func moveDiscoveredServos(position: UInt16, speed: UInt16, acceleration: UInt8)
    func setServoTorque(id: UInt8, enabled: Bool)
    func changeServoID(currentID: UInt8, newID: UInt8)
    func calibrateServoCenter(id: UInt8)
    func driveServoWheel(id: UInt8, speed: Int16, acceleration: UInt8)
    func setServoPositionMode(id: UInt8)
    func refreshServoState(id: UInt8)
    func trackServoAxis(id: UInt8)
    func jogServo(id: UInt8, degrees: Double)
    func beginServoJog(id: UInt8, direction: ServoJogDirection, speed: UInt16)
    func stopServoMotion(id: UInt8)
    func markServoTravel(id: UInt8, _ mark: ServoTravelMark)
    func moveServoToPercent(id: UInt8, percent: Double, speed: UInt16)
    func moveServoToAngle(id: UInt8, degrees: Double)
    func refreshServoAxisStatus(id: UInt8)
}

extension ESP32BLEManager: ServoCommanding {}

// MARK: - List view

/// Vertical list of collapsible servo cards. Feed it servo IDs, live
/// positions, states, and axis statuses; it renders and dispatches commands.
final class ServoControlListView: UIView {

    weak var commander: ServoCommanding?

    /// Status line feedback ("Marked min", errors, …)
    var onStatus: ((String, Bool) -> Void)?

    /// Host-presented confirmation for destructive actions (EEPROM writes).
    /// If nil the action runs immediately.
    var presentConfirmation: ((String, String, @escaping () -> Void) -> Void)?

    private let stack = UIStackView()
    private var cards: [UInt8: ServoCardView] = [:]
    private var expandedIDs = Set<UInt8>()
    private var controlsEnabled = true
    private var screenActive = false
    private var appActive = UIApplication.shared.applicationState == .active
    private var refreshTimer: Timer?
    private var refreshIndex = 0
    private var urgentIDs = Set<UInt8>()
    private var lastWasUrgent = false
    private var lastStateRequests: [UInt8: Date] = [:]

    override init(frame: CGRect) {
        super.init(frame: frame)
        stack.translatesAutoresizingMaskIntoConstraints = false
        stack.axis = .vertical
        stack.spacing = 10
        addSubview(stack)
        NSLayoutConstraint.activate([
            stack.topAnchor.constraint(equalTo: topAnchor),
            stack.leadingAnchor.constraint(equalTo: leadingAnchor),
            stack.trailingAnchor.constraint(equalTo: trailingAnchor),
            stack.bottomAnchor.constraint(equalTo: bottomAnchor),
        ])
        showEmptyPlaceholder()
        NotificationCenter.default.addObserver(self, selector: #selector(appWillResignActive), name: UIApplication.willResignActiveNotification, object: nil)
        NotificationCenter.default.addObserver(self, selector: #selector(appDidBecomeActive), name: UIApplication.didBecomeActiveNotification, object: nil)
    }

    required init?(coder: NSCoder) { fatalError("init(coder:) has not been implemented") }

    deinit {
        refreshTimer?.invalidate()
        NotificationCenter.default.removeObserver(self)
    }

    override func didMoveToWindow() {
        super.didMoveToWindow()
        updateRefreshLoop()
    }

    func setScreenActive(_ active: Bool) {
        screenActive = active
        updateRefreshLoop()
    }

    @objc private func appWillResignActive() {
        appActive = false
        updateRefreshLoop()
    }

    @objc private func appDidBecomeActive() {
        appActive = true
        updateRefreshLoop()
    }

    private var canRefresh: Bool { screenActive && appActive && controlsEnabled && window != nil }

    private func updateRefreshLoop() {
        guard canRefresh, !cards.isEmpty else {
            refreshTimer?.invalidate()
            refreshTimer = nil
            urgentIDs.removeAll()
            lastStateRequests.removeAll()
            stopAllJogs()
            cards.values.forEach { $0.invalidateTelemetry() }
            return
        }
        guard refreshTimer == nil else { return }
        cards.values.forEach { $0.invalidateTelemetry() }
        let timer = Timer(timeInterval: 0.25, repeats: true) { [weak self] _ in self?.refreshNextServo() }
        refreshTimer = timer
        // Keep polling during slider/button tracking, without creating a timer per servo.
        RunLoop.main.add(timer, forMode: .common)
        refreshNextServo()
    }

    private func refreshNextServo() {
        guard canRefresh else { return }
        let ids = cards.keys.sorted()
        guard !ids.isEmpty else { return }
        let now = Date()
        // Allow two round-robin passes plus network/readback latency before declaring stale.
        let staleAfter = max(3.0, Double(ids.count) * 1.5)
        cards.values.forEach { $0.checkFreshness(now: now, staleAfter: staleAfter) }
        let id: UInt8
        if !lastWasUrgent, let urgent = urgentIDs.sorted().first {
            id = urgent
            lastWasUrgent = true
        } else {
            id = ids[refreshIndex % ids.count]
            refreshIndex = (refreshIndex + 1) % ids.count
            lastWasUrgent = false
        }
        let urgent = urgentIDs.remove(id) != nil
        guard let card = cards[id] else { return }
        // One request per tick. Health reads are staggered and alternate with axis reads.
        if !urgent, card.isExpanded, now.timeIntervalSince(lastStateRequests[id] ?? .distantPast) >= 3 {
            lastStateRequests[id] = now
            commander?.refreshServoState(id: id)
        } else {
            commander?.refreshServoAxisStatus(id: id)
        }
    }

    // MARK: Updates from the robot

    func setServoIDs(_ ids: [UInt8]) {
        var unique: [UInt8] = []
        for id in ids where !unique.contains(id) { unique.append(id) }
        guard unique != stack.arrangedSubviews.compactMap({ ($0 as? ServoCardView)?.servoID }) else { return }

        for (id, card) in cards where !unique.contains(id) { card.cancelActiveJog() }
        urgentIDs.formIntersection(unique)
        stack.arrangedSubviews.forEach { $0.removeFromSuperview() }
        cards = cards.filter { unique.contains($0.key) }
        expandedIDs.formIntersection(unique)

        guard !unique.isEmpty else {
            showEmptyPlaceholder()
            updateRefreshLoop()
            return
        }
        for id in unique {
            let card = cards[id] ?? makeCard(id: id)
            cards[id] = card
            card.setExpanded(expandedIDs.contains(id), animated: false)
            stack.addArrangedSubview(card)
        }
        setControlsEnabled(controlsEnabled)
    }

    func apply(positions: [UInt8: UInt16]) {
        for (id, pos) in positions {
            cards[id]?.updateLivePosition(pos)
        }
    }

    func apply(state: ServoState) {
        cards[state.id]?.updateState(state)
    }

    func apply(axisStatus: ServoAxisStatus) {
        guard canRefresh else { return } // Cached/hidden-screen values are not fresh telemetry.
        cards[axisStatus.id]?.updateAxisStatus(axisStatus)
    }

    func setControlsEnabled(_ enabled: Bool) {
        controlsEnabled = enabled
        if !enabled { stopAllJogs() }
        cards.values.forEach { $0.setControlsEnabled(enabled) }
        updateRefreshLoop()
    }

    /// Safety: abort any held-down jogs (disconnect, dismiss, background).
    func stopAllJogs() {
        cards.values.forEach { $0.cancelActiveJog() }
    }

    // MARK: Internals

    private func showEmptyPlaceholder() {
        let label = UILabel()
        label.text = "No servos scanned"
        label.font = .systemFont(ofSize: 13, weight: .medium)
        label.textColor = UIColor.white.withAlphaComponent(0.45)
        label.textAlignment = .center
        label.backgroundColor = UIColor(white: 0.17, alpha: 1)
        label.layer.cornerRadius = 8
        label.layer.masksToBounds = true
        label.heightAnchor.constraint(equalToConstant: 52).isActive = true
        stack.addArrangedSubview(label)
    }

    private func makeCard(id: UInt8) -> ServoCardView {
        let card = ServoCardView(id: id)
        card.commander = { [weak self] in self?.commander }
        card.onNeedsRefresh = { [weak self] in
            guard let self, self.canRefresh else { return }
            self.urgentIDs.insert(id)
        }
        card.onStatus = { [weak self] text, isError in self?.onStatus?(text, isError) }
        card.presentConfirmation = { [weak self] title, message, action in
            if let present = self?.presentConfirmation {
                present(title, message, action)
            } else {
                action()
            }
        }
        card.onToggleExpanded = { [weak self] id, expanded in
            guard let self else { return }
            if expanded {
                self.expandedIDs.insert(id)
                self.urgentIDs.insert(id)
                self.lastStateRequests.removeValue(forKey: id)
            } else {
                self.expandedIDs.remove(id)
            }
        }
        return card
    }
}

// MARK: - Card view

/// One servo: a compact header row that expands into full controls on tap.
private final class ServoCardView: UIView {

    let servoID: UInt8

    var commander: (() -> ServoCommanding?) = { nil }
    var onStatus: ((String, Bool) -> Void)?
    var presentConfirmation: ((String, String, @escaping () -> Void) -> Void)?
    var onToggleExpanded: ((UInt8, Bool) -> Void)?
    var onNeedsRefresh: (() -> Void)?

    private(set) var isExpanded = false
    private var isJogging = false
    private var wheelRunning = false
    private var latestAxisStatus: ServoAxisStatus?
    private var lastAxisUpdate: Date?
    private var lastStateUpdate: Date?
    private var lastPositionUpdate: Date?
    private var liveRawPosition: UInt16?
    private var motionPendingUntil: Date?
    private var telemetryFresh = false
    private var controlsEnabled = true
    private var motionActive: Bool {
        isJogging || wheelRunning || motionPendingUntil != nil || latestAxisStatus?.isMoving == true
    }

    // Header
    private let headerButton = UIControl()
    private let titleLabel = UILabel()
    private let summaryLabel = UILabel()
    private let chevron = UIImageView(image: UIImage(systemName: "chevron.down"))

    // Detail
    private let detailStack = UIStackView()
    private lazy var collapsedBottomConstraint = headerButton.bottomAnchor.constraint(equalTo: bottomAnchor)
    private lazy var expandedBottomConstraint = detailStack.bottomAnchor.constraint(equalTo: bottomAnchor, constant: -12)
    private let stateLabel = UILabel()
    private let axisLabel = UILabel()
    private let positionLabel = UILabel()
    private let positionSlider = UISlider()
    private let speedField = UITextField()
    private let jogSpeedField = UITextField()
    private let jogStepField = UITextField()
    private let rawPositionLabel = UILabel()
    private let targetPositionLabel = UILabel()
    private let limitsLabel = UILabel()
    private let advancedStack = UIStackView()
    private lazy var advancedButton = makeButton("Advanced ▾", color: UIColor(white: 0.22, alpha: 1), action: #selector(toggleAdvanced))
    private lazy var markMinButton = makeButton("Set minimum here", color: .systemBrown, action: #selector(markMinTapped))
    private lazy var markMaxButton = makeButton("Set maximum here", color: .systemBrown, action: #selector(markMaxTapped))
    private let wheelSpeedField = UITextField()
    private let newIDField = UITextField()
    private lazy var torqueOnButton = makeButton("Torque On", color: .clear, action: #selector(torqueOnTapped))
    private lazy var torqueOffButton = makeButton("Torque Off", color: .clear, action: #selector(torqueOffTapped))

    private var suppressLiveSliderUpdates: Bool { positionSlider.isTracking }

    init(id: UInt8) {
        self.servoID = id
        super.init(frame: .zero)
        backgroundColor = UIColor(white: 0.16, alpha: 1)
        layer.cornerRadius = 10
        layer.borderWidth = 1
        layer.borderColor = UIColor.white.withAlphaComponent(0.08).cgColor
        buildHeader()
        buildDetail()
    }

    required init?(coder: NSCoder) { fatalError("init(coder:) has not been implemented") }

    // MARK: Public updates

    func setExpanded(_ expanded: Bool, animated: Bool) {
        guard expanded != isExpanded else { return }
        let layoutView = superview
        layoutView?.layoutIfNeeded()
        isExpanded = expanded
        if !expanded { cancelActiveJog() }
        // Hidden regular subviews still participate in Auto Layout. Only the
        // visible content should determine the card's bottom edge.
        collapsedBottomConstraint.isActive = false
        expandedBottomConstraint.isActive = false
        if expanded {
            expandedBottomConstraint.isActive = true
        } else {
            collapsedBottomConstraint.isActive = true
        }
        let apply = {
            self.detailStack.isHidden = !expanded
            self.detailStack.alpha = expanded ? 1 : 0
            self.chevron.transform = expanded ? CGAffineTransform(rotationAngle: .pi) : .identity
            layoutView?.layoutIfNeeded()
        }
        if animated {
            UIView.animate(withDuration: 0.25, delay: 0, options: [.curveEaseInOut], animations: apply)
        } else {
            apply()
        }
    }

    func updateLivePosition(_ position: UInt16) {
        liveRawPosition = position
        lastPositionUpdate = Date()
        rawPositionLabel.text = "Raw: \(position) / 4095"
        updatePositionSummary()
        if !suppressLiveSliderUpdates {
            positionSlider.setValue(Float(position), animated: false)
            positionSliderChanged()
        }
    }

    func updateState(_ state: ServoState) {
        updateTorqueButtons(state.isReadFailure ? nil : state.torqueEnabled)
        guard !state.isReadFailure else {
            lastStateUpdate = nil
            stateLabel.text = "Motor health unavailable · torque unknown"
            return
        }
        lastStateUpdate = Date()
        let torque = state.torqueEnabled.map { $0 ? "Torque on" : "Torque off" } ?? "Torque unknown"
        stateLabel.text = "\(torque) · Load \(state.load) · \(state.temperature)°C · \(Double(state.voltage) / 10.0)V"
        updateLivePosition(state.position)
    }

    func updateAxisStatus(_ status: ServoAxisStatus) {
        guard !status.isError, status.isTracked else {
            invalidateTelemetry()
            axisLabel.text = "Position unavailable · waiting for firmware"
            return
        }
        latestAxisStatus = status
        lastAxisUpdate = Date()
        telemetryFresh = true
        // An early idle read can race an asynchronous move. Keep marking disabled
        // until a later read confirms idle.
        if let until = motionPendingUntil, Date() >= until {
            motionPendingUntil = nil
        }
        limitsLabel.text = "Minimum: \(status.hasMin ? "set ✓" : "not set")    Maximum: \(status.hasMax ? "set ✓" : "not set")"
        updateMotionUI()
    }

    func invalidateTelemetry() {
        telemetryFresh = false
        lastAxisUpdate = nil
        lastStateUpdate = nil
        lastPositionUpdate = nil
        liveRawPosition = nil
        rawPositionLabel.text = "Raw: — / 4095"
        stateLabel.text = "Waiting for motor health · torque unknown"
        updateTorqueButtons(nil)
        updateMotionUI()
    }

    func checkFreshness(now: Date, staleAfter: TimeInterval) {
        if let lastPositionUpdate, now.timeIntervalSince(lastPositionUpdate) > staleAfter {
            self.lastPositionUpdate = nil
            liveRawPosition = nil
            rawPositionLabel.text = "Raw: stale"
        }
        if let lastAxisUpdate, now.timeIntervalSince(lastAxisUpdate) > staleAfter {
            telemetryFresh = false
        }
        if let lastStateUpdate, now.timeIntervalSince(lastStateUpdate) > max(6, staleAfter * 2) {
            self.lastStateUpdate = nil
            stateLabel.text = "Motor health stale · torque unknown"
            updateTorqueButtons(nil)
        }
        updateMotionUI()
    }

    private func updateMotionUI() {
        let canMark = controlsEnabled && telemetryFresh && !motionActive
        for button in [markMinButton, markMaxButton] {
            button.isEnabled = canMark
            button.alpha = canMark ? 1 : 0.4
        }
        if !telemetryFresh {
            axisLabel.text = latestAxisStatus == nil ? "Waiting for firmware position…" : "Position stale · waiting for firmware"
            limitsLabel.text = "Travel limits: waiting for live status"
        } else {
            axisLabel.text = wheelRunning ? "Wheel running · use Stop Wheel to stop" : (isJogging ? "Jogging · release to stop" : (motionPendingUntil != nil ? "Command sent · awaiting position" : (latestAxisStatus?.isMoving == true ? "Moving" : "Live · idle")))
        }
        updatePositionSummary()
    }

    private func updatePositionSummary() {
        let angle: String
        let turns: String
        if telemetryFresh, let status = latestAxisStatus {
            angle = status.angleDegrees.map { String(format: "%.1f°", $0) } ?? "Position —"
            turns = status.fullTurns > 0 ? "+\(status.fullTurns)" : "\(status.fullTurns)"
        } else {
            angle = "Position —"
            turns = "—"
        }
        positionLabel.text = "\(angle) · \(turns) full turns"
        let raw = liveRawPosition.map { String($0) } ?? "—"
        summaryLabel.text = "\(angle) · \(turns) turns\nRaw: \(raw)"
    }

    func setControlsEnabled(_ enabled: Bool) {
        controlsEnabled = enabled
        detailStack.alpha = enabled && isExpanded ? 1.0 : (isExpanded ? 0.4 : 0)
        detailStack.isUserInteractionEnabled = enabled
        if !enabled { updateTorqueButtons(nil) }
        updateMotionUI()
    }

    private func updateTorqueButtons(_ enabled: Bool?) {
        for (button, active, color) in [
            (torqueOnButton, enabled == true, UIColor.systemGreen),
            (torqueOffButton, enabled == false, UIColor.systemRed),
        ] {
            button.isSelected = active
            button.backgroundColor = active ? color : UIColor(white: 0.22, alpha: 1)
            button.setTitleColor(active ? .black : UIColor.white.withAlphaComponent(0.55), for: .normal)
            button.layer.borderWidth = active ? 1.5 : 0
            button.layer.borderColor = color.withAlphaComponent(0.9).cgColor
            button.layer.shadowColor = color.cgColor
            button.layer.shadowOpacity = active ? 0.6 : 0
            button.layer.shadowRadius = 6
            button.layer.shadowOffset = .zero
            button.accessibilityValue = enabled == nil ? "Torque state unknown" : (active ? "Active" : "Inactive")
        }
    }

    func cancelActiveJog() {
        guard motionActive else { return }
        isJogging = false
        commander()?.stopServoMotion(id: servoID)
        if wheelRunning {
            commander()?.driveServoWheel(id: servoID, speed: 0, acceleration: 50)
            wheelRunning = false
        }
        motionPendingUntil = Date().addingTimeInterval(1)
        updateMotionUI()
    }

    // MARK: Header

    private func buildHeader() {
        headerButton.translatesAutoresizingMaskIntoConstraints = false
        headerButton.addTarget(self, action: #selector(headerTapped), for: .touchUpInside)
        addSubview(headerButton)

        titleLabel.text = "Servo \(servoID)"
        titleLabel.font = .systemFont(ofSize: 16, weight: .semibold)
        titleLabel.textColor = .white

        summaryLabel.text = "Position —"
        summaryLabel.font = .monospacedSystemFont(ofSize: 13, weight: .medium)
        summaryLabel.textColor = UIColor.cyan.withAlphaComponent(0.85)
        summaryLabel.textAlignment = .right
        summaryLabel.numberOfLines = 2
        summaryLabel.adjustsFontSizeToFitWidth = true

        chevron.tintColor = UIColor.white.withAlphaComponent(0.5)
        chevron.contentMode = .scaleAspectFit

        let headerStack = UIStackView(arrangedSubviews: [titleLabel, summaryLabel, chevron])
        headerStack.translatesAutoresizingMaskIntoConstraints = false
        headerStack.axis = .horizontal
        headerStack.spacing = 10
        headerStack.isUserInteractionEnabled = false
        headerButton.addSubview(headerStack)

        NSLayoutConstraint.activate([
            headerButton.topAnchor.constraint(equalTo: topAnchor),
            headerButton.leadingAnchor.constraint(equalTo: leadingAnchor),
            headerButton.trailingAnchor.constraint(equalTo: trailingAnchor),
            headerButton.heightAnchor.constraint(equalToConstant: 48),

            headerStack.leadingAnchor.constraint(equalTo: headerButton.leadingAnchor, constant: 12),
            headerStack.trailingAnchor.constraint(equalTo: headerButton.trailingAnchor, constant: -12),
            headerStack.centerYAnchor.constraint(equalTo: headerButton.centerYAnchor),

            chevron.widthAnchor.constraint(equalToConstant: 16),
        ])
    }

    @objc private func headerTapped() {
        setExpanded(!isExpanded, animated: true)
        onToggleExpanded?(servoID, isExpanded)
    }

    // MARK: Detail layout

    private func buildDetail() {
        detailStack.translatesAutoresizingMaskIntoConstraints = false
        detailStack.axis = .vertical
        detailStack.spacing = 8
        detailStack.isHidden = true
        detailStack.alpha = 0
        addSubview(detailStack)

        NSLayoutConstraint.activate([
            detailStack.topAnchor.constraint(equalTo: headerButton.bottomAnchor),
            detailStack.leadingAnchor.constraint(equalTo: leadingAnchor, constant: 12),
            detailStack.trailingAnchor.constraint(equalTo: trailingAnchor, constant: -12),
            collapsedBottomConstraint,
        ])

        stateLabel.text = "Waiting for motor health…"
        stateLabel.font = .monospacedSystemFont(ofSize: 12, weight: .medium)
        stateLabel.textColor = UIColor.white.withAlphaComponent(0.55)
        stateLabel.numberOfLines = 0

        axisLabel.text = "Waiting for firmware position…"
        axisLabel.font = .monospacedSystemFont(ofSize: 12, weight: .medium)
        axisLabel.textColor = UIColor.white.withAlphaComponent(0.55)
        axisLabel.numberOfLines = 0

        // -- Position section
        positionLabel.text = "Position —"
        positionLabel.font = ServoFont.position.font
        positionLabel.adjustsFontForContentSizeCategory = true
        positionLabel.numberOfLines = 0
        positionLabel.textColor = UIColor.cyan.withAlphaComponent(0.85)

        rawPositionLabel.text = "Raw: — / 4095"
        rawPositionLabel.font = ServoFont.caption.font
        rawPositionLabel.adjustsFontForContentSizeCategory = true
        rawPositionLabel.textColor = UIColor.white.withAlphaComponent(0.7)
        rawPositionLabel.numberOfLines = 0
        targetPositionLabel.font = ServoFont.caption.font
        targetPositionLabel.adjustsFontForContentSizeCategory = true
        targetPositionLabel.textColor = UIColor.white.withAlphaComponent(0.7)
        targetPositionLabel.numberOfLines = 0
        positionSliderChanged()

        positionSlider.minimumValue = 0
        positionSlider.maximumValue = 4095
        positionSlider.isContinuous = true
        positionSlider.minimumTrackTintColor = UIColor(red: 0.2, green: 0.6, blue: 1.0, alpha: 1)
        positionSlider.maximumTrackTintColor = UIColor(white: 0.35, alpha: 1)
        positionSlider.addTarget(self, action: #selector(positionSliderChanged), for: .valueChanged)
        positionSlider.addTarget(self, action: #selector(positionSliderReleased), for: [.touchUpInside, .touchUpOutside])
        positionSlider.accessibilityLabel = "Raw encoder target"

        configureField(speedField, placeholder: "Speed", text: "1000")

        // -- Jog section (hold-to-jog)
        let jogLeftButton = makeHoldButton("Hold −")
        jogLeftButton.addTarget(self, action: #selector(jogLeftDown), for: .touchDown)
        let jogRightButton = makeHoldButton("Hold +")
        jogRightButton.addTarget(self, action: #selector(jogRightDown), for: .touchDown)
        for button in [jogLeftButton, jogRightButton] {
            button.addTarget(self, action: #selector(jogReleased), for: [.touchUpInside, .touchUpOutside, .touchCancel, .touchDragExit])
        }
        configureField(jogSpeedField, placeholder: "Jog speed", text: "0")
        jogLeftButton.accessibilityLabel = "Hold to jog in the negative direction"
        jogRightButton.accessibilityLabel = "Hold to jog in the positive direction"

        configureField(jogStepField, placeholder: "Step°", text: "15")
        jogStepField.keyboardType = .numbersAndPunctuation
        let stepBackButton = makeButton("− Step", color: UIColor(white: 0.30, alpha: 1), action: #selector(stepBackTapped))
        let stepForwardButton = makeButton("+ Step", color: UIColor(white: 0.30, alpha: 1), action: #selector(stepForwardTapped))

        limitsLabel.font = ServoFont.caption.font
        limitsLabel.adjustsFontForContentSizeCategory = true
        limitsLabel.textColor = UIColor.white.withAlphaComponent(0.7)
        limitsLabel.numberOfLines = 0

        // -- Wheel section
        configureField(wheelSpeedField, placeholder: "Wheel speed", text: "0")
        wheelSpeedField.keyboardType = .numbersAndPunctuation
        let wheelButton = makeButton("Drive Wheel", color: UIColor(red: 0.18, green: 0.43, blue: 0.86, alpha: 1), action: #selector(driveWheelTapped))
        let wheelStopButton = makeButton("Stop Wheel", color: UIColor(red: 0.58, green: 0.18, blue: 0.18, alpha: 1), action: #selector(stopWheelTapped))

        // -- Admin section
        updateTorqueButtons(nil)
        let positionModeButton = makeButton("Stop & Hold Position", color: UIColor(white: 0.30, alpha: 1), action: #selector(positionModeTapped))
        positionModeButton.accessibilityHint = "Stops continuous rotation and holds the current encoder position with torque enabled."
        let calibrateButton = makeButton("Calibrate Center…", color: UIColor(white: 0.30, alpha: 1), action: #selector(calibrateCenterTapped))
        configureField(newIDField, placeholder: "New ID", text: "\(min(253, Int(servoID) + 1))")
        let setIDButton = makeButton("Change ID…", color: UIColor(white: 0.30, alpha: 1), action: #selector(changeIDTapped))

        detailStack.addArrangedSubview(sectionLabel("Position"))
        detailStack.addArrangedSubview(row([positionLabel, rawPositionLabel]))
        detailStack.addArrangedSubview(axisLabel)
        detailStack.addArrangedSubview(stateLabel)
        detailStack.addArrangedSubview(sectionLabel("Jog"))
        detailStack.addArrangedSubview(row([jogLeftButton, jogRightButton]))
        detailStack.addArrangedSubview(hint("Hold to move. Release to stop. Use steps for fine adjustments."))
        detailStack.addArrangedSubview(row([stepBackButton, labeledField(jogStepField, "Step (°)"), stepForwardButton]))
        detailStack.addArrangedSubview(labeledField(jogSpeedField, "Hold speed · 0 = Auto, max 3000"))
        detailStack.addArrangedSubview(sectionLabel("Travel limits"))
        detailStack.addArrangedSubview(limitsLabel)
        detailStack.addArrangedSubview(row([markMinButton, markMaxButton]))
        detailStack.addArrangedSubview(hint("Jog to each endpoint, then set it here. Firmware enforces the marked limits. Limits and the turn counter reset on controller restart."))
        detailStack.addArrangedSubview(sectionLabel("Torque"))
        detailStack.addArrangedSubview(row([torqueOnButton, torqueOffButton]))
        detailStack.addArrangedSubview(hint("Torque off releases the motor for positioning by hand. Support the mechanism before releasing it."))
        detailStack.addArrangedSubview(advancedButton)
        advancedStack.axis = .vertical
        advancedStack.spacing = 8
        advancedStack.isHidden = true
        detailStack.addArrangedSubview(advancedStack)
        advancedStack.addArrangedSubview(sectionLabel("Raw position"))
        advancedStack.addArrangedSubview(hint("Direct hardware controls bypass multi-turn travel limits. Use with care."))
        advancedStack.addArrangedSubview(targetPositionLabel)
        advancedStack.addArrangedSubview(positionSlider)
        advancedStack.addArrangedSubview(labeledField(speedField, "Raw move speed · 0–4095"))
        advancedStack.addArrangedSubview(sectionLabel("Continuous wheel mode"))
        advancedStack.addArrangedSubview(labeledField(wheelSpeedField, "Wheel velocity (ticks/s) · −4095 to +4095"))
        wheelSpeedField.accessibilityHint = "Positive is clockwise, negative is counterclockwise, zero stops. 4096 ticks is one revolution."
        advancedStack.addArrangedSubview(hint("+ clockwise · − counterclockwise · 0 stopped. 4096 ticks = 1 revolution. Runs continuously until stopped; travel limits do not apply."))
        advancedStack.addArrangedSubview(row([wheelButton, wheelStopButton]))
        advancedStack.addArrangedSubview(positionModeButton)
        advancedStack.addArrangedSubview(hint("Stop & Hold exits wheel mode and holds the current position with torque on. Stop Wheel stops rotation without position holding."))
        advancedStack.addArrangedSubview(sectionLabel("Hardware setup"))
        advancedStack.addArrangedSubview(calibrateButton)
        advancedStack.addArrangedSubview(row([labeledField(newIDField, "New servo ID"), setIDButton]))
        updateMotionUI()
    }

    @objc private func toggleAdvanced() {
        // Do not hide the dedicated wheel stop while continuous rotation is active.
        if wheelRunning && !advancedStack.isHidden { stopWheelTapped() }
        advancedStack.isHidden.toggle()
        advancedButton.setTitle(advancedStack.isHidden ? "Advanced ▾" : "Advanced ▴", for: .normal)
        advancedButton.accessibilityValue = advancedStack.isHidden ? "Collapsed" : "Expanded"
    }

    // MARK: Actions — position

    @objc private func positionSliderChanged() {
        targetPositionLabel.text = "Target encoder: \(Int(positionSlider.value.rounded())) / 4095"
    }

    @objc private func positionSliderReleased() {
        cancelActiveJog()
        let position = UInt16(max(0, min(4095, Int(positionSlider.value.rounded()))))
        commander()?.moveServo(id: servoID, position: position, speed: currentSpeed())
    }

    // MARK: Actions — jog

    @objc private func jogLeftDown() { beginJog(.counterclockwise) }
    @objc private func jogRightDown() { beginJog(.clockwise) }

    private func beginJog(_ direction: ServoJogDirection) {
        guard !isJogging, !wheelRunning else { return }
        isJogging = true
        motionPendingUntil = Date().addingTimeInterval(1)
        updateMotionUI()
        commander()?.beginServoJog(id: servoID, direction: direction, speed: jogSpeed())
        refreshAxisSoon()
    }

    @objc private func jogReleased() {
        guard isJogging else { return }
        cancelActiveJog()
        refreshAxisSoon()
    }

    @objc private func stepBackTapped() { step(-1) }
    @objc private func stepForwardTapped() { step(1) }

    private func step(_ sign: Double) {
        guard !isJogging, !wheelRunning else { return }
        guard let degrees = number(from: jogStepField), degrees >= 0.1, degrees <= 360_000 else {
            onStatus?("Step must be 0.1–360000 degrees", true)
            return
        }
        motionPendingUntil = Date().addingTimeInterval(1)
        updateMotionUI()
        commander()?.jogServo(id: servoID, degrees: sign * degrees)
        refreshAxisSoon()
    }

    // MARK: Actions — marks & multiturn

    @objc private func markMinTapped() {
        guard telemetryFresh, !motionActive else { return }
        commander()?.markServoTravel(id: servoID, .min)
        onStatus?("Servo \(servoID): minimum mark requested", false)
        refreshAxisSoon()
    }

    @objc private func markMaxTapped() {
        guard telemetryFresh, !motionActive else { return }
        commander()?.markServoTravel(id: servoID, .max)
        onStatus?("Servo \(servoID): maximum mark requested", false)
        refreshAxisSoon()
    }

    // MARK: Actions — wheel & setup

    @objc private func driveWheelTapped() {
        guard !isJogging, latestAxisStatus?.isMoving != true, motionPendingUntil == nil else { return }
        guard let speed = number(from: wheelSpeedField), abs(speed) <= 4095 else {
            onStatus?("Wheel speed must be -4095…4095", true)
            return
        }
        wheelRunning = Int16(speed) != 0
        commander()?.driveServoWheel(id: servoID, speed: Int16(speed), acceleration: 50)
        updateMotionUI()
    }

    @objc private func stopWheelTapped() {
        commander()?.driveServoWheel(id: servoID, speed: 0, acceleration: 50)
        wheelRunning = false
        updateMotionUI()
    }

    @objc private func torqueOnTapped() {
        commander()?.setServoTorque(id: servoID, enabled: true)
        onStatus?("Servo \(servoID): torque on requested", false)
    }

    @objc private func torqueOffTapped() {
        cancelActiveJog()
        commander()?.setServoTorque(id: servoID, enabled: false)
        onStatus?("Servo \(servoID): torque off requested — support the mechanism", false)
    }

    @objc private func positionModeTapped() {
        cancelActiveJog()
        commander()?.setServoPositionMode(id: servoID)
    }

    @objc private func calibrateCenterTapped() {
        guard telemetryFresh, !motionActive else {
            onStatus?("Stop the servo and wait for live idle status before resetting center", true)
            return
        }
        let id = servoID
        presentConfirmation?(
            "Reset Center",
            "Store the current position of servo \(id) as center (raw 2048), zero angle, and zero turns? This writes to servo EEPROM and clears travel limits. Re-mark the limits afterward. Requires firmware with center-relative tracking.",
            { [weak self] in
                guard let self, self.telemetryFresh, !self.motionActive else { return }
                self.commander()?.calibrateServoCenter(id: id)
                self.invalidateTelemetry()
                self.refreshAxisSoon()
            }
        )
    }

    @objc private func changeIDTapped() {
        guard let newID = number(from: newIDField), (1...253).contains(newID), newID.rounded() == newID else {
            onStatus?("New ID must be 1-253", true)
            return
        }
        let id = servoID
        presentConfirmation?(
            "Change Servo ID",
            "Change servo \(id) to ID \(Int(newID))? This writes to servo EEPROM and rescans the bus.",
            { [weak self] in
                self?.cancelActiveJog()
                self?.commander()?.changeServoID(currentID: id, newID: UInt8(newID))
            }
        )
    }

    // MARK: Helpers

    private func currentSpeed() -> UInt16 {
        guard let speed = number(from: speedField), (0...4095).contains(speed) else { return 1000 }
        return UInt16(speed)
    }

    private func jogSpeed() -> UInt16 {
        guard let speed = number(from: jogSpeedField), (0...3000).contains(speed) else { return 0 }
        return UInt16(speed)
    }

    private func number(from field: UITextField) -> Double? {
        guard let value = Double(field.text?.trimmingCharacters(in: .whitespacesAndNewlines) ?? ""), value.isFinite else { return nil }
        return value
    }

    /// Coalesce command readbacks into the list's next polling tick.
    private func refreshAxisSoon() {
        onNeedsRefresh?()
    }

    private func hint(_ text: String) -> UILabel {
        let label = UILabel()
        label.text = text
        label.font = ServoFont.caption.font
        label.adjustsFontForContentSizeCategory = true
        label.textColor = UIColor.white.withAlphaComponent(0.55)
        label.numberOfLines = 0
        return label
    }

    private func labeledField(_ field: UITextField, _ title: String) -> UIStackView {
        field.accessibilityLabel = title
        let stack = UIStackView(arrangedSubviews: [hint(title), field])
        stack.axis = .vertical
        stack.spacing = 4
        return stack
    }

    @objc private func dismissKeyboard() {
        endEditing(true)
    }

    private func sectionLabel(_ text: String) -> UILabel {
        let label = UILabel()
        label.text = text.uppercased()
        label.font = .systemFont(ofSize: 11, weight: .semibold)
        label.textColor = UIColor.white.withAlphaComponent(0.4)
        return label
    }

    private func row(_ views: [UIView]) -> UIStackView {
        let row = UIStackView(arrangedSubviews: views)
        row.axis = .horizontal
        row.spacing = 8
        row.distribution = .fillEqually
        row.alignment = .bottom
        return row
    }

    private func configureField(_ field: UITextField, placeholder: String, text: String) {
        field.placeholder = placeholder
        field.text = text
        field.font = .monospacedSystemFont(ofSize: 13, weight: .medium)
        field.textColor = .white
        field.backgroundColor = UIColor(white: 0.2, alpha: 1)
        field.layer.cornerRadius = 8
        field.keyboardType = .numberPad
        field.textAlignment = .center
        field.accessibilityLabel = placeholder
        let toolbar = UIToolbar()
        toolbar.items = [UIBarButtonItem(systemItem: .flexibleSpace), UIBarButtonItem(title: "Done", style: .done, target: self, action: #selector(dismissKeyboard))]
        toolbar.sizeToFit()
        field.inputAccessoryView = toolbar
        field.attributedPlaceholder = NSAttributedString(
            string: placeholder,
            attributes: [.foregroundColor: UIColor(white: 0.5, alpha: 1)]
        )
        field.heightAnchor.constraint(greaterThanOrEqualToConstant: 44).isActive = true
    }

    private func makeButton(_ title: String, color: UIColor, action: Selector) -> UIButton {
        let button = UIButton(type: .custom)
        button.setTitle(title, for: .normal)
        button.titleLabel?.font = ServoFont.button.font
        button.titleLabel?.adjustsFontForContentSizeCategory = true
        button.titleLabel?.adjustsFontSizeToFitWidth = true
        button.setTitleColor(.white, for: .normal)
        button.backgroundColor = color
        button.layer.cornerRadius = 8
        button.addTarget(self, action: action, for: .touchUpInside)
        button.heightAnchor.constraint(greaterThanOrEqualToConstant: 44).isActive = true
        return button
    }

    private func makeHoldButton(_ title: String) -> UIButton {
        let button = UIButton(type: .system)
        button.setTitle(title, for: .normal)
        button.setTitleColor(.white, for: .normal)
        button.titleLabel?.font = ServoFont.holdButton.font
        button.titleLabel?.adjustsFontForContentSizeCategory = true
        button.isExclusiveTouch = true
        button.accessibilityHint = "Release to stop. For a discrete move, use the step buttons."
        button.backgroundColor = UIColor(red: 0.18, green: 0.43, blue: 0.86, alpha: 1)
        button.layer.cornerRadius = 8
        button.heightAnchor.constraint(greaterThanOrEqualToConstant: 52).isActive = true
        return button
    }
}
