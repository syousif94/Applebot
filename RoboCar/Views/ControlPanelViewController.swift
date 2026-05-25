//
//  ControlPanelViewController.swift
//  RoboCar
//
//  Created by Sammy Yousif on 2/6/26.
//

import UIKit

/// Bottom sheet that shows ESP32 BLE connection status and a joystick for manual driving
class ControlPanelViewController: UIViewController {
    
    // MARK: - Views
    
    private let scrollView = UIScrollView()
    private let contentView = UIView()
    private let grabber = UIView()
    private let titleLabel = UILabel()
    private let statusDot = UIView()
    private let statusLabel = UILabel()
    private let connectButton = UIButton(type: .system)
    private let joystickView = JoystickView()
    private let joystickLabel = UILabel()
    private let motorLabel = UILabel()
    
    // Servo views
    private let servoLabel = UILabel()
    private let servoStatusLabel = UILabel()
    private let servoSettingsButton = UIButton(type: .custom)
    private let servoStack = UIStackView()
    private let servoListStack = UIStackView()
    private let servoScanFromField = UITextField()
    private let servoScanToField = UITextField()
    private let rescanServosButton = UIButton(type: .custom)
    private let servoIDField = UITextField()
    private let servoNewIDField = UITextField()
    private let changeServoIDButton = UIButton(type: .custom)
    private let servoPositionLabel = UILabel()
    private let servoPositionSlider = UISlider()
    private let servoSpeedField = UITextField()
    private let refreshServoStateButton = UIButton(type: .custom)
    private let moveServoButton = UIButton(type: .custom)
    private let moveAllServosButton = UIButton(type: .custom)
    private let servoTorqueOnButton = UIButton(type: .custom)
    private let servoTorqueOffButton = UIButton(type: .custom)
    private let servoAllTorqueOnButton = UIButton(type: .custom)
    private let servoAllTorqueOffButton = UIButton(type: .custom)
    private var servoControlViews: [UIView] = []
    private var scannedServoIDs: [UInt8] = []
    private var servoNewIDFieldsByID: [UInt8: UITextField] = [:]
    private var servoPositionSlidersByID: [UInt8: UISlider] = [:]
    private var servoPositionLabelsByID: [UInt8: UILabel] = [:]
    private var servoStateLabelsByID: [UInt8: UILabel] = [:]
    private var servoStatesByID: [UInt8: ServoState] = [:]
    private var servoPopupConstraints: [NSLayoutConstraint] = []
    private weak var servoSettingsViewController: UIViewController?
    
    // WiFi config views
    private let wifiLabel = UILabel()
    private let ssidField = UITextField()
    private let passwordField = UITextField()
    private let sendWifiButton = UIButton(type: .custom)
    private let wifiStatusLabel = UILabel()
    
    // Battery views
    private let batteryLabel = UILabel()
    private let batteryPercentLabel = UILabel()
    private let batteryVoltageLabel = UILabel()
    private let batteryBar = UIView()
    private let batteryBarFill = UIView()
    private var batteryBarFillWidth: NSLayoutConstraint!
    
    // Telemetry views
    private let telemetryLabel = UILabel()
    private let serverURLField = UITextField()
    private let telemetryButton = UIButton(type: .custom)
    private let telemetryStatusDot = UIView()
    private let telemetryStatusLabel = UILabel()
    
    private let ble = ESP32BLEManager.shared
    
    // MARK: - Lifecycle
    
    override func viewDidLoad() {
        super.viewDidLoad()
        
        view.backgroundColor = UIColor(white: 0.12, alpha: 1)
        
        setupScrollView()
        setupGrabber()
        setupHeader()
        setupConnectionRow()
        setupBatteryStatus()
        setupServoControl()
        setupJoystick()
        setupWiFiConfig()
        setupTelemetryConfig()
        
        // Dismiss keyboard on tap outside text fields
        let tap = UITapGestureRecognizer(target: self, action: #selector(dismissKeyboard))
        tap.cancelsTouchesInView = false
        tap.delegate = self
        scrollView.addGestureRecognizer(tap)
        
        // Listen for BLE state changes
        ble.onStateChanged = { [weak self] state in
            DispatchQueue.main.async { self?.updateConnectionUI(state) }
        }
        ble.onBatteryUpdated = { [weak self] pct, volts in
            DispatchQueue.main.async { self?.updateBatteryUI(percentage: pct, voltage: volts) }
        }
        updateConnectionUI(ble.connectionState)
        
        // Adjust scroll view insets when keyboard appears/disappears
        NotificationCenter.default.addObserver(self, selector: #selector(keyboardWillShow(_:)), name: UIResponder.keyboardWillShowNotification, object: nil)
        NotificationCenter.default.addObserver(self, selector: #selector(keyboardWillHide(_:)), name: UIResponder.keyboardWillHideNotification, object: nil)
    }
    
    override func viewDidLayoutSubviews() {
        super.viewDidLayoutSubviews()
        statusDot.layer.cornerRadius = statusDot.bounds.width / 2
        telemetryStatusDot.layer.cornerRadius = telemetryStatusDot.bounds.width / 2
        batteryBar.layer.cornerRadius = 4
        batteryBarFill.layer.cornerRadius = 3
    }
    
    // MARK: - Setup
    
    private func setupScrollView() {
        scrollView.translatesAutoresizingMaskIntoConstraints = false
        scrollView.showsVerticalScrollIndicator = true
        scrollView.alwaysBounceVertical = true
        scrollView.keyboardDismissMode = .interactive
        scrollView.delaysContentTouches = false
        scrollView.canCancelContentTouches = false
        view.addSubview(scrollView)
        
        contentView.translatesAutoresizingMaskIntoConstraints = false
        scrollView.addSubview(contentView)
        
        NSLayoutConstraint.activate([
            scrollView.topAnchor.constraint(equalTo: view.topAnchor),
            scrollView.leadingAnchor.constraint(equalTo: view.leadingAnchor),
            scrollView.trailingAnchor.constraint(equalTo: view.trailingAnchor),
            scrollView.bottomAnchor.constraint(equalTo: view.bottomAnchor),
            
            contentView.topAnchor.constraint(equalTo: scrollView.contentLayoutGuide.topAnchor),
            contentView.leadingAnchor.constraint(equalTo: scrollView.contentLayoutGuide.leadingAnchor),
            contentView.trailingAnchor.constraint(equalTo: scrollView.contentLayoutGuide.trailingAnchor),
            contentView.bottomAnchor.constraint(equalTo: scrollView.contentLayoutGuide.bottomAnchor),
            contentView.widthAnchor.constraint(equalTo: scrollView.frameLayoutGuide.widthAnchor)
        ])
    }
    
    private func setupGrabber() {
        grabber.translatesAutoresizingMaskIntoConstraints = false
        grabber.backgroundColor = UIColor.white.withAlphaComponent(0.3)
        grabber.layer.cornerRadius = 2.5
        contentView.addSubview(grabber)
        
        NSLayoutConstraint.activate([
            grabber.topAnchor.constraint(equalTo: contentView.topAnchor, constant: 8),
            grabber.centerXAnchor.constraint(equalTo: contentView.centerXAnchor),
            grabber.widthAnchor.constraint(equalToConstant: 36),
            grabber.heightAnchor.constraint(equalToConstant: 5)
        ])
    }
    
    private func setupHeader() {
        titleLabel.translatesAutoresizingMaskIntoConstraints = false
        titleLabel.text = "ESP32 Controller"
        titleLabel.font = .systemFont(ofSize: 18, weight: .bold)
        titleLabel.textColor = .white
        contentView.addSubview(titleLabel)
        
        NSLayoutConstraint.activate([
            titleLabel.topAnchor.constraint(equalTo: grabber.bottomAnchor, constant: 16),
            titleLabel.leadingAnchor.constraint(equalTo: contentView.leadingAnchor, constant: 20)
        ])
    }
    
    private func setupConnectionRow() {
        // Status dot
        statusDot.translatesAutoresizingMaskIntoConstraints = false
        statusDot.backgroundColor = .red
        contentView.addSubview(statusDot)
        
        // Status label
        statusLabel.translatesAutoresizingMaskIntoConstraints = false
        statusLabel.font = .systemFont(ofSize: 14, weight: .medium)
        statusLabel.textColor = .white
        statusLabel.text = "Disconnected"
        contentView.addSubview(statusLabel)
        
        // Connect button
        connectButton.translatesAutoresizingMaskIntoConstraints = false
        connectButton.setTitle("Connect", for: .normal)
        connectButton.titleLabel?.font = .systemFont(ofSize: 15, weight: .semibold)
        connectButton.setTitleColor(.white, for: .normal)
        connectButton.backgroundColor = UIColor(red: 0.2, green: 0.5, blue: 1.0, alpha: 1)
        connectButton.layer.cornerRadius = 8
        connectButton.contentEdgeInsets = UIEdgeInsets(top: 8, left: 20, bottom: 8, right: 20)
        connectButton.addTarget(self, action: #selector(connectTapped), for: .touchUpInside)
        contentView.addSubview(connectButton)
        
        NSLayoutConstraint.activate([
            statusDot.leadingAnchor.constraint(equalTo: contentView.leadingAnchor, constant: 20),
            statusDot.centerYAnchor.constraint(equalTo: connectButton.centerYAnchor),
            statusDot.widthAnchor.constraint(equalToConstant: 10),
            statusDot.heightAnchor.constraint(equalToConstant: 10),
            
            statusLabel.leadingAnchor.constraint(equalTo: statusDot.trailingAnchor, constant: 8),
            statusLabel.centerYAnchor.constraint(equalTo: connectButton.centerYAnchor),
            
            connectButton.topAnchor.constraint(equalTo: titleLabel.bottomAnchor, constant: 16),
            connectButton.trailingAnchor.constraint(equalTo: contentView.trailingAnchor, constant: -20)
        ])
    }
    
    private func setupBatteryStatus() {
        // Section label
        batteryLabel.translatesAutoresizingMaskIntoConstraints = false
        batteryLabel.text = "Battery"
        batteryLabel.font = .systemFont(ofSize: 14, weight: .semibold)
        batteryLabel.textColor = UIColor.white.withAlphaComponent(0.6)
        contentView.addSubview(batteryLabel)
        
        // Percentage label
        batteryPercentLabel.translatesAutoresizingMaskIntoConstraints = false
        batteryPercentLabel.text = "—"
        batteryPercentLabel.font = .monospacedSystemFont(ofSize: 14, weight: .bold)
        batteryPercentLabel.textColor = .white
        contentView.addSubview(batteryPercentLabel)
        
        // Voltage label
        batteryVoltageLabel.translatesAutoresizingMaskIntoConstraints = false
        batteryVoltageLabel.text = ""
        batteryVoltageLabel.font = .monospacedSystemFont(ofSize: 12, weight: .medium)
        batteryVoltageLabel.textColor = UIColor.white.withAlphaComponent(0.5)
        contentView.addSubview(batteryVoltageLabel)
        
        // Progress bar background
        batteryBar.translatesAutoresizingMaskIntoConstraints = false
        batteryBar.backgroundColor = UIColor(white: 0.25, alpha: 1)
        batteryBar.clipsToBounds = true
        contentView.addSubview(batteryBar)
        
        // Progress bar fill
        batteryBarFill.translatesAutoresizingMaskIntoConstraints = false
        batteryBarFill.backgroundColor = UIColor(red: 0.2, green: 0.9, blue: 0.4, alpha: 1)
        batteryBar.addSubview(batteryBarFill)
        
        batteryBarFillWidth = batteryBarFill.widthAnchor.constraint(equalToConstant: 0)
        
        NSLayoutConstraint.activate([
            batteryLabel.topAnchor.constraint(equalTo: connectButton.bottomAnchor, constant: 20),
            batteryLabel.leadingAnchor.constraint(equalTo: contentView.leadingAnchor, constant: 20),
            
            batteryPercentLabel.centerYAnchor.constraint(equalTo: batteryLabel.centerYAnchor),
            batteryPercentLabel.trailingAnchor.constraint(equalTo: contentView.trailingAnchor, constant: -20),
            
            batteryVoltageLabel.centerYAnchor.constraint(equalTo: batteryLabel.centerYAnchor),
            batteryVoltageLabel.trailingAnchor.constraint(equalTo: batteryPercentLabel.leadingAnchor, constant: -8),
            
            batteryBar.topAnchor.constraint(equalTo: batteryLabel.bottomAnchor, constant: 8),
            batteryBar.leadingAnchor.constraint(equalTo: contentView.leadingAnchor, constant: 20),
            batteryBar.trailingAnchor.constraint(equalTo: contentView.trailingAnchor, constant: -20),
            batteryBar.heightAnchor.constraint(equalToConstant: 8),
            
            batteryBarFill.leadingAnchor.constraint(equalTo: batteryBar.leadingAnchor, constant: 1),
            batteryBarFill.topAnchor.constraint(equalTo: batteryBar.topAnchor, constant: 1),
            batteryBarFill.bottomAnchor.constraint(equalTo: batteryBar.bottomAnchor, constant: -1),
            batteryBarFillWidth,
        ])
        
        // Load cached battery level if available
        let cachedPct = UserDefaults.standard.integer(forKey: Self.cachedBatteryPercentKey)
        let cachedVolts = UserDefaults.standard.double(forKey: Self.cachedBatteryVoltageKey)
        if cachedPct > 0 {
            updateBatteryUI(percentage: UInt8(cachedPct), voltage: cachedVolts)
        } else {
            updateBatteryUI(percentage: 0, voltage: 0)
        }
    }
    
    private static let cachedBatteryPercentKey = "cachedBatteryPercent"
    private static let cachedBatteryVoltageKey = "cachedBatteryVoltage"
    private static let cachedServoIDKey = "cachedServoID"
    private static let cachedServoNewIDKey = "cachedServoNewID"
    private static let cachedServoPositionKey = "cachedServoPosition"
    private static let cachedServoSpeedKey = "cachedServoSpeed"
    
    private func updateBatteryUI(percentage: UInt8, voltage: Double) {
        let pct = Int(percentage)
        if pct == 0 && voltage == 0 {
            batteryPercentLabel.text = "—"
            batteryVoltageLabel.text = ""
            batteryBarFillWidth.constant = 0
            batteryBarFill.backgroundColor = .gray
        } else {
            // Cache latest values
            UserDefaults.standard.set(Int(percentage), forKey: Self.cachedBatteryPercentKey)
            UserDefaults.standard.set(voltage, forKey: Self.cachedBatteryVoltageKey)
            
            batteryPercentLabel.text = "\(pct)%"
            batteryVoltageLabel.text = String(format: "%.2fV", voltage)
            
            // Bar width = fraction of (bar width - 2px inset)
            let barMaxWidth = view.bounds.width - 42 // 20+20 margins + 2 inset
            batteryBarFillWidth.constant = max(0, barMaxWidth * CGFloat(pct) / 100.0)
            
            // Color: green > 50%, yellow 20-50%, red < 20%
            if pct > 50 {
                batteryBarFill.backgroundColor = UIColor(red: 0.2, green: 0.9, blue: 0.4, alpha: 1)
            } else if pct > 20 {
                batteryBarFill.backgroundColor = UIColor(red: 1.0, green: 0.8, blue: 0.2, alpha: 1)
            } else {
                batteryBarFill.backgroundColor = UIColor(red: 1.0, green: 0.3, blue: 0.3, alpha: 1)
            }
        }
        
        UIView.animate(withDuration: 0.3) {
            self.batteryBar.layoutIfNeeded()
        }
    }
    
    private func setupJoystick() {
        // Joystick label
        joystickLabel.translatesAutoresizingMaskIntoConstraints = false
        joystickLabel.text = "Manual Drive"
        joystickLabel.font = .systemFont(ofSize: 14, weight: .semibold)
        joystickLabel.textColor = UIColor.white.withAlphaComponent(0.6)
        contentView.addSubview(joystickLabel)
        
        // Motor power readout
        motorLabel.translatesAutoresizingMaskIntoConstraints = false
        motorLabel.text = "L: 0%  R: 0%"
        motorLabel.font = .monospacedSystemFont(ofSize: 13, weight: .medium)
        motorLabel.textColor = UIColor.cyan.withAlphaComponent(0.8)
        motorLabel.textAlignment = .right
        contentView.addSubview(motorLabel)
        
        // Joystick
        joystickView.translatesAutoresizingMaskIntoConstraints = false
        contentView.addSubview(joystickView)
        
        NSLayoutConstraint.activate([
            joystickLabel.topAnchor.constraint(equalTo: servoSettingsButton.bottomAnchor, constant: 24),
            joystickLabel.leadingAnchor.constraint(equalTo: contentView.leadingAnchor, constant: 20),
            
            motorLabel.centerYAnchor.constraint(equalTo: joystickLabel.centerYAnchor),
            motorLabel.trailingAnchor.constraint(equalTo: contentView.trailingAnchor, constant: -20),
            
            joystickView.topAnchor.constraint(equalTo: joystickLabel.bottomAnchor, constant: 12),
            joystickView.centerXAnchor.constraint(equalTo: contentView.centerXAnchor),
            joystickView.widthAnchor.constraint(equalToConstant: 180),
            joystickView.heightAnchor.constraint(equalToConstant: 180),
        ])
        
        // Joystick callbacks
        joystickView.onMove = { [weak self] x, y in
            let powers = self?.ble.drive(x: x, y: y) ?? (left: Float(0), right: Float(0))
            self?.motorLabel.text = String(format: "L: %d%%  R: %d%%",
                                           Int(powers.left * 100), Int(powers.right * 100))
        }
        
        joystickView.onRelease = { [weak self] in
            self?.ble.stopAll()
            self?.motorLabel.text = "L: 0%  R: 0%"
        }
        
        joystickView.onTouchStateChanged = { [weak self] isTouching in
            self?.isModalInPresentation = isTouching
            // Disable the sheet's internal pan-to-dismiss gesture while joystick is active
            self?.setSheetGesturesEnabled(!isTouching)
        }
    }
    
    private func setupServoControl() {
        servoLabel.translatesAutoresizingMaskIntoConstraints = false
        servoLabel.text = "ST3215 Servos"
        servoLabel.font = .systemFont(ofSize: 14, weight: .semibold)
        servoLabel.textColor = UIColor.white.withAlphaComponent(0.6)
        contentView.addSubview(servoLabel)
        
        servoStatusLabel.translatesAutoresizingMaskIntoConstraints = false
        servoStatusLabel.text = "No servos scanned"
        servoStatusLabel.font = .monospacedSystemFont(ofSize: 12, weight: .medium)
        servoStatusLabel.textColor = UIColor.cyan.withAlphaComponent(0.8)
        servoStatusLabel.textAlignment = .right
        contentView.addSubview(servoStatusLabel)

        configureServoButton(servoSettingsButton, title: "Servo Settings", color: UIColor(white: 0.28, alpha: 1), action: #selector(showServoSettingsTapped))
        servoSettingsButton.translatesAutoresizingMaskIntoConstraints = false
        contentView.addSubview(servoSettingsButton)

        servoStack.translatesAutoresizingMaskIntoConstraints = false
        servoStack.axis = .vertical
        servoStack.spacing = 10

        servoListStack.translatesAutoresizingMaskIntoConstraints = false
        servoListStack.axis = .vertical
        servoListStack.spacing = 10

        configureServoTextField(servoScanFromField, placeholder: "From", text: "1")
        configureServoTextField(servoScanToField, placeholder: "To", text: "20")
        configureServoButton(rescanServosButton, title: "Scan", color: UIColor(red: 0.2, green: 0.5, blue: 1.0, alpha: 1), action: #selector(rescanServosTapped))

        let cachedPosition = UserDefaults.standard.object(forKey: Self.cachedServoPositionKey) as? Int ?? 2048
        servoPositionLabel.text = "Position \(min(4095, max(0, cachedPosition)))"
        servoPositionLabel.font = .monospacedSystemFont(ofSize: 13, weight: .medium)
        servoPositionLabel.textColor = UIColor.white.withAlphaComponent(0.7)

        servoPositionSlider.minimumValue = 0
        servoPositionSlider.maximumValue = 4095
        servoPositionSlider.value = Float(min(4095, max(0, cachedPosition)))
        servoPositionSlider.isContinuous = false
        servoPositionSlider.minimumTrackTintColor = UIColor(red: 0.2, green: 0.6, blue: 1.0, alpha: 1)
        servoPositionSlider.maximumTrackTintColor = UIColor(white: 0.35, alpha: 1)
        servoPositionSlider.addTarget(self, action: #selector(servoPositionChanged), for: .valueChanged)

        let cachedSpeed = UserDefaults.standard.object(forKey: Self.cachedServoSpeedKey) as? Int ?? 1000
        configureServoTextField(servoSpeedField, placeholder: "Speed", text: "\(cachedSpeed)")
        configureServoButton(moveAllServosButton, title: "Move All", color: UIColor(red: 0.25, green: 0.45, blue: 0.85, alpha: 1), action: #selector(moveAllServosTapped))
        configureServoButton(servoAllTorqueOnButton, title: "All On", color: UIColor(red: 0.2, green: 0.55, blue: 0.35, alpha: 1), action: #selector(enableAllServoTorqueTapped))
        configureServoButton(servoAllTorqueOffButton, title: "All Off", color: UIColor(red: 0.65, green: 0.2, blue: 0.2, alpha: 1), action: #selector(disableAllServoTorqueTapped))

        servoStack.addArrangedSubview(makeServoRow([servoScanFromField, servoScanToField, rescanServosButton]))
        servoStack.addArrangedSubview(makeServoRow([servoSpeedField, moveAllServosButton]))
        servoStack.addArrangedSubview(makeServoRow([servoAllTorqueOnButton, servoAllTorqueOffButton]))
        servoStack.addArrangedSubview(servoListStack)
        rebuildServoRows()

        servoControlViews = [
            servoScanFromField, servoScanToField, rescanServosButton,
            servoSpeedField, moveAllServosButton,
            servoAllTorqueOnButton, servoAllTorqueOffButton,
        ]
        
        NSLayoutConstraint.activate([
            servoLabel.topAnchor.constraint(equalTo: batteryBar.bottomAnchor, constant: 24),
            servoLabel.leadingAnchor.constraint(equalTo: contentView.leadingAnchor, constant: 20),
            
            servoStatusLabel.centerYAnchor.constraint(equalTo: servoLabel.centerYAnchor),
            servoStatusLabel.leadingAnchor.constraint(greaterThanOrEqualTo: servoLabel.trailingAnchor, constant: 12),
            servoStatusLabel.trailingAnchor.constraint(equalTo: contentView.trailingAnchor, constant: -20),
            
            servoSettingsButton.topAnchor.constraint(equalTo: servoLabel.bottomAnchor, constant: 12),
            servoSettingsButton.leadingAnchor.constraint(equalTo: contentView.leadingAnchor, constant: 20),
            servoSettingsButton.trailingAnchor.constraint(equalTo: contentView.trailingAnchor, constant: -20),
        ])

        ble.onServoListUpdated = { [weak self] ids in
            DispatchQueue.main.async { self?.updateServoList(ids) }
        }
        ble.onServoStateUpdated = { [weak self] state in
            DispatchQueue.main.async { self?.updateServoState(state) }
        }
    }

    private func configureServoTextField(_ field: UITextField, placeholder: String, text: String) {
        field.placeholder = placeholder
        field.text = text
        field.font = .monospacedSystemFont(ofSize: 13, weight: .medium)
        field.textColor = .white
        field.backgroundColor = UIColor(white: 0.2, alpha: 1)
        field.layer.cornerRadius = 8
        field.keyboardType = .numberPad
        field.textAlignment = .center
        field.delegate = self
        field.attributedPlaceholder = NSAttributedString(
            string: placeholder,
            attributes: [.foregroundColor: UIColor(white: 0.5, alpha: 1)]
        )
        field.heightAnchor.constraint(equalToConstant: 40).isActive = true
    }

    private func configureServoButton(_ button: UIButton, title: String, color: UIColor, action: Selector) {
        button.setTitle(title, for: .normal)
        button.titleLabel?.font = .systemFont(ofSize: 13, weight: .semibold)
        button.setTitleColor(.white, for: .normal)
        button.backgroundColor = color
        button.layer.cornerRadius = 8
        button.addTarget(self, action: action, for: .touchUpInside)
        button.heightAnchor.constraint(equalToConstant: 40).isActive = true
    }

    private func makeServoRow(_ views: [UIView]) -> UIStackView {
        let row = UIStackView(arrangedSubviews: views)
        row.axis = .horizontal
        row.spacing = 8
        row.distribution = .fillEqually
        return row
    }

    private func servoID(from field: UITextField, name: String) -> UInt8? {
        guard let value = boundedServoNumber(from: field, name: name, range: 1...253) else { return nil }
        return UInt8(value)
    }

    private func boundedServoNumber(from field: UITextField, name: String, range: ClosedRange<Int>) -> Int? {
        let rawValue = field.text?.trimmingCharacters(in: .whitespacesAndNewlines) ?? ""
        guard let value = Int(rawValue), range.contains(value) else {
            setServoStatus("\(name) must be \(range.lowerBound)-\(range.upperBound)", isError: true)
            return nil
        }
        return value
    }

    private func servoPosition() -> UInt16 {
        let position = Int(servoPositionSlider.value.rounded())
        UserDefaults.standard.set(position, forKey: Self.cachedServoPositionKey)
        return UInt16(position)
    }

    private func servoSpeed() -> UInt16? {
        guard let speed = boundedServoNumber(from: servoSpeedField, name: "Speed", range: 0...4095) else { return nil }
        UserDefaults.standard.set(speed, forKey: Self.cachedServoSpeedKey)
        return UInt16(speed)
    }

    private func setServoStatus(_ text: String, isError: Bool = false) {
        servoStatusLabel.text = text
        servoStatusLabel.textColor = isError ? UIColor(red: 1.0, green: 0.35, blue: 0.35, alpha: 1) : UIColor.cyan.withAlphaComponent(0.8)
    }

    private func rebuildServoRows() {
        servoListStack.arrangedSubviews.forEach { view in
            servoListStack.removeArrangedSubview(view)
            view.removeFromSuperview()
        }
        servoNewIDFieldsByID.removeAll()
        servoPositionSlidersByID.removeAll()
        servoPositionLabelsByID.removeAll()
        servoStateLabelsByID.removeAll()

        guard !scannedServoIDs.isEmpty else {
            let emptyLabel = UILabel()
            emptyLabel.text = "No servos scanned"
            emptyLabel.font = .systemFont(ofSize: 13, weight: .medium)
            emptyLabel.textColor = UIColor.white.withAlphaComponent(0.45)
            emptyLabel.textAlignment = .center
            emptyLabel.backgroundColor = UIColor(white: 0.17, alpha: 1)
            emptyLabel.layer.cornerRadius = 8
            emptyLabel.layer.masksToBounds = true
            emptyLabel.heightAnchor.constraint(equalToConstant: 52).isActive = true
            servoListStack.addArrangedSubview(emptyLabel)
            return
        }
        scannedServoIDs.forEach { id in
            servoListStack.addArrangedSubview(makeServoControlCard(id: id))
        }
        updateServoRowsEnabled(ble.connectionState == .connected)
    }

    private func makeServoControlCard(id: UInt8) -> UIView {
        let card = UIView()
        card.backgroundColor = UIColor(white: 0.17, alpha: 1)
        card.layer.cornerRadius = 8

        let stack = UIStackView()
        stack.translatesAutoresizingMaskIntoConstraints = false
        stack.axis = .vertical
        stack.spacing = 8
        card.addSubview(stack)

        let titleLabel = UILabel()
        titleLabel.text = "Servo ID \(id)"
        titleLabel.font = .monospacedSystemFont(ofSize: 14, weight: .semibold)
        titleLabel.textColor = .white

        let stateLabel = UILabel()
        stateLabel.text = servoStateText(for: id)
        stateLabel.font = .monospacedSystemFont(ofSize: 12, weight: .medium)
        stateLabel.textColor = UIColor.white.withAlphaComponent(0.55)
        servoStateLabelsByID[id] = stateLabel

        let positionLabel = UILabel()
        let position = Int(servoStatesByID[id]?.position ?? servoPosition())
        positionLabel.text = "Position \(position)"
        positionLabel.font = .monospacedSystemFont(ofSize: 13, weight: .medium)
        positionLabel.textColor = UIColor.cyan.withAlphaComponent(0.85)
        servoPositionLabelsByID[id] = positionLabel

        let positionSlider = UISlider()
        positionSlider.minimumValue = 0
        positionSlider.maximumValue = 4095
        positionSlider.value = Float(position)
        positionSlider.isContinuous = false
        positionSlider.minimumTrackTintColor = UIColor(red: 0.2, green: 0.6, blue: 1.0, alpha: 1)
        positionSlider.maximumTrackTintColor = UIColor(white: 0.35, alpha: 1)
        positionSlider.tag = Int(id)
        positionSlider.addTarget(self, action: #selector(scannedServoPositionChanged(_:)), for: .valueChanged)
        servoPositionSlidersByID[id] = positionSlider

        let readButton = makeServoActionButton(title: "Read", color: UIColor(white: 0.35, alpha: 1), action: #selector(readScannedServoTapped(_:)), id: id)
        let moveButton = makeServoActionButton(title: "Move", color: UIColor(red: 0.2, green: 0.5, blue: 1.0, alpha: 1), action: #selector(moveScannedServoTapped(_:)), id: id)
        let torqueOnButton = makeServoActionButton(title: "Torque On", color: UIColor(red: 0.2, green: 0.6, blue: 0.3, alpha: 1), action: #selector(enableScannedServoTorqueTapped(_:)), id: id)
        let torqueOffButton = makeServoActionButton(title: "Torque Off", color: UIColor(red: 0.8, green: 0.2, blue: 0.2, alpha: 1), action: #selector(disableScannedServoTorqueTapped(_:)), id: id)

        let newIDField = UITextField()
        let suggestedID = min(253, Int(id) + 1)
        configureServoTextField(newIDField, placeholder: "New ID", text: "\(suggestedID)")
        newIDField.tag = Int(id)
        servoNewIDFieldsByID[id] = newIDField

        let setIDButton = makeServoActionButton(title: "Set ID", color: UIColor(red: 0.2, green: 0.6, blue: 0.3, alpha: 1), action: #selector(changeScannedServoIDTapped(_:)), id: id)

        stack.addArrangedSubview(titleLabel)
    stack.addArrangedSubview(stateLabel)
    stack.addArrangedSubview(positionLabel)
    stack.addArrangedSubview(positionSlider)
        stack.addArrangedSubview(makeServoRow([readButton, moveButton]))
        stack.addArrangedSubview(makeServoRow([torqueOnButton, torqueOffButton]))
        stack.addArrangedSubview(makeServoRow([newIDField, setIDButton]))

        NSLayoutConstraint.activate([
            stack.topAnchor.constraint(equalTo: card.topAnchor, constant: 12),
            stack.leadingAnchor.constraint(equalTo: card.leadingAnchor, constant: 12),
            stack.trailingAnchor.constraint(equalTo: card.trailingAnchor, constant: -12),
            stack.bottomAnchor.constraint(equalTo: card.bottomAnchor, constant: -12),
        ])

        return card
    }

    private func makeServoActionButton(title: String, color: UIColor, action: Selector, id: UInt8) -> UIButton {
        let button = UIButton(type: .custom)
        configureServoButton(button, title: title, color: color, action: action)
        button.tag = Int(id)
        return button
    }

    private func servoStateText(for id: UInt8) -> String {
        guard let state = servoStatesByID[id], !state.isReadFailure else {
            return "Not read yet"
        }
        return "Pos \(state.position)  Load \(state.load)  Temp \(state.temperature)  Volt \(state.voltage)"
    }

    private func positionForScannedServo(id: UInt8) -> UInt16 {
        let position = Int(servoPositionSlidersByID[id]?.value.rounded() ?? Float(servoPosition()))
        return UInt16(min(4095, max(0, position)))
    }

    private func updateServoRowsEnabled(_ enabled: Bool) {
        servoListStack.arrangedSubviews.forEach { row in
            row.alpha = enabled ? 1.0 : 0.4
            row.isUserInteractionEnabled = enabled
        }
    }

    private func updateServoList(_ ids: [UInt8]) {
        scannedServoIDs = ids.reduce(into: []) { uniqueIDs, id in
            if !uniqueIDs.contains(id) {
                uniqueIDs.append(id)
            }
        }
        rebuildServoRows()
        guard !scannedServoIDs.isEmpty else {
            setServoStatus("No servos found")
            return
        }
        setServoStatus("\(scannedServoIDs.count) servo\(scannedServoIDs.count == 1 ? "" : "s") scanned")
        refreshScannedServoStates()
    }

    private func updateServoState(_ state: ServoState) {
        servoStatesByID[state.id] = state
        if state.isReadFailure {
            setServoStatus("ID \(state.id) read failed", isError: true)
            servoStateLabelsByID[state.id]?.text = "Read failed"
        } else {
            setServoStatus("ID \(state.id) pos \(state.position) load \(state.load) temp \(state.temperature)")
            servoPositionSlidersByID[state.id]?.setValue(Float(state.position), animated: true)
            servoPositionLabelsByID[state.id]?.text = "Position \(state.position)"
            servoStateLabelsByID[state.id]?.text = servoStateText(for: state.id)
            UserDefaults.standard.set(Int(state.position), forKey: Self.cachedServoPositionKey)
        }
    }

    private func refreshScannedServoStates(after delay: TimeInterval = 0.2) {
        scannedServoIDs.enumerated().forEach { index, id in
            DispatchQueue.main.asyncAfter(deadline: .now() + delay + (Double(index) * 0.15)) { [weak self] in
                self?.ble.refreshServoState(id: id)
            }
        }
    }
    
    private func setupWiFiConfig() {
        // Section label
        wifiLabel.translatesAutoresizingMaskIntoConstraints = false
        wifiLabel.text = "WiFi Configuration"
        wifiLabel.font = .systemFont(ofSize: 14, weight: .semibold)
        wifiLabel.textColor = UIColor.white.withAlphaComponent(0.6)
        contentView.addSubview(wifiLabel)
        
        // SSID field
        ssidField.translatesAutoresizingMaskIntoConstraints = false
        ssidField.placeholder = "Network Name (SSID)"
        ssidField.font = .systemFont(ofSize: 15)
        ssidField.textColor = .white
        ssidField.backgroundColor = UIColor(white: 0.2, alpha: 1)
        ssidField.layer.cornerRadius = 8
        ssidField.autocorrectionType = .no
        ssidField.autocapitalizationType = .none
        ssidField.returnKeyType = .next
        ssidField.leftView = UIView(frame: CGRect(x: 0, y: 0, width: 12, height: 0))
        ssidField.leftViewMode = .always
        ssidField.attributedPlaceholder = NSAttributedString(
            string: "Network Name (SSID)",
            attributes: [.foregroundColor: UIColor(white: 0.5, alpha: 1)]
        )
        ssidField.delegate = self
        contentView.addSubview(ssidField)
        
        // Password field
        passwordField.translatesAutoresizingMaskIntoConstraints = false
        passwordField.placeholder = "Password"
        passwordField.font = .systemFont(ofSize: 15)
        passwordField.textColor = .white
        passwordField.backgroundColor = UIColor(white: 0.2, alpha: 1)
        passwordField.layer.cornerRadius = 8
        passwordField.isSecureTextEntry = true
        passwordField.autocorrectionType = .no
        passwordField.autocapitalizationType = .none
        passwordField.returnKeyType = .send
        passwordField.leftView = UIView(frame: CGRect(x: 0, y: 0, width: 12, height: 0))
        passwordField.leftViewMode = .always
        passwordField.attributedPlaceholder = NSAttributedString(
            string: "Password",
            attributes: [.foregroundColor: UIColor(white: 0.5, alpha: 1)]
        )
        passwordField.delegate = self
        contentView.addSubview(passwordField)
        
        // Send button
        sendWifiButton.translatesAutoresizingMaskIntoConstraints = false
        sendWifiButton.setTitle("Save to ESP32", for: .normal)
        sendWifiButton.titleLabel?.font = .systemFont(ofSize: 15, weight: .semibold)
        sendWifiButton.setTitleColor(.white, for: .normal)
        sendWifiButton.backgroundColor = UIColor(red: 0.2, green: 0.6, blue: 0.3, alpha: 1)
        sendWifiButton.layer.cornerRadius = 8
        sendWifiButton.contentEdgeInsets = UIEdgeInsets(top: 10, left: 20, bottom: 10, right: 20)
        sendWifiButton.addTarget(self, action: #selector(sendWifiTapped), for: .touchUpInside)
        contentView.addSubview(sendWifiButton)
        
        // Status label
        wifiStatusLabel.translatesAutoresizingMaskIntoConstraints = false
        wifiStatusLabel.font = .systemFont(ofSize: 13, weight: .medium)
        wifiStatusLabel.textColor = UIColor.white.withAlphaComponent(0.5)
        wifiStatusLabel.text = ""
        wifiStatusLabel.textAlignment = .center
        contentView.addSubview(wifiStatusLabel)
        
        NSLayoutConstraint.activate([
            wifiLabel.topAnchor.constraint(equalTo: joystickView.bottomAnchor, constant: 24),
            wifiLabel.leadingAnchor.constraint(equalTo: contentView.leadingAnchor, constant: 20),
            
            ssidField.topAnchor.constraint(equalTo: wifiLabel.bottomAnchor, constant: 10),
            ssidField.leadingAnchor.constraint(equalTo: contentView.leadingAnchor, constant: 20),
            ssidField.trailingAnchor.constraint(equalTo: contentView.trailingAnchor, constant: -20),
            ssidField.heightAnchor.constraint(equalToConstant: 44),
            
            passwordField.topAnchor.constraint(equalTo: ssidField.bottomAnchor, constant: 10),
            passwordField.leadingAnchor.constraint(equalTo: contentView.leadingAnchor, constant: 20),
            passwordField.trailingAnchor.constraint(equalTo: contentView.trailingAnchor, constant: -20),
            passwordField.heightAnchor.constraint(equalToConstant: 44),
            
            sendWifiButton.topAnchor.constraint(equalTo: passwordField.bottomAnchor, constant: 14),
            sendWifiButton.centerXAnchor.constraint(equalTo: contentView.centerXAnchor),
            
            wifiStatusLabel.topAnchor.constraint(equalTo: sendWifiButton.bottomAnchor, constant: 8),
            wifiStatusLabel.centerXAnchor.constraint(equalTo: contentView.centerXAnchor)
        ])
        
        // WiFi write result callback
        ble.onWifiWriteResult = { [weak self] success in
            print("[WiFi] Write result callback — success: \(success)")
            DispatchQueue.main.async {
                if success {
                    self?.wifiStatusLabel.text = "✅ Saved! Will connect on next boot."
                    self?.wifiStatusLabel.textColor = UIColor(red: 0.2, green: 0.9, blue: 0.4, alpha: 1)
                } else {
                    self?.wifiStatusLabel.text = "❌ Failed to save credentials."
                    self?.wifiStatusLabel.textColor = UIColor(red: 1.0, green: 0.3, blue: 0.3, alpha: 1)
                }
                self?.sendWifiButton.isEnabled = true
                self?.sendWifiButton.alpha = 1.0
            }
        }
    }
    
    // MARK: - Telemetry Config
    
    private func setupTelemetryConfig() {
        // Section label
        telemetryLabel.translatesAutoresizingMaskIntoConstraints = false
        telemetryLabel.text = "Telemetry Server"
        telemetryLabel.font = .systemFont(ofSize: 14, weight: .semibold)
        telemetryLabel.textColor = UIColor.white.withAlphaComponent(0.6)
        contentView.addSubview(telemetryLabel)
        
        // Server URL field
        serverURLField.translatesAutoresizingMaskIntoConstraints = false
        serverURLField.font = .systemFont(ofSize: 15)
        serverURLField.textColor = .white
        serverURLField.backgroundColor = UIColor(white: 0.2, alpha: 1)
        serverURLField.layer.cornerRadius = 8
        serverURLField.autocorrectionType = .no
        serverURLField.autocapitalizationType = .none
        serverURLField.keyboardType = .URL
        serverURLField.returnKeyType = .done
        serverURLField.leftView = UIView(frame: CGRect(x: 0, y: 0, width: 12, height: 0))
        serverURLField.leftViewMode = .always
        serverURLField.text = TelemetryService.shared.serverURL
        serverURLField.attributedPlaceholder = NSAttributedString(
            string: "ws://192.168.1.100:8765",
            attributes: [.foregroundColor: UIColor(white: 0.5, alpha: 1)]
        )
        serverURLField.delegate = self
        contentView.addSubview(serverURLField)
        
        // Connect button
        telemetryButton.translatesAutoresizingMaskIntoConstraints = false
        telemetryButton.titleLabel?.font = .systemFont(ofSize: 15, weight: .semibold)
        telemetryButton.setTitleColor(.white, for: .normal)
        telemetryButton.layer.cornerRadius = 8
        telemetryButton.contentEdgeInsets = UIEdgeInsets(top: 10, left: 20, bottom: 10, right: 20)
        telemetryButton.addTarget(self, action: #selector(telemetryButtonTapped), for: .touchUpInside)
        contentView.addSubview(telemetryButton)
        
        // Status dot
        telemetryStatusDot.translatesAutoresizingMaskIntoConstraints = false
        telemetryStatusDot.layer.cornerRadius = 5
        contentView.addSubview(telemetryStatusDot)
        
        // Status label
        telemetryStatusLabel.translatesAutoresizingMaskIntoConstraints = false
        telemetryStatusLabel.font = .systemFont(ofSize: 13, weight: .medium)
        telemetryStatusLabel.textColor = UIColor.white.withAlphaComponent(0.5)
        contentView.addSubview(telemetryStatusLabel)
        
        NSLayoutConstraint.activate([
            telemetryLabel.topAnchor.constraint(equalTo: wifiStatusLabel.bottomAnchor, constant: 24),
            telemetryLabel.leadingAnchor.constraint(equalTo: contentView.leadingAnchor, constant: 20),
            
            serverURLField.topAnchor.constraint(equalTo: telemetryLabel.bottomAnchor, constant: 10),
            serverURLField.leadingAnchor.constraint(equalTo: contentView.leadingAnchor, constant: 20),
            serverURLField.trailingAnchor.constraint(equalTo: contentView.trailingAnchor, constant: -20),
            serverURLField.heightAnchor.constraint(equalToConstant: 44),
            
            telemetryButton.topAnchor.constraint(equalTo: serverURLField.bottomAnchor, constant: 14),
            telemetryButton.centerXAnchor.constraint(equalTo: contentView.centerXAnchor),
            
            telemetryStatusDot.leadingAnchor.constraint(equalTo: contentView.leadingAnchor, constant: 20),
            telemetryStatusDot.topAnchor.constraint(equalTo: telemetryButton.bottomAnchor, constant: 12),
            telemetryStatusDot.widthAnchor.constraint(equalToConstant: 10),
            telemetryStatusDot.heightAnchor.constraint(equalToConstant: 10),
            
            telemetryStatusLabel.leadingAnchor.constraint(equalTo: telemetryStatusDot.trailingAnchor, constant: 8),
            telemetryStatusLabel.centerYAnchor.constraint(equalTo: telemetryStatusDot.centerYAnchor),
            telemetryStatusLabel.trailingAnchor.constraint(lessThanOrEqualTo: contentView.trailingAnchor, constant: -20),
            telemetryStatusLabel.bottomAnchor.constraint(equalTo: contentView.bottomAnchor, constant: -20)
        ])
        
        // Listen for connection status
        TelemetryService.shared.onConnectionStatusChanged = { [weak self] connected, url in
            self?.updateTelemetryUI(connected: connected, url: url)
        }
        
        updateTelemetryUI(connected: false, url: TelemetryService.shared.serverURL)
    }
    
    @objc private func telemetryButtonTapped() {
        dismissKeyboard()
        
        let service = TelemetryService.shared
        
        // If already running, stop first
        if service.isRunning {
            service.stop(reason: "user_restart")
        }
        
        // Read the URL and (re)connect
        let url = serverURLField.text?.trimmingCharacters(in: .whitespacesAndNewlines) ?? ""
        guard !url.isEmpty else {
            telemetryStatusLabel.text = "Enter a server URL"
            telemetryStatusLabel.textColor = UIColor(red: 1.0, green: 0.8, blue: 0.2, alpha: 1)
            updateTelemetryUI(connected: false, url: "")
            return
        }
        service.serverURL = url
        service.start()
        telemetryStatusLabel.text = "Connecting…"
        telemetryStatusLabel.textColor = UIColor.white.withAlphaComponent(0.5)
        telemetryStatusDot.backgroundColor = .yellow
    }
    
    private func updateTelemetryUI(connected: Bool, url: String) {
        let running = TelemetryService.shared.isRunning
        
        if running {
            telemetryButton.setTitle("Reconnect", for: .normal)
            telemetryButton.backgroundColor = UIColor(red: 0.4, green: 0.2, blue: 0.8, alpha: 1)
        } else {
            telemetryButton.setTitle("Connect", for: .normal)
            telemetryButton.backgroundColor = UIColor(red: 0.4, green: 0.2, blue: 0.8, alpha: 1)
        }
        
        if connected {
            telemetryStatusDot.backgroundColor = .green
            telemetryStatusLabel.text = "Connected to \(url)"
            telemetryStatusLabel.textColor = UIColor(red: 0.2, green: 0.9, blue: 0.4, alpha: 1)
        } else if running {
            telemetryStatusDot.backgroundColor = .yellow
            telemetryStatusLabel.text = "Reconnecting…"
            telemetryStatusLabel.textColor = UIColor(red: 1.0, green: 0.8, blue: 0.2, alpha: 1)
        } else {
            telemetryStatusDot.backgroundColor = .gray
            telemetryStatusLabel.text = "Not connected"
            telemetryStatusLabel.textColor = UIColor.white.withAlphaComponent(0.5)
        }
    }
    
    @objc private func sendWifiTapped() {
        dismissKeyboard()
        
        let ssid = ssidField.text?.trimmingCharacters(in: .whitespacesAndNewlines) ?? ""
        guard !ssid.isEmpty else {
            print("[WiFi] Send tapped but SSID is empty")
            wifiStatusLabel.text = "Enter an SSID"
            wifiStatusLabel.textColor = UIColor(red: 1.0, green: 0.8, blue: 0.2, alpha: 1)
            return
        }
        
        let password = passwordField.text ?? ""
        print("[WiFi] Sending credentials — SSID: \"\(ssid)\", password length: \(password.count)")
        
        sendWifiButton.isEnabled = false
        sendWifiButton.alpha = 0.5
        wifiStatusLabel.text = "Sending…"
        wifiStatusLabel.textColor = UIColor.white.withAlphaComponent(0.5)
        
        ble.setWiFiCredentials(ssid: ssid, password: password)
    }

    @objc private func showServoSettingsTapped() {
        dismissKeyboard()
        cleanupServoSettingsPopup()

        let settingsViewController = UIViewController()
        settingsViewController.view.backgroundColor = UIColor(white: 0.12, alpha: 1)
        settingsViewController.modalPresentationStyle = .pageSheet

        let scrollView = UIScrollView()
        scrollView.translatesAutoresizingMaskIntoConstraints = false
        scrollView.showsVerticalScrollIndicator = true
        scrollView.alwaysBounceVertical = true
        scrollView.keyboardDismissMode = .interactive
        settingsViewController.view.addSubview(scrollView)

        let contentView = UIView()
        contentView.translatesAutoresizingMaskIntoConstraints = false
        scrollView.addSubview(contentView)

        let titleLabel = UILabel()
        titleLabel.translatesAutoresizingMaskIntoConstraints = false
        titleLabel.text = "ST3215 Servos"
        titleLabel.font = .systemFont(ofSize: 20, weight: .bold)
        titleLabel.textColor = .white
        contentView.addSubview(titleLabel)

        let closeButton = UIButton(type: .system)
        closeButton.translatesAutoresizingMaskIntoConstraints = false
        closeButton.setTitle("Done", for: .normal)
        closeButton.titleLabel?.font = .systemFont(ofSize: 16, weight: .semibold)
        closeButton.addTarget(self, action: #selector(dismissServoSettingsTapped), for: .touchUpInside)
        contentView.addSubview(closeButton)

        let listLabel = UILabel()
        listLabel.translatesAutoresizingMaskIntoConstraints = false
        listLabel.text = "Scanned Servos"
        listLabel.font = .systemFont(ofSize: 14, weight: .semibold)
        listLabel.textColor = UIColor.white.withAlphaComponent(0.6)
        contentView.addSubview(listLabel)

        rebuildServoRows()
        contentView.addSubview(servoStack)

        servoPopupConstraints = [
            scrollView.topAnchor.constraint(equalTo: settingsViewController.view.safeAreaLayoutGuide.topAnchor),
            scrollView.leadingAnchor.constraint(equalTo: settingsViewController.view.leadingAnchor),
            scrollView.trailingAnchor.constraint(equalTo: settingsViewController.view.trailingAnchor),
            scrollView.bottomAnchor.constraint(equalTo: settingsViewController.view.bottomAnchor),

            contentView.topAnchor.constraint(equalTo: scrollView.contentLayoutGuide.topAnchor),
            contentView.leadingAnchor.constraint(equalTo: scrollView.contentLayoutGuide.leadingAnchor),
            contentView.trailingAnchor.constraint(equalTo: scrollView.contentLayoutGuide.trailingAnchor),
            contentView.bottomAnchor.constraint(equalTo: scrollView.contentLayoutGuide.bottomAnchor),
            contentView.widthAnchor.constraint(equalTo: scrollView.frameLayoutGuide.widthAnchor),

            titleLabel.topAnchor.constraint(equalTo: contentView.topAnchor, constant: 20),
            titleLabel.leadingAnchor.constraint(equalTo: contentView.leadingAnchor, constant: 20),

            closeButton.centerYAnchor.constraint(equalTo: titleLabel.centerYAnchor),
            closeButton.trailingAnchor.constraint(equalTo: contentView.trailingAnchor, constant: -20),

            listLabel.topAnchor.constraint(equalTo: titleLabel.bottomAnchor, constant: 18),
            listLabel.leadingAnchor.constraint(equalTo: contentView.leadingAnchor, constant: 20),

            servoStack.topAnchor.constraint(equalTo: listLabel.bottomAnchor, constant: 8),
            servoStack.leadingAnchor.constraint(equalTo: contentView.leadingAnchor, constant: 20),
            servoStack.trailingAnchor.constraint(equalTo: contentView.trailingAnchor, constant: -20),
            servoStack.bottomAnchor.constraint(equalTo: contentView.bottomAnchor, constant: -24),
        ]
        NSLayoutConstraint.activate(servoPopupConstraints)

        if let sheet = settingsViewController.sheetPresentationController {
            sheet.detents = [.medium(), .large()]
            sheet.prefersGrabberVisible = true
            sheet.prefersScrollingExpandsWhenScrolledToEdge = true
        }
        settingsViewController.presentationController?.delegate = self
        servoSettingsViewController = settingsViewController
        present(settingsViewController, animated: true)
    }

    @objc private func dismissServoSettingsTapped() {
        servoSettingsViewController?.dismiss(animated: true) { [weak self] in
            self?.cleanupServoSettingsPopup()
        }
    }

    private func cleanupServoSettingsPopup() {
        NSLayoutConstraint.deactivate(servoPopupConstraints)
        servoPopupConstraints.removeAll()
        servoStack.removeFromSuperview()
        servoSettingsViewController = nil
    }
    
    @objc private func dismissKeyboard() {
        view.endEditing(true)
    }
    
    @objc private func keyboardWillShow(_ notification: Notification) {
        guard let keyboardFrame = notification.userInfo?[UIResponder.keyboardFrameEndUserInfoKey] as? CGRect,
              let duration = notification.userInfo?[UIResponder.keyboardAnimationDurationUserInfoKey] as? Double else { return }
        let keyboardHeight = keyboardFrame.height - view.safeAreaInsets.bottom
        UIView.animate(withDuration: duration) {
            self.scrollView.contentInset.bottom = keyboardHeight
            self.scrollView.verticalScrollIndicatorInsets.bottom = keyboardHeight
        }
        // Scroll the active field into view
        if let activeField = view.findFirstResponder() as? UIView {
            let rect = activeField.convert(activeField.bounds, to: scrollView)
            scrollView.scrollRectToVisible(rect.insetBy(dx: 0, dy: -20), animated: true)
        }
    }
    
    @objc private func keyboardWillHide(_ notification: Notification) {
        guard let duration = notification.userInfo?[UIResponder.keyboardAnimationDurationUserInfoKey] as? Double else { return }
        UIView.animate(withDuration: duration) {
            self.scrollView.contentInset.bottom = 0
            self.scrollView.verticalScrollIndicatorInsets.bottom = 0
        }
    }
    
    // MARK: - Sheet gesture control
    
    /// Walk the view hierarchy above this VC's view to find and enable/disable
    /// pan gesture recognizers that drive the sheet dismissal.
    private func setSheetGesturesEnabled(_ enabled: Bool) {
        var current = view.superview
        while let v = current {
            for gr in v.gestureRecognizers ?? [] where gr is UIPanGestureRecognizer {
                gr.isEnabled = enabled
            }
            current = v.superview
        }
    }
    
    // MARK: - Actions
    
    @objc private func connectTapped() {
        ble.toggleConnection()
    }
    
    @objc private func rescanServosTapped() {
        dismissKeyboard()
        guard let from = boundedServoNumber(from: servoScanFromField, name: "From", range: 0...253),
              let to = boundedServoNumber(from: servoScanToField, name: "To", range: 0...253) else { return }
        guard from == 0 || to == 0 || from <= to else {
            setServoStatus("Scan range is reversed", isError: true)
            return
        }
        ble.rescanServos(from: UInt8(from), to: UInt8(to))
        setServoStatus("Scanning \(from)-\(to)")
    }

    @objc private func changeServoIDTapped() {
        dismissKeyboard()
        guard let currentID = servoID(from: servoIDField, name: "ID"),
              let newID = servoID(from: servoNewIDField, name: "New ID") else { return }
        UserDefaults.standard.set(Int(currentID), forKey: Self.cachedServoIDKey)
        UserDefaults.standard.set(Int(newID), forKey: Self.cachedServoNewIDKey)
        ble.changeServoID(currentID: currentID, newID: newID)
        setServoStatus("Set ID \(currentID) -> \(newID)")
    }

    @objc private func servoPositionChanged() {
        let position = Int(servoPositionSlider.value.rounded())
        servoPositionLabel.text = "Position \(position)"
        UserDefaults.standard.set(position, forKey: Self.cachedServoPositionKey)
    }

    @objc private func scannedServoPositionChanged(_ sender: UISlider) {
        let id = UInt8(sender.tag)
        let position = Int(sender.value.rounded())
        sender.setValue(Float(position), animated: false)
        servoPositionLabelsByID[id]?.text = "Position \(position)"
        UserDefaults.standard.set(position, forKey: Self.cachedServoPositionKey)
    }

    @objc private func refreshServoStateTapped() {
        dismissKeyboard()
        guard let id = servoID(from: servoIDField, name: "ID") else { return }
        UserDefaults.standard.set(Int(id), forKey: Self.cachedServoIDKey)
        ble.refreshServoState(id: id)
        setServoStatus("Reading ID \(id)")
    }

    @objc private func moveServoTapped() {
        dismissKeyboard()
        guard let id = servoID(from: servoIDField, name: "ID"), let speed = servoSpeed() else { return }
        UserDefaults.standard.set(Int(id), forKey: Self.cachedServoIDKey)
        ble.moveServo(id: id, position: servoPosition(), speed: speed)
        setServoStatus("Moving ID \(id)")
    }

    @objc private func moveAllServosTapped() {
        dismissKeyboard()
        guard let speed = servoSpeed() else { return }
        guard !scannedServoIDs.isEmpty else {
            setServoStatus("Scan servos first", isError: true)
            return
        }
        scannedServoIDs.forEach { id in
            ble.moveServo(id: id, position: positionForScannedServo(id: id), speed: speed)
        }
        setServoStatus("Moving \(scannedServoIDs.count) servos")
        refreshScannedServoStates(after: 0.4)
    }

    @objc private func readScannedServoTapped(_ sender: UIButton) {
        let id = UInt8(sender.tag)
        ble.refreshServoState(id: id)
        setServoStatus("Reading ID \(id)")
    }

    @objc private func moveScannedServoTapped(_ sender: UIButton) {
        dismissKeyboard()
        let id = UInt8(sender.tag)
        guard let speed = servoSpeed() else { return }
        ble.moveServo(id: id, position: positionForScannedServo(id: id), speed: speed)
        setServoStatus("Moving ID \(id)")
        DispatchQueue.main.asyncAfter(deadline: .now() + 0.35) { [weak self] in
            self?.ble.refreshServoState(id: id)
        }
    }

    @objc private func changeScannedServoIDTapped(_ sender: UIButton) {
        dismissKeyboard()
        let currentID = UInt8(sender.tag)
        guard let newIDField = servoNewIDFieldsByID[currentID],
              let newID = servoID(from: newIDField, name: "New ID") else { return }
        ble.changeServoID(currentID: currentID, newID: newID)
        setServoStatus("Set ID \(currentID) -> \(newID); scan to refresh")
        DispatchQueue.main.asyncAfter(deadline: .now() + 0.5) { [weak self] in
            self?.rescanServosTapped()
        }
    }

    @objc private func enableScannedServoTorqueTapped(_ sender: UIButton) {
        setScannedServoTorque(id: UInt8(sender.tag), enabled: true)
    }

    @objc private func disableScannedServoTorqueTapped(_ sender: UIButton) {
        setScannedServoTorque(id: UInt8(sender.tag), enabled: false)
    }

    @objc private func enableServoTorqueTapped() {
        setServoTorque(enabled: true)
    }

    @objc private func disableServoTorqueTapped() {
        setServoTorque(enabled: false)
    }

    @objc private func enableAllServoTorqueTapped() {
        setAllServoTorque(enabled: true)
    }

    @objc private func disableAllServoTorqueTapped() {
        setAllServoTorque(enabled: false)
    }

    private func setServoTorque(enabled: Bool) {
        dismissKeyboard()
        guard let id = servoID(from: servoIDField, name: "ID") else { return }
        UserDefaults.standard.set(Int(id), forKey: Self.cachedServoIDKey)
        ble.setServoTorque(id: id, enabled: enabled)
        setServoStatus("Torque \(enabled ? "on" : "off") for ID \(id)")
    }

    private func setScannedServoTorque(id: UInt8, enabled: Bool) {
        ble.setServoTorque(id: id, enabled: enabled)
        setServoStatus("Torque \(enabled ? "on" : "off") for ID \(id)")
        DispatchQueue.main.asyncAfter(deadline: .now() + 0.25) { [weak self] in
            self?.ble.refreshServoState(id: id)
        }
    }

    private func setAllServoTorque(enabled: Bool) {
        dismissKeyboard()
        guard !scannedServoIDs.isEmpty else {
            setServoStatus("Scan servos first", isError: true)
            return
        }
        scannedServoIDs.forEach { id in
            ble.setServoTorque(id: id, enabled: enabled)
        }
        setServoStatus("Torque \(enabled ? "on" : "off") for \(scannedServoIDs.count) servos")
        refreshScannedServoStates(after: 0.3)
    }
    
    // MARK: - UI updates
    
    private func updateConnectionUI(_ state: ESP32ConnectionState) {
        let color = state.color
        let uiColor = UIColor(red: color.r, green: color.g, blue: color.b, alpha: 1)
        
        statusDot.backgroundColor = uiColor
        statusLabel.text = state.displayText
        
        if let name = ble.peripheralName, state == .connected {
            statusLabel.text = name
        }
        
        switch state {
        case .disconnected:
            connectButton.setTitle("Connect", for: .normal)
            connectButton.backgroundColor = UIColor(red: 0.2, green: 0.5, blue: 1.0, alpha: 1)
        case .scanning, .connecting:
            connectButton.setTitle("Cancel", for: .normal)
            connectButton.backgroundColor = UIColor(red: 0.5, green: 0.5, blue: 0.5, alpha: 1)
        case .connected:
            connectButton.setTitle("Disconnect", for: .normal)
            connectButton.backgroundColor = UIColor(red: 0.8, green: 0.2, blue: 0.2, alpha: 1)
        }
        
        // Enable/disable joystick visual feedback
        let connected = state == .connected
        joystickView.alpha = connected ? 1.0 : 0.4
        joystickView.isUserInteractionEnabled = connected
        servoSettingsButton.isUserInteractionEnabled = connected
        servoSettingsButton.alpha = connected ? 1.0 : 0.4
        servoControlViews.forEach { view in
            view.isUserInteractionEnabled = connected
            view.alpha = connected ? 1.0 : 0.4
        }
        if !connected {
            scannedServoIDs.removeAll()
            rebuildServoRows()
        } else {
            updateServoRowsEnabled(true)
        }
        
        // Enable/disable WiFi controls
        ssidField.isEnabled = connected
        passwordField.isEnabled = connected
        sendWifiButton.isEnabled = connected
        ssidField.alpha = connected ? 1.0 : 0.4
        passwordField.alpha = connected ? 1.0 : 0.4
        sendWifiButton.alpha = connected ? 1.0 : 0.4
    }
}

// MARK: - UIGestureRecognizerDelegate
extension ControlPanelViewController: UIGestureRecognizerDelegate {
    func gestureRecognizer(_ gestureRecognizer: UIGestureRecognizer, shouldReceive touch: UITouch) -> Bool {
        // Don't fire the dismiss-keyboard tap when touching a text field
        return !(touch.view is UITextField)
    }
}

// MARK: - UIAdaptivePresentationControllerDelegate
extension ControlPanelViewController: UIAdaptivePresentationControllerDelegate {
    func presentationControllerDidDismiss(_ presentationController: UIPresentationController) {
        cleanupServoSettingsPopup()
    }
}

// MARK: - UITextFieldDelegate
extension ControlPanelViewController: UITextFieldDelegate {
    func textFieldShouldReturn(_ textField: UITextField) -> Bool {
        if textField == ssidField {
            passwordField.becomeFirstResponder()
        } else if textField == passwordField {
            sendWifiTapped()
        } else if textField == serverURLField {
            telemetryButtonTapped()
        }
        return true
    }
}

// MARK: - UIView first responder helper
private extension UIView {
    func findFirstResponder() -> UIResponder? {
        if isFirstResponder { return self }
        for sub in subviews {
            if let responder = sub.findFirstResponder() { return responder }
        }
        return nil
    }
}