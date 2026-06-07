//
//  RemoteControlViewController.swift
//  RoboCar
//

import UIKit
import Network
import WebRTC

final class RemoteControlViewController: UIViewController {
    private let client = RemoteControlClientService()
    private let browser = RemoteControlBrowser()
    private let remoteGrid = OccupancyGrid(cellSize: 0.05, gridRadius: 500)
    private let keyboardDriveState = KeyboardDriveState()

    private let cameraVideoView = RTCMTLVideoView()
    private let videoFallbackImageView = UIImageView()
    private let mapView: GridMapView
    private let statusLabel = UILabel()
    private let runButton = UIButton(type: .system)
    private let pauseButton = UIButton(type: .system)
    private let clearButton = UIButton(type: .system)
    private let settingsButton = UIButton(type: .system)

    private var compactContentConstraints: [NSLayoutConstraint] = []
    private var wideContentConstraints: [NSLayoutConstraint] = []
    private var usesWideContentLayout = false

    private var discoveredHosts: [RemoteControlDiscoveredHost] = []
    private weak var settingsViewController: RemoteControlSettingsViewController?

    override var canBecomeFirstResponder: Bool { true }

    init() {
        mapView = GridMapView(occupancyGrid: remoteGrid)
        super.init(nibName: nil, bundle: nil)
    }

    required init?(coder: NSCoder) {
        fatalError("init(coder:) has not been implemented")
    }

    override func viewDidLoad() {
        super.viewDidLoad()
        view.backgroundColor = .black
        setupUI()
        setupNetworking()
        browser.start()
    }

    override func viewWillDisappear(_ animated: Bool) {
        super.viewWillDisappear(animated)
        stopKeyboardDriveIfNeeded()
        client.disconnect()
        browser.stop()
    }

    override func viewDidAppear(_ animated: Bool) {
        super.viewDidAppear(animated)
        becomeFirstResponder()
    }

    override func viewDidLayoutSubviews() {
        super.viewDidLayoutSubviews()
        updateContentLayoutIfNeeded()
    }

    override func pressesBegan(_ presses: Set<UIPress>, with event: UIPressesEvent?) {
        if !view.hasFirstResponderTextInput, handleKeyboardDrive(keyboardDriveState.pressesBegan(presses)) {
            return
        }
        super.pressesBegan(presses, with: event)
    }

    override func pressesEnded(_ presses: Set<UIPress>, with event: UIPressesEvent?) {
        if !view.hasFirstResponderTextInput, handleKeyboardDrive(keyboardDriveState.pressesEnded(presses)) {
            return
        }
        super.pressesEnded(presses, with: event)
    }

    override func pressesCancelled(_ presses: Set<UIPress>, with event: UIPressesEvent?) {
        if !view.hasFirstResponderTextInput, handleKeyboardDrive(keyboardDriveState.pressesEnded(presses)) {
            return
        }
        super.pressesCancelled(presses, with: event)
    }

    private func setupUI() {
        cameraVideoView.translatesAutoresizingMaskIntoConstraints = false
        cameraVideoView.backgroundColor = UIColor(white: 0.06, alpha: 1)
        cameraVideoView.videoContentMode = .scaleAspectFill
        cameraVideoView.clipsToBounds = true
        view.addSubview(cameraVideoView)

        videoFallbackImageView.translatesAutoresizingMaskIntoConstraints = false
        videoFallbackImageView.backgroundColor = .clear
        videoFallbackImageView.contentMode = .scaleAspectFill
        videoFallbackImageView.clipsToBounds = true
        videoFallbackImageView.isHidden = true
        view.addSubview(videoFallbackImageView)

        mapView.translatesAutoresizingMaskIntoConstraints = false
        mapView.backgroundColor = UIColor(white: 0.1, alpha: 1)
        mapView.onTapWorldPosition = { [weak self] x, y in
            var message = RemoteMessage(type: "addRoutePoint")
            message.x = x
            message.y = y
            self?.client.send(message)
        }
        view.addSubview(mapView)

        statusLabel.translatesAutoresizingMaskIntoConstraints = false
        statusLabel.textColor = .white
        statusLabel.font = .monospacedSystemFont(ofSize: 13, weight: .medium)
        statusLabel.numberOfLines = 2
        statusLabel.text = "Discovering RoboCar..."
        view.addSubview(statusLabel)

        configureGlassButton(runButton, title: "Run", systemImageName: "play.fill")
        configureGlassButton(pauseButton, title: "Pause", systemImageName: "pause.fill")
        configureGlassButton(clearButton, title: "Clear", systemImageName: "trash")
        configureIconGlassButton(settingsButton, systemImageName: "gearshape.fill")
        runButton.addTarget(self, action: #selector(runRouteTapped), for: .touchUpInside)
        pauseButton.addTarget(self, action: #selector(pauseRouteTapped), for: .touchUpInside)
        clearButton.addTarget(self, action: #selector(clearRouteTapped), for: .touchUpInside)
        settingsButton.addTarget(self, action: #selector(settingsTapped), for: .touchUpInside)
        [runButton, pauseButton, clearButton, settingsButton].forEach(view.addSubview)

        NSLayoutConstraint.activate([
            statusLabel.topAnchor.constraint(equalTo: view.safeAreaLayoutGuide.topAnchor, constant: 12),
            statusLabel.leadingAnchor.constraint(equalTo: view.leadingAnchor, constant: 16),
            statusLabel.trailingAnchor.constraint(equalTo: settingsButton.leadingAnchor, constant: -12),

            settingsButton.topAnchor.constraint(equalTo: view.safeAreaLayoutGuide.topAnchor, constant: 10),
            settingsButton.trailingAnchor.constraint(equalTo: view.trailingAnchor, constant: -16),
            settingsButton.widthAnchor.constraint(equalToConstant: 44),
            settingsButton.heightAnchor.constraint(equalToConstant: 36),

            runButton.leadingAnchor.constraint(equalTo: view.leadingAnchor, constant: 16),
            runButton.bottomAnchor.constraint(equalTo: view.safeAreaLayoutGuide.bottomAnchor, constant: -10),
            runButton.heightAnchor.constraint(equalToConstant: 40),

            pauseButton.leadingAnchor.constraint(equalTo: runButton.trailingAnchor, constant: 10),
            pauseButton.centerYAnchor.constraint(equalTo: runButton.centerYAnchor),
            pauseButton.widthAnchor.constraint(equalTo: runButton.widthAnchor),
            pauseButton.heightAnchor.constraint(equalTo: runButton.heightAnchor),

            clearButton.leadingAnchor.constraint(equalTo: pauseButton.trailingAnchor, constant: 10),
            clearButton.trailingAnchor.constraint(equalTo: view.trailingAnchor, constant: -16),
            clearButton.centerYAnchor.constraint(equalTo: runButton.centerYAnchor),
            clearButton.widthAnchor.constraint(equalTo: runButton.widthAnchor),
            clearButton.heightAnchor.constraint(equalTo: runButton.heightAnchor)
        ])

        compactContentConstraints = [
            cameraVideoView.topAnchor.constraint(equalTo: statusLabel.bottomAnchor, constant: 12),
            cameraVideoView.leadingAnchor.constraint(equalTo: view.leadingAnchor),
            cameraVideoView.trailingAnchor.constraint(equalTo: view.trailingAnchor),
            cameraVideoView.heightAnchor.constraint(equalTo: view.heightAnchor, multiplier: 0.52),
            videoFallbackImageView.topAnchor.constraint(equalTo: cameraVideoView.topAnchor),
            videoFallbackImageView.leadingAnchor.constraint(equalTo: cameraVideoView.leadingAnchor),
            videoFallbackImageView.trailingAnchor.constraint(equalTo: cameraVideoView.trailingAnchor),
            videoFallbackImageView.bottomAnchor.constraint(equalTo: cameraVideoView.bottomAnchor),

            mapView.topAnchor.constraint(equalTo: cameraVideoView.bottomAnchor),
            mapView.leadingAnchor.constraint(equalTo: view.leadingAnchor),
            mapView.trailingAnchor.constraint(equalTo: view.trailingAnchor),
            mapView.bottomAnchor.constraint(equalTo: runButton.topAnchor, constant: -12)
        ]

        wideContentConstraints = [
            cameraVideoView.topAnchor.constraint(equalTo: statusLabel.bottomAnchor, constant: 12),
            cameraVideoView.leadingAnchor.constraint(equalTo: view.leadingAnchor),
            cameraVideoView.bottomAnchor.constraint(equalTo: runButton.topAnchor, constant: -12),
            cameraVideoView.widthAnchor.constraint(equalTo: view.widthAnchor, multiplier: 0.5),
            videoFallbackImageView.topAnchor.constraint(equalTo: cameraVideoView.topAnchor),
            videoFallbackImageView.leadingAnchor.constraint(equalTo: cameraVideoView.leadingAnchor),
            videoFallbackImageView.trailingAnchor.constraint(equalTo: cameraVideoView.trailingAnchor),
            videoFallbackImageView.bottomAnchor.constraint(equalTo: cameraVideoView.bottomAnchor),

            mapView.topAnchor.constraint(equalTo: cameraVideoView.topAnchor),
            mapView.leadingAnchor.constraint(equalTo: cameraVideoView.trailingAnchor),
            mapView.trailingAnchor.constraint(equalTo: view.trailingAnchor),
            mapView.bottomAnchor.constraint(equalTo: cameraVideoView.bottomAnchor)
        ]
        updateContentLayoutIfNeeded(force: true)
    }

    private func updateContentLayoutIfNeeded(force: Bool = false) {
        guard !compactContentConstraints.isEmpty else { return }
        let shouldUseWideLayout = view.bounds.width >= 760 && view.bounds.width > view.bounds.height
        guard force || shouldUseWideLayout != usesWideContentLayout else { return }
        usesWideContentLayout = shouldUseWideLayout
        NSLayoutConstraint.deactivate(shouldUseWideLayout ? compactContentConstraints : wideContentConstraints)
        NSLayoutConstraint.activate(shouldUseWideLayout ? wideContentConstraints : compactContentConstraints)
    }

    private func setupNetworking() {
        client.onStatusChanged = { [weak self] status in
            self?.statusLabel.text = status
        }
        client.onMessage = { [weak self] message in
            self?.handleRemoteMessage(message)
        }
        client.onVideoTrack = { [weak self] track in
            guard let self else { return }
            track.add(self.cameraVideoView)
        }
        client.onVideoFrameImage = { [weak self] image in
            self?.videoFallbackImageView.image = image
            self?.videoFallbackImageView.isHidden = false
        }
        browser.onStatusChanged = { [weak self] status in
            if self?.client.isConnected == false {
                self?.statusLabel.text = status
            }
        }
        browser.onHostsChanged = { [weak self] hosts in
            self?.discoveredHosts = hosts
            self?.settingsViewController?.updateDiscoveredHosts(hosts)
        }
    }

    private func handleRemoteMessage(_ message: RemoteMessage) {
        switch message.type {
        case "cameraFrame":
            break
        case "mapState":
            if let pose = message.pose {
                remoteGrid.devicePosition = DevicePosition(x: pose.x, y: pose.y, z: pose.z, heading: pose.heading)
            }
            mapView.routeWaypoints = message.routeWaypoints?.map { ($0.x, $0.y) } ?? mapView.routeWaypoints
            mapView.plannedPath = message.plannedPath?.map { ($0.x, $0.y) } ?? mapView.plannedPath
            mapView.routePreviewPaths = message.routePreviewPaths?.map { segment in
                segment.map { ($0.x, $0.y) }
            } ?? mapView.routePreviewPaths
            mapView.activeRouteWaypointIndex = message.activeRouteWaypointIndex ?? mapView.activeRouteWaypointIndex
            mapView.setNeedsDisplay()
            let ble = message.bleConnected == true ? "BLE connected" : "BLE disconnected"
            statusLabel.text = "\(message.navState ?? "remote") - \(ble)"
        case "gridUpdate":
            applyGridUpdate(message.grid)
        case "servoList", "servoState", "status":
            break
        default:
            break
        }
    }

    private func applyGridUpdate(_ update: RemoteGridUpdate?) {
        guard let update else { return }
        var freeCells: [(x: Float, y: Float, classification: MeshClassification)] = []
        var occupiedCells: [(x: Float, y: Float, height: Float, classification: MeshClassification)] = []
        freeCells.reserveCapacity(update.cells.count)
        occupiedCells.reserveCapacity(update.cells.count)

        for cell in update.cells {
            let classification = MeshClassification(rawValue: cell.classification) ?? .none
            switch CellState(rawValue: cell.state) ?? .unknown {
            case .free:
                freeCells.append((x: cell.x, y: cell.y, classification: classification))
            case .occupied:
                occupiedCells.append((x: cell.x, y: cell.y, height: cell.height, classification: classification))
            case .unknown:
                break
            }
        }

        if !freeCells.isEmpty {
            remoteGrid.markFreeBatchWithClassification(freeCells)
        }
        if !occupiedCells.isEmpty {
            remoteGrid.markOccupiedBatchWithClassification(occupiedCells)
        }
        mapView.setNeedsDisplay()
    }

    @objc private func runRouteTapped() {
        client.send(RemoteMessage(type: "runRoute"))
    }

    @objc private func pauseRouteTapped() {
        client.send(RemoteMessage(type: "pauseRoute"))
    }

    @objc private func clearRouteTapped() {
        mapView.clearRouteWaypoints()
        client.send(RemoteMessage(type: "clearRoute"))
    }

    @objc private func settingsTapped() {
        let settings = RemoteControlSettingsViewController(client: client, discoveredHosts: discoveredHosts)
        settingsViewController = settings
        settings.preferredContentSize = CGSize(width: 620, height: 760)
    #if targetEnvironment(macCatalyst)
        settings.modalPresentationStyle = .formSheet
    #else
        settings.modalPresentationStyle = .pageSheet
    #endif
        present(settings, animated: true)
    }

    private func handleKeyboardDrive(_ vector: KeyboardDriveVector?) -> Bool {
        guard let vector else { return false }
        if vector.isActive {
            client.sendDrive(x: vector.x, y: vector.y)
        } else {
            client.sendStopDrive()
        }
        return true
    }

    private func stopKeyboardDriveIfNeeded() {
        _ = handleKeyboardDrive(keyboardDriveState.reset())
    }
}

private final class RemoteControlSettingsViewController: UIViewController {
    private let client: RemoteControlClientService
    private var discoveredHosts: [RemoteControlDiscoveredHost]
    private let keyboardDriveState = KeyboardDriveState()
    private let hostField = UITextField()
    private let connectionStatusLabel = UILabel()
    private let discoveredStack = UIStackView()
    private let joystickView = JoystickView()
    private let motorLabel = UILabel()
    private let servoIDField = UITextField()
    private let servoFromField = UITextField()
    private let servoToField = UITextField()
    private let servoPositionField = UITextField()
    private let servoSpeedField = UITextField()

    override var canBecomeFirstResponder: Bool { true }

    override var keyCommands: [UIKeyCommand]? {
        [
            UIKeyCommand(input: UIKeyCommand.inputEscape, modifierFlags: [], action: #selector(doneTapped)),
            UIKeyCommand(input: "w", modifierFlags: .command, action: #selector(doneTapped))
        ]
    }

    init(client: RemoteControlClientService, discoveredHosts: [RemoteControlDiscoveredHost]) {
        self.client = client
        self.discoveredHosts = discoveredHosts
        super.init(nibName: nil, bundle: nil)
    }

    required init?(coder: NSCoder) {
        fatalError("init(coder:) has not been implemented")
    }

    override func viewDidLoad() {
        super.viewDidLoad()
        view.backgroundColor = UIColor(white: 0.11, alpha: 1)
        setupUI()
        rebuildDiscoveredHosts()
    }

    override func viewDidAppear(_ animated: Bool) {
        super.viewDidAppear(animated)
        becomeFirstResponder()
    }

    override func viewWillDisappear(_ animated: Bool) {
        super.viewWillDisappear(animated)
        stopKeyboardDriveIfNeeded()
    }

    override func pressesBegan(_ presses: Set<UIPress>, with event: UIPressesEvent?) {
        if !view.hasFirstResponderTextInput, handleKeyboardDrive(keyboardDriveState.pressesBegan(presses)) {
            return
        }
        super.pressesBegan(presses, with: event)
    }

    override func pressesEnded(_ presses: Set<UIPress>, with event: UIPressesEvent?) {
        if !view.hasFirstResponderTextInput, handleKeyboardDrive(keyboardDriveState.pressesEnded(presses)) {
            return
        }
        super.pressesEnded(presses, with: event)
    }

    override func pressesCancelled(_ presses: Set<UIPress>, with event: UIPressesEvent?) {
        if !view.hasFirstResponderTextInput, handleKeyboardDrive(keyboardDriveState.pressesEnded(presses)) {
            return
        }
        super.pressesCancelled(presses, with: event)
    }

    func updateDiscoveredHosts(_ hosts: [RemoteControlDiscoveredHost]) {
        discoveredHosts = hosts
        if isViewLoaded {
            rebuildDiscoveredHosts()
        }
    }

    private func setupUI() {
        let title = UILabel()
        title.translatesAutoresizingMaskIntoConstraints = false
        title.text = "Remote Controls"
        title.textColor = .white
        title.font = .systemFont(ofSize: 22, weight: .bold)
        view.addSubview(title)

        let doneButton = makeIconButton("xmark.circle.fill", action: #selector(doneTapped))
        view.addSubview(doneButton)

        let switchModeButton = makeButton("Switch to Robot Host", systemImageName: "arrow.triangle.2.circlepath", action: #selector(switchToRobotTapped))
        view.addSubview(switchModeButton)

        let connectionTitle = UILabel()
        connectionTitle.translatesAutoresizingMaskIntoConstraints = false
        connectionTitle.text = "Connection"
        connectionTitle.textColor = UIColor.white.withAlphaComponent(0.75)
        connectionTitle.font = .systemFont(ofSize: 15, weight: .semibold)
        view.addSubview(connectionTitle)

        hostField.translatesAutoresizingMaskIntoConstraints = false
        hostField.text = UserDefaults.standard.string(forKey: "remoteControlHost") ?? ""
        hostField.placeholder = "Phone IP or Tailscale IP"
        hostField.textColor = .white
        hostField.backgroundColor = UIColor(white: 0.2, alpha: 1)
        hostField.layer.cornerRadius = 8
        hostField.autocorrectionType = .no
        hostField.autocapitalizationType = .none
        hostField.keyboardType = .URL
        hostField.returnKeyType = .done
        hostField.leftView = UIView(frame: CGRect(x: 0, y: 0, width: 12, height: 0))
        hostField.leftViewMode = .always
        view.addSubview(hostField)

        let connectButton = makeIconButton("link", action: #selector(connectTapped))
        connectButton.accessibilityLabel = "Connect"
        view.addSubview(connectButton)

        connectionStatusLabel.translatesAutoresizingMaskIntoConstraints = false
        connectionStatusLabel.text = "Bonjour hosts"
        connectionStatusLabel.textColor = UIColor.white.withAlphaComponent(0.6)
        connectionStatusLabel.font = .systemFont(ofSize: 13, weight: .medium)
        view.addSubview(connectionStatusLabel)

        discoveredStack.translatesAutoresizingMaskIntoConstraints = false
        discoveredStack.axis = .horizontal
        discoveredStack.spacing = 8
        discoveredStack.alignment = .fill
        view.addSubview(discoveredStack)

        motorLabel.translatesAutoresizingMaskIntoConstraints = false
        motorLabel.text = "x: 0.00  y: 0.00"
        motorLabel.textColor = .cyan
        motorLabel.font = .monospacedSystemFont(ofSize: 14, weight: .medium)
        view.addSubview(motorLabel)

        joystickView.translatesAutoresizingMaskIntoConstraints = false
        joystickView.onMove = { [weak self] x, y in
            self?.client.sendDrive(x: x, y: y)
            self?.motorLabel.text = String(format: "x: %.2f  y: %.2f", x, y)
        }
        joystickView.onRelease = { [weak self] in
            self?.client.sendStopDrive()
            self?.motorLabel.text = "x: 0.00  y: 0.00"
        }
        view.addSubview(joystickView)

        let servoTitle = UILabel()
        servoTitle.translatesAutoresizingMaskIntoConstraints = false
        servoTitle.text = "ST3215 Servos"
        servoTitle.textColor = UIColor.white.withAlphaComponent(0.75)
        servoTitle.font = .systemFont(ofSize: 15, weight: .semibold)
        view.addSubview(servoTitle)

        [servoIDField, servoFromField, servoToField, servoPositionField, servoSpeedField].forEach(configureField)
        servoIDField.placeholder = "ID"
        servoFromField.placeholder = "From"
        servoToField.placeholder = "To"
        servoPositionField.placeholder = "Position"
        servoSpeedField.placeholder = "Speed"
        servoFromField.text = "1"
        servoToField.text = "20"
        servoPositionField.text = "2048"
        servoSpeedField.text = "1000"

        let scanButton = makeButton("Scan", systemImageName: "dot.radiowaves.left.and.right", action: #selector(scanServosTapped))
        let moveButton = makeButton("Move", systemImageName: "arrow.up.and.down.and.arrow.left.and.right", action: #selector(moveServoTapped))
        let moveAllButton = makeButton("Move All", systemImageName: "arrow.up.and.down.square", action: #selector(moveAllTapped))
        let torqueOnButton = makeButton("Torque On", systemImageName: "bolt.fill", action: #selector(torqueOnTapped))
        let torqueOffButton = makeButton("Torque Off", systemImageName: "bolt.slash.fill", action: #selector(torqueOffTapped))

        let scanRow = UIStackView(arrangedSubviews: [servoFromField, servoToField, scanButton])
        let moveRow = UIStackView(arrangedSubviews: [servoIDField, servoPositionField, servoSpeedField])
        let buttonRow = UIStackView(arrangedSubviews: [moveButton, moveAllButton])
        let torqueRow = UIStackView(arrangedSubviews: [torqueOnButton, torqueOffButton])
        [scanRow, moveRow, buttonRow, torqueRow].forEach { row in
            row.translatesAutoresizingMaskIntoConstraints = false
            row.axis = .horizontal
            row.spacing = 8
            row.distribution = .fillEqually
            view.addSubview(row)
        }

        NSLayoutConstraint.activate([
            title.topAnchor.constraint(equalTo: view.safeAreaLayoutGuide.topAnchor, constant: 24),
            title.leadingAnchor.constraint(equalTo: view.leadingAnchor, constant: 24),

            doneButton.centerYAnchor.constraint(equalTo: title.centerYAnchor),
            doneButton.trailingAnchor.constraint(equalTo: view.trailingAnchor, constant: -24),
            doneButton.widthAnchor.constraint(equalToConstant: 44),
            doneButton.heightAnchor.constraint(equalToConstant: 38),

            switchModeButton.topAnchor.constraint(equalTo: title.bottomAnchor, constant: 20),
            switchModeButton.leadingAnchor.constraint(equalTo: view.leadingAnchor, constant: 24),
            switchModeButton.trailingAnchor.constraint(equalTo: view.trailingAnchor, constant: -24),
            switchModeButton.heightAnchor.constraint(equalToConstant: 40),

            connectionTitle.topAnchor.constraint(equalTo: switchModeButton.bottomAnchor, constant: 18),
            connectionTitle.leadingAnchor.constraint(equalTo: view.leadingAnchor, constant: 24),

            hostField.topAnchor.constraint(equalTo: connectionTitle.bottomAnchor, constant: 10),
            hostField.leadingAnchor.constraint(equalTo: view.leadingAnchor, constant: 24),
            hostField.heightAnchor.constraint(equalToConstant: 40),

            connectButton.leadingAnchor.constraint(equalTo: hostField.trailingAnchor, constant: 8),
            connectButton.trailingAnchor.constraint(equalTo: view.trailingAnchor, constant: -24),
            connectButton.centerYAnchor.constraint(equalTo: hostField.centerYAnchor),
            connectButton.widthAnchor.constraint(equalToConstant: 54),
            connectButton.heightAnchor.constraint(equalTo: hostField.heightAnchor),

            connectionStatusLabel.topAnchor.constraint(equalTo: hostField.bottomAnchor, constant: 10),
            connectionStatusLabel.leadingAnchor.constraint(equalTo: view.leadingAnchor, constant: 24),
            connectionStatusLabel.trailingAnchor.constraint(equalTo: view.trailingAnchor, constant: -24),

            discoveredStack.topAnchor.constraint(equalTo: connectionStatusLabel.bottomAnchor, constant: 8),
            discoveredStack.leadingAnchor.constraint(equalTo: view.leadingAnchor, constant: 24),
            discoveredStack.trailingAnchor.constraint(equalTo: view.trailingAnchor, constant: -24),
            discoveredStack.heightAnchor.constraint(equalToConstant: 38),

            motorLabel.topAnchor.constraint(equalTo: discoveredStack.bottomAnchor, constant: 20),
            motorLabel.centerXAnchor.constraint(equalTo: view.centerXAnchor),

            joystickView.topAnchor.constraint(equalTo: motorLabel.bottomAnchor, constant: 12),
            joystickView.centerXAnchor.constraint(equalTo: view.centerXAnchor),
            joystickView.widthAnchor.constraint(equalToConstant: 190),
            joystickView.heightAnchor.constraint(equalToConstant: 190),

            servoTitle.topAnchor.constraint(equalTo: joystickView.bottomAnchor, constant: 28),
            servoTitle.leadingAnchor.constraint(equalTo: view.leadingAnchor, constant: 24),

            scanRow.topAnchor.constraint(equalTo: servoTitle.bottomAnchor, constant: 12),
            scanRow.leadingAnchor.constraint(equalTo: view.leadingAnchor, constant: 24),
            scanRow.trailingAnchor.constraint(equalTo: view.trailingAnchor, constant: -24),
            scanRow.heightAnchor.constraint(equalToConstant: 40),

            moveRow.topAnchor.constraint(equalTo: scanRow.bottomAnchor, constant: 10),
            moveRow.leadingAnchor.constraint(equalTo: scanRow.leadingAnchor),
            moveRow.trailingAnchor.constraint(equalTo: scanRow.trailingAnchor),
            moveRow.heightAnchor.constraint(equalTo: scanRow.heightAnchor),

            buttonRow.topAnchor.constraint(equalTo: moveRow.bottomAnchor, constant: 10),
            buttonRow.leadingAnchor.constraint(equalTo: scanRow.leadingAnchor),
            buttonRow.trailingAnchor.constraint(equalTo: scanRow.trailingAnchor),
            buttonRow.heightAnchor.constraint(equalTo: scanRow.heightAnchor),

            torqueRow.topAnchor.constraint(equalTo: buttonRow.bottomAnchor, constant: 10),
            torqueRow.leadingAnchor.constraint(equalTo: scanRow.leadingAnchor),
            torqueRow.trailingAnchor.constraint(equalTo: scanRow.trailingAnchor),
            torqueRow.heightAnchor.constraint(equalTo: scanRow.heightAnchor)
        ])
    }

    private func configureField(_ field: UITextField) {
        field.translatesAutoresizingMaskIntoConstraints = false
        field.textColor = .white
        field.backgroundColor = UIColor(white: 0.2, alpha: 1)
        field.layer.cornerRadius = 8
        field.keyboardType = .numberPad
        field.textAlignment = .center
        field.font = .systemFont(ofSize: 14, weight: .medium)
    }

    private func makeButton(_ title: String, systemImageName: String, action: Selector) -> UIButton {
        let button = UIButton(type: .system)
        configureGlassButton(button, title: title, systemImageName: systemImageName)
        button.addTarget(self, action: action, for: .touchUpInside)
        return button
    }

    private func makeIconButton(_ systemImageName: String, action: Selector) -> UIButton {
        let button = UIButton(type: .system)
        configureIconGlassButton(button, systemImageName: systemImageName)
        button.addTarget(self, action: action, for: .touchUpInside)
        return button
    }

    private func rebuildDiscoveredHosts() {
        discoveredStack.arrangedSubviews.forEach { view in
            discoveredStack.removeArrangedSubview(view)
            view.removeFromSuperview()
        }
        guard !discoveredHosts.isEmpty else {
            connectionStatusLabel.text = "No Bonjour hosts found"
            return
        }
        connectionStatusLabel.text = "Bonjour hosts"
        for (index, host) in discoveredHosts.prefix(3).enumerated() {
            let button = makeButton(host.name, systemImageName: "network", action: #selector(discoveredHostTapped(_:)))
            button.tag = index
            discoveredStack.addArrangedSubview(button)
        }
    }

    @objc private func doneTapped() {
        view.endEditing(true)
        dismiss(animated: true)
    }

    @objc private func switchToRobotTapped() {
        requestAppRoleSwitch(.robot)
    }

    @objc private func connectTapped() {
        let host = hostField.text?.trimmingCharacters(in: .whitespacesAndNewlines) ?? ""
        guard !host.isEmpty else { return }
        UserDefaults.standard.set(host, forKey: "remoteControlHost")
        client.connect(host: host)
        hostField.resignFirstResponder()
    }

    @objc private func discoveredHostTapped(_ sender: UIButton) {
        guard discoveredHosts.indices.contains(sender.tag) else { return }
        client.connect(to: discoveredHosts[sender.tag].endpoint)
    }

    @objc private func scanServosTapped() {
        var message = RemoteMessage(type: "scanServos")
        message.from = UInt8(servoFromField.text ?? "") ?? 1
        message.to = UInt8(servoToField.text ?? "") ?? 20
        client.send(message)
    }

    @objc private func moveServoTapped() {
        guard let id = UInt8(servoIDField.text ?? "") else { return }
        var message = RemoteMessage(type: "moveServo")
        message.id = id
        message.position = UInt16(servoPositionField.text ?? "") ?? 2048
        message.speed = UInt16(servoSpeedField.text ?? "") ?? 1000
        client.send(message)
    }

    @objc private func moveAllTapped() {
        var message = RemoteMessage(type: "moveAllServos")
        message.position = UInt16(servoPositionField.text ?? "") ?? 2048
        message.speed = UInt16(servoSpeedField.text ?? "") ?? 1000
        client.send(message)
    }

    @objc private func torqueOnTapped() {
        sendTorque(enabled: true)
    }

    @objc private func torqueOffTapped() {
        sendTorque(enabled: false)
    }

    private func sendTorque(enabled: Bool) {
        guard let id = UInt8(servoIDField.text ?? "") else { return }
        var message = RemoteMessage(type: "setServoTorque")
        message.id = id
        message.enabled = enabled
        client.send(message)
    }

    private func handleKeyboardDrive(_ vector: KeyboardDriveVector?) -> Bool {
        guard let vector else { return false }
        if vector.isActive {
            client.sendDrive(x: vector.x, y: vector.y)
            motorLabel.text = String(format: "x: %.2f  y: %.2f", vector.x, vector.y)
        } else {
            client.sendStopDrive()
            motorLabel.text = "x: 0.00  y: 0.00"
        }
        return true
    }

    private func stopKeyboardDriveIfNeeded() {
        _ = handleKeyboardDrive(keyboardDriveState.reset())
    }
}

fileprivate func configureGlassButton(_ button: UIButton, title: String, systemImageName: String) {
    button.translatesAutoresizingMaskIntoConstraints = false

    var configuration = UIButton.Configuration.plain()
    configuration.title = title
    configuration.image = UIImage(systemName: systemImageName)
    configuration.imagePadding = 6
    configuration.baseForegroundColor = .white
    configuration.contentInsets = NSDirectionalEdgeInsets(top: 8, leading: 12, bottom: 8, trailing: 12)
    button.configuration = configuration
    button.titleLabel?.numberOfLines = 1
    button.titleLabel?.lineBreakMode = .byTruncatingTail
    button.titleLabel?.adjustsFontSizeToFitWidth = true
    button.titleLabel?.minimumScaleFactor = 0.78

    button.tintColor = .white
    button.backgroundColor = UIColor.white.withAlphaComponent(0.16)
    button.layer.cornerRadius = 12
    button.layer.borderWidth = 1
    button.layer.borderColor = UIColor.white.withAlphaComponent(0.28).cgColor
    button.layer.shadowColor = UIColor.black.cgColor
    button.layer.shadowOpacity = 0.25
    button.layer.shadowRadius = 12
    button.layer.shadowOffset = CGSize(width: 0, height: 4)
}

fileprivate func configureIconGlassButton(_ button: UIButton, systemImageName: String) {
    button.translatesAutoresizingMaskIntoConstraints = false

    var configuration = UIButton.Configuration.plain()
    configuration.image = UIImage(systemName: systemImageName)
    configuration.baseForegroundColor = .white
    configuration.contentInsets = NSDirectionalEdgeInsets(top: 8, leading: 10, bottom: 8, trailing: 10)
    button.configuration = configuration

    button.tintColor = .white
    button.backgroundColor = UIColor.white.withAlphaComponent(0.16)
    button.layer.cornerRadius = 12
    button.layer.borderWidth = 1
    button.layer.borderColor = UIColor.white.withAlphaComponent(0.28).cgColor
    button.layer.shadowColor = UIColor.black.cgColor
    button.layer.shadowOpacity = 0.25
    button.layer.shadowRadius = 12
    button.layer.shadowOffset = CGSize(width: 0, height: 4)
}

