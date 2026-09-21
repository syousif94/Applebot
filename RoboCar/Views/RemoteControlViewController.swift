//
//  RemoteControlViewController.swift
//  RoboCar
//

import UIKit
import Speech
import AVFoundation

final class RemoteControlViewController: PanelViewController {
    private let client = RemoteControlClientService()
    private let remoteGrid = OccupancyGrid(cellSize: 0.05, gridRadius: 500)
    private let keyboardDriveState = KeyboardDriveState()

    private let cameraVideoView = RemoteVideoView()
    private let videoFallbackImageView = UIImageView()
    private let personBoxOverlay = RemotePersonBoxOverlay()
    private let mapView: GridMapView
    private let meshVoxelView = MeshVoxelView()
    private let mapToggleButton = UIButton(type: .system)
    private var showingMesh = false
    private let statusLabel = UILabel()
    private let runButton = UIButton(type: .system)
    private let pauseButton = UIButton(type: .system)
    private let clearButton = UIButton(type: .system)
    private let settingsButton = UIButton(type: .system)

    // NL command bar
    private let nlCommandBar = UIView()
    private let nlCommandField = UITextField()
    private let nlCommandMicButton = UIButton(type: .system)
    private let nlCommandSendButton = UIButton(type: .system)
    private let nlCommandStopButton = UIButton(type: .system)

    // Speech recognition
    private var sfRecognizer: SFSpeechRecognizer?
    private var sfRequest: SFSpeechAudioBufferRecognitionRequest?
    private var sfTask: SFSpeechRecognitionTask?
    private var audioEngine: AVAudioEngine?
    private var isRecording = false

    private var compactContentConstraints: [NSLayoutConstraint] = []
    private var wideContentConstraints: [NSLayoutConstraint] = []
    private var usesWideContentLayout = false

    private var discoveredHosts: [RemotePeer] = []
    private weak var settingsViewController: RemoteControlSettingsViewController?
    private weak var servoViewController: RemoteControlSettingsViewController?
    private var servoDataControllers: [RemoteControlSettingsViewController] {
        #if targetEnvironment(macCatalyst)
        return [settingsViewController, servoViewController].compactMap { $0 }
        #else
        return [settingsViewController].compactMap { $0 }
        #endif
    }
    private var remoteServoIDs: [UInt8] = []
    private var remoteServoPositions: [UInt8: UInt16] = [:]
    private var remoteServoStates: [UInt8: ServoState] = [:]
    private var remoteServoAxisStatuses: [UInt8: ServoAxisStatus] = [:]


    private var lastVideoFrameAt = Date()
    private var lastVideoRecoveryAt = Date.distantPast
    private var videoWatchdog: Timer?
    private let videoStallThreshold: TimeInterval = 3.0
    private let videoRecoveryCooldown: TimeInterval = 5.0

    private let connectionBadgeLabel = UILabel()
    private var connectionBadgeHiddenConstraint: NSLayoutConstraint!

    override var canBecomeFirstResponder: Bool { true }

    override var keyCommands: [UIKeyCommand]? { nil }

    override func panelResignedKey() {
        stopKeyboardDriveIfNeeded()
    }

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
        client.start(videoView: cameraVideoView)
        startAutoConnect()
    }

    override func viewWillDisappear(_ animated: Bool) {
        super.viewWillDisappear(animated)
        if presentedViewController != nil { stopKeyboardDriveIfNeeded(); return }
        stopVideoWatchdog()
        stopKeyboardDriveIfNeeded()
        stopSpeechRecognition()
        client.sendStopNLCommand()
        client.disconnect()
    }

    override func viewDidAppear(_ animated: Bool) {
        super.viewDidAppear(animated)
        becomeFirstResponder()
        startVideoWatchdog()
    }

    func resumeRemoteConnection() {
        loadViewIfNeeded()
        client.start(videoView: cameraVideoView)
        startAutoConnect()
        startVideoWatchdog()
        recoverVideoIfStalled(force: false)
    }

    private func startVideoWatchdog() {
        stopVideoWatchdog()
        lastVideoFrameAt = Date()
        let timer = Timer(timeInterval: 1.0, repeats: true) { [weak self] _ in
            self?.recoverVideoIfStalled(force: false)
        }
        RunLoop.main.add(timer, forMode: .common)
        videoWatchdog = timer
    }

    private func stopVideoWatchdog() {
        videoWatchdog?.invalidate()
        videoWatchdog = nil
    }

    private func recoverVideoIfStalled(force: Bool) {
        guard client.isConnected else { return }
        let now = Date()
        let stalled = now.timeIntervalSince(lastVideoFrameAt) > videoStallThreshold
        guard force || stalled else { return }
        guard now.timeIntervalSince(lastVideoRecoveryAt) > videoRecoveryCooldown else { return }
        lastVideoRecoveryAt = now
        lastVideoFrameAt = now
        client.restartVideo()
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
        cameraVideoView.displayLayer.videoGravity = .resizeAspectFill
        cameraVideoView.clipsToBounds = true
        view.addSubview(cameraVideoView)

        videoFallbackImageView.translatesAutoresizingMaskIntoConstraints = false
        videoFallbackImageView.backgroundColor = .clear
        videoFallbackImageView.contentMode = .scaleAspectFill
        videoFallbackImageView.clipsToBounds = true
        videoFallbackImageView.isHidden = true
        view.addSubview(videoFallbackImageView)

        personBoxOverlay.translatesAutoresizingMaskIntoConstraints = false
        personBoxOverlay.onTapBody = { [weak self] id in
            self?.sendFollowPerson(id: id)
        }
        personBoxOverlay.onTapName = { [weak self] id in
            self?.promptNamePerson(id: id)
        }
        personBoxOverlay.onTapDelete = { [weak self] id in
            self?.sendDeleteNamedPerson(id: id)
        }
        view.addSubview(personBoxOverlay)
        NSLayoutConstraint.activate([
            personBoxOverlay.topAnchor.constraint(equalTo: cameraVideoView.topAnchor),
            personBoxOverlay.leadingAnchor.constraint(equalTo: cameraVideoView.leadingAnchor),
            personBoxOverlay.trailingAnchor.constraint(equalTo: cameraVideoView.trailingAnchor),
            personBoxOverlay.bottomAnchor.constraint(equalTo: cameraVideoView.bottomAnchor),
        ])

        mapView.translatesAutoresizingMaskIntoConstraints = false
        mapView.backgroundColor = UIColor(white: 0.1, alpha: 1)
        mapView.onTapWorldPosition = { [weak self] x, y in
            var message = RemoteMessage(type: "addRoutePoint")
            message.x = x
            message.y = y
            self?.client.send(message)
        }
        view.addSubview(mapView)

        meshVoxelView.translatesAutoresizingMaskIntoConstraints = false
        meshVoxelView.occupancyGrid = remoteGrid
        meshVoxelView.isHidden = true
        view.addSubview(meshVoxelView)

        configureIconGlassButton(mapToggleButton, systemImageName: "square.3.layers.3d")
        mapToggleButton.addTarget(self, action: #selector(mapViewToggleTapped), for: .touchUpInside)
        view.addSubview(mapToggleButton)

        statusLabel.translatesAutoresizingMaskIntoConstraints = false
        statusLabel.textColor = .white
        statusLabel.font = .monospacedSystemFont(ofSize: 13, weight: .medium)
        statusLabel.numberOfLines = 2
        statusLabel.text = "Discovering RoboCar..."
        view.addSubview(statusLabel)

        connectionBadgeLabel.translatesAutoresizingMaskIntoConstraints = false
        connectionBadgeLabel.font = .systemFont(ofSize: 11, weight: .bold)
        connectionBadgeLabel.textColor = .white
        connectionBadgeLabel.textAlignment = .center
        connectionBadgeLabel.layer.cornerRadius = 6
        connectionBadgeLabel.clipsToBounds = true
        connectionBadgeLabel.setContentHuggingPriority(.required, for: .horizontal)
        connectionBadgeLabel.setContentCompressionResistancePriority(.required, for: .horizontal)
        view.addSubview(connectionBadgeLabel)
        connectionBadgeHiddenConstraint = connectionBadgeLabel.widthAnchor.constraint(equalToConstant: 0)
        connectionBadgeHiddenConstraint.isActive = true

        configureGlassButton(runButton, title: "Run", systemImageName: "play.fill")
        configureGlassButton(pauseButton, title: "Pause", systemImageName: "pause.fill")
        configureGlassButton(clearButton, title: "Clear", systemImageName: "trash")
        configureIconGlassButton(settingsButton, systemImageName: "gearshape.fill")
        runButton.addTarget(self, action: #selector(runRouteTapped), for: .touchUpInside)
        pauseButton.addTarget(self, action: #selector(pauseRouteTapped), for: .touchUpInside)
        clearButton.addTarget(self, action: #selector(clearRouteTapped), for: .touchUpInside)
        settingsButton.addTarget(self, action: #selector(settingsTapped), for: .touchUpInside)
        [runButton, pauseButton, clearButton, settingsButton].forEach(view.addSubview)

        setupNLCommandBar()

        NSLayoutConstraint.activate([
            statusLabel.topAnchor.constraint(equalTo: view.safeAreaLayoutGuide.topAnchor, constant: 12),
            statusLabel.leadingAnchor.constraint(equalTo: view.leadingAnchor, constant: 16),
            statusLabel.trailingAnchor.constraint(equalTo: connectionBadgeLabel.leadingAnchor, constant: -4),

            connectionBadgeLabel.trailingAnchor.constraint(equalTo: settingsButton.leadingAnchor, constant: -8),
            connectionBadgeLabel.centerYAnchor.constraint(equalTo: settingsButton.centerYAnchor),
            connectionBadgeLabel.heightAnchor.constraint(equalToConstant: 22),

            settingsButton.topAnchor.constraint(equalTo: view.safeAreaLayoutGuide.topAnchor, constant: 10),
            settingsButton.trailingAnchor.constraint(equalTo: view.trailingAnchor, constant: -16),
            settingsButton.widthAnchor.constraint(equalToConstant: 44),
            settingsButton.heightAnchor.constraint(equalToConstant: 36),

            runButton.leadingAnchor.constraint(equalTo: view.leadingAnchor, constant: 16),
            runButton.bottomAnchor.constraint(equalTo: nlCommandBar.topAnchor, constant: -8),
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
        NSLayoutConstraint.activate([
            meshVoxelView.topAnchor.constraint(equalTo: mapView.topAnchor),
            meshVoxelView.leadingAnchor.constraint(equalTo: mapView.leadingAnchor),
            meshVoxelView.trailingAnchor.constraint(equalTo: mapView.trailingAnchor),
            meshVoxelView.bottomAnchor.constraint(equalTo: mapView.bottomAnchor),

            mapToggleButton.topAnchor.constraint(equalTo: mapView.topAnchor, constant: 8),
            mapToggleButton.leadingAnchor.constraint(equalTo: mapView.leadingAnchor, constant: 8),
            mapToggleButton.widthAnchor.constraint(equalToConstant: 44),
            mapToggleButton.heightAnchor.constraint(equalToConstant: 36),
        ])
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
            self?.discoveredHosts = RemoteControlIrohSession.shared.store?.peers.filter { $0.role == .robot } ?? []
            self?.settingsViewController?.updateDiscoveredHosts(self?.discoveredHosts ?? [])
            self?.updateConnectionBadge()
        }
        client.onMessage = { [weak self] message in
            self?.handleRemoteMessage(message)
        }
        client.onVideoFrameImage = { [weak self] image in
            guard let self else { return }
            self.lastVideoFrameAt = Date()
            self.videoFallbackImageView.image = image
            self.videoFallbackImageView.isHidden = false
        }
        client.onVideoFrameSize = { [weak self] size in
            self?.personBoxOverlay.videoSize = size
            self?.lastVideoFrameAt = Date()
            self?.videoFallbackImageView.isHidden = true
        }
        client.onConnected = { [weak self] in
            guard let self else { return }
            self.resetRemoteMap()
            self.lastVideoFrameAt = Date()
            self.lastVideoRecoveryAt = Date()
            self.updateConnectionBadge()
            self.servoDataControllers.forEach { $0.setConnected(true) }
        }
        client.onDisconnected = { [weak self] in
            self?.resetRemoteMap()
            self?.remoteServoIDs.removeAll()
            self?.remoteServoPositions.removeAll()
            self?.remoteServoStates.removeAll()
            self?.remoteServoAxisStatuses.removeAll()
            self?.servoDataControllers.forEach { $0.setConnected(false) }
            self?.updateConnectionBadge()
        }
    }

    private func startAutoConnect() {
        client.connectSelectedHost()
    }

    private func updateConnectionBadge() {
        if client.isConnected {
            connectionBadgeHiddenConstraint.isActive = false
            connectionBadgeLabel.isHidden = false
            if RemoteControlIrohSession.shared.isRelay {
                connectionBadgeLabel.text = "  RELAY  "
                connectionBadgeLabel.backgroundColor = UIColor(red: 0.35, green: 0.45, blue: 0.95, alpha: 1)
            } else {
                connectionBadgeLabel.text = "  DIRECT  "
                connectionBadgeLabel.backgroundColor = UIColor(red: 0.2, green: 0.72, blue: 0.4, alpha: 1)
            }
        } else {
            connectionBadgeLabel.isHidden = true
            connectionBadgeHiddenConstraint.isActive = true
        }
    }

    private func resetRemoteMap() {
        remoteGrid.clear()
        mapView.resetInitialHeading()
        mapView.setNeedsDisplay()
        meshVoxelView.refresh()
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
            if !meshVoxelView.isHidden { meshVoxelView.refresh() }
            let ble = message.bleConnected == true ? "BLE connected" : "BLE disconnected"
            statusLabel.text = "\(message.navState ?? "remote") - \(ble)"
        case "gridUpdate":
            applyGridUpdate(message.grid)
        case "gridReset":
            resetRemoteMap()
        case "meshAnchors":
            if let anchors = message.meshAnchors, !anchors.isEmpty {
                meshVoxelView.updateMeshAnchors(anchors)
            }
        case "personBoxes":
            personBoxOverlay.people = message.personBoxes ?? []
        case "servoList":
            remoteServoIDs = message.servoIDs ?? []
            servoDataControllers.forEach { $0.updateServoIDs(remoteServoIDs) }
        case "servoState":
            if let remoteState = message.servoState {
                let state = remoteState.asServoState
                remoteServoStates[state.id] = state
                servoDataControllers.forEach { $0.applyServoState(state) }
            }
        case "servoPositions":
            if let samples = message.servoPositions {
                var positions: [UInt8: UInt16] = [:]
                samples.forEach { positions[$0.id] = $0.position }
                remoteServoPositions.merge(positions) { _, new in new }
                servoDataControllers.forEach { $0.applyServoPositions(positions) }
            }
        case "servoAxisStatus":
            if let remoteStatus = message.servoAxisStatus {
                let status = remoteStatus.asServoAxisStatus
                remoteServoAxisStatuses[status.id] = status
                servoDataControllers.forEach { $0.applyServoAxisStatus(status) }
            }
        case "status":
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
        if !meshVoxelView.isHidden { meshVoxelView.refresh() }
    }

    // MARK: - Person follow / naming (controller → host)

    /// Tapping a person follows them; tapping the active person again cancels.
    private func sendFollowPerson(id: String) {
        if let box = personBoxOverlay.people.first(where: { $0.id == id }), box.isActive {
            client.send(RemoteMessage(type: "stopFollowing"))
            return
        }
        var message = RemoteMessage(type: "followPerson")
        message.personID = id
        client.send(message)
    }

    /// Prompt for a name and ask the host to save this person's embedding.
    private func promptNamePerson(id: String) {
        let existingName = personBoxOverlay.people.first(where: { $0.id == id })?.name
        let alert = UIAlertController(
            title: existingName == nil ? "Name this person" : "Rename person",
            message: "The robot will remember them and can follow them by name.",
            preferredStyle: .alert
        )
        alert.addTextField { tf in
            tf.placeholder = "Name"
            tf.text = existingName
            tf.autocapitalizationType = .words
            tf.clearButtonMode = .whileEditing
        }
        alert.addAction(UIAlertAction(title: "Cancel", style: .cancel))
        alert.addAction(UIAlertAction(title: "Save", style: .default) { [weak self] _ in
            guard let self = self else { return }
            let name = alert.textFields?.first?.text?.trimmingCharacters(in: .whitespacesAndNewlines) ?? ""
            guard !name.isEmpty else { return }
            var message = RemoteMessage(type: "namePerson")
            message.personID = id
            message.personName = name
            self.client.send(message)
        })
        present(alert, animated: true)
    }

    /// Ask the host to delete the saved person matching this box's name.
    private func sendDeleteNamedPerson(id: String) {
        guard let name = personBoxOverlay.people.first(where: { $0.id == id })?.name else { return }
        var message = RemoteMessage(type: "deleteNamedPerson")
        message.personName = name
        client.send(message)
    }

    @objc private func mapViewToggleTapped() {
        showingMesh.toggle()
        mapView.isHidden = showingMesh
        meshVoxelView.isHidden = !showingMesh
        var cfg = mapToggleButton.configuration ?? .plain()
        cfg.image = UIImage(systemName: showingMesh ? "map.fill" : "square.3.layers.3d")
        mapToggleButton.configuration = cfg
        if showingMesh { meshVoxelView.refresh() }
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
        #if targetEnvironment(macCatalyst)
        guard let scene = view.window?.windowScene else { return }
        PanelWindows.shared.open(.settings, from: scene) {
            let settings = RemoteControlSettingsViewController(client: client, discoveredHosts: discoveredHosts, mode: .settings)
            settings.onOpenServos = { [weak self] in self?.openServoWindow() }
            settingsViewController = settings
            settings.seedServoData(ids: remoteServoIDs, positions: remoteServoPositions, states: remoteServoStates, axisStatuses: remoteServoAxisStatuses)
            return settings
        }
        #else
        let settings = RemoteControlSettingsViewController(client: client, discoveredHosts: discoveredHosts)
        settingsViewController = settings
        settings.preferredContentSize = CGSize(width: 620, height: 760)
        settings.modalPresentationStyle = .pageSheet
        PanelPresentation.prepare(settings)
        present(settings, animated: true)
        settings.seedServoData(
            ids: remoteServoIDs,
            positions: remoteServoPositions,
            states: remoteServoStates,
            axisStatuses: remoteServoAxisStatuses
        )
        settings.setConnected(client.isConnected)
        #endif
    }

    #if targetEnvironment(macCatalyst)
    private func openServoWindow() {
        guard let scene = view.window?.windowScene else { return }
        PanelWindows.shared.open(.servos, from: scene) {
            let servos = RemoteControlSettingsViewController(client: client, discoveredHosts: [], mode: .servos)
            servoViewController = servos
            servos.seedServoData(ids: remoteServoIDs, positions: remoteServoPositions, states: remoteServoStates, axisStatuses: remoteServoAxisStatuses)
            servos.setConnected(client.isConnected)
            return servos
        }
    }
    #endif

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

    // MARK: - NL Command Bar

    private func setupNLCommandBar() {
        nlCommandBar.translatesAutoresizingMaskIntoConstraints = false
        nlCommandBar.backgroundColor = UIColor(white: 0.08, alpha: 1)
        view.addSubview(nlCommandBar)

        nlCommandField.translatesAutoresizingMaskIntoConstraints = false
        nlCommandField.backgroundColor = UIColor(white: 0.18, alpha: 1)
        nlCommandField.textColor = .white
        nlCommandField.font = .systemFont(ofSize: 15)
        nlCommandField.layer.cornerRadius = 10
        nlCommandField.leftView = UIView(frame: CGRect(x: 0, y: 0, width: 10, height: 1))
        nlCommandField.leftViewMode = .always
        nlCommandField.returnKeyType = .send
        nlCommandField.delegate = self
        nlCommandField.attributedPlaceholder = NSAttributedString(
            string: "Send a command to the robot…",
            attributes: [.foregroundColor: UIColor(white: 0.45, alpha: 1)]
        )
        nlCommandBar.addSubview(nlCommandField)

        var micCfg = UIButton.Configuration.plain()
        micCfg.image = UIImage(systemName: "mic.fill")
        micCfg.baseForegroundColor = .white
        nlCommandMicButton.translatesAutoresizingMaskIntoConstraints = false
        nlCommandMicButton.configuration = micCfg
        nlCommandMicButton.addTarget(self, action: #selector(nlMicTapped), for: .touchUpInside)
        nlCommandBar.addSubview(nlCommandMicButton)

        var sendCfg = UIButton.Configuration.plain()
        sendCfg.image = UIImage(systemName: "arrow.up.circle.fill")
        sendCfg.baseForegroundColor = UIColor(red: 0.3, green: 0.7, blue: 1.0, alpha: 1)
        nlCommandSendButton.translatesAutoresizingMaskIntoConstraints = false
        nlCommandSendButton.configuration = sendCfg
        nlCommandSendButton.addTarget(self, action: #selector(nlSendTapped), for: .touchUpInside)
        nlCommandBar.addSubview(nlCommandSendButton)

        var stopCfg = UIButton.Configuration.plain()
        stopCfg.image = UIImage(systemName: "stop.circle.fill")
        stopCfg.baseForegroundColor = UIColor(red: 1.0, green: 0.4, blue: 0.4, alpha: 1)
        nlCommandStopButton.translatesAutoresizingMaskIntoConstraints = false
        nlCommandStopButton.configuration = stopCfg
        nlCommandStopButton.isHidden = true
        nlCommandStopButton.addTarget(self, action: #selector(nlStopTapped), for: .touchUpInside)
        nlCommandBar.addSubview(nlCommandStopButton)

        NSLayoutConstraint.activate([
            nlCommandBar.leadingAnchor.constraint(equalTo: view.leadingAnchor),
            nlCommandBar.trailingAnchor.constraint(equalTo: view.trailingAnchor),
            nlCommandBar.bottomAnchor.constraint(equalTo: view.safeAreaLayoutGuide.bottomAnchor),
            nlCommandBar.heightAnchor.constraint(equalToConstant: 52),

            nlCommandStopButton.trailingAnchor.constraint(equalTo: nlCommandBar.trailingAnchor, constant: -10),
            nlCommandStopButton.centerYAnchor.constraint(equalTo: nlCommandBar.centerYAnchor),
            nlCommandStopButton.widthAnchor.constraint(equalToConstant: 40),
            nlCommandStopButton.heightAnchor.constraint(equalToConstant: 40),

            nlCommandSendButton.trailingAnchor.constraint(equalTo: nlCommandStopButton.leadingAnchor, constant: -2),
            nlCommandSendButton.centerYAnchor.constraint(equalTo: nlCommandBar.centerYAnchor),
            nlCommandSendButton.widthAnchor.constraint(equalToConstant: 40),
            nlCommandSendButton.heightAnchor.constraint(equalToConstant: 40),

            nlCommandMicButton.trailingAnchor.constraint(equalTo: nlCommandSendButton.leadingAnchor, constant: -2),
            nlCommandMicButton.centerYAnchor.constraint(equalTo: nlCommandBar.centerYAnchor),
            nlCommandMicButton.widthAnchor.constraint(equalToConstant: 40),
            nlCommandMicButton.heightAnchor.constraint(equalToConstant: 40),

            nlCommandField.leadingAnchor.constraint(equalTo: nlCommandBar.leadingAnchor, constant: 10),
            nlCommandField.trailingAnchor.constraint(equalTo: nlCommandMicButton.leadingAnchor, constant: -6),
            nlCommandField.centerYAnchor.constraint(equalTo: nlCommandBar.centerYAnchor),
            nlCommandField.heightAnchor.constraint(equalToConstant: 36),
        ])
    }

    @objc private func nlMicTapped() {
        if isRecording {
            stopSpeechRecognition()
        } else {
            startSpeechRecognition()
        }
    }

    @objc private func nlSendTapped() {
        sendNLCommand()
    }

    @objc private func nlStopTapped() {
        client.sendStopNLCommand()
        nlCommandStopButton.isHidden = true
        nlCommandField.text = nil
    }

    private func sendNLCommand() {
        stopSpeechRecognition()
        guard let text = nlCommandField.text?.trimmingCharacters(in: .whitespacesAndNewlines), !text.isEmpty else { return }
        client.sendNLCommand(text)
        nlCommandField.resignFirstResponder()
        nlCommandField.text = nil
        nlCommandStopButton.isHidden = false
    }

    // MARK: - Speech Recognition

    private func startSpeechRecognition() {
        SFSpeechRecognizer.requestAuthorization { [weak self] status in
            guard status == .authorized else { return }
            DispatchQueue.main.async { self?.beginSpeechCapture() }
        }
    }

    private func beginSpeechCapture() {
        guard audioEngine == nil, !isRecording else { return }
        sfRecognizer = SFSpeechRecognizer()
        guard let recognizer = sfRecognizer, recognizer.isAvailable else { return }

        do {
            try AVAudioSession.sharedInstance().setCategory(.record, mode: .measurement, options: .duckOthers)
            try AVAudioSession.sharedInstance().setActive(true, options: .notifyOthersOnDeactivation)

            let engine = AVAudioEngine()
            audioEngine = engine
            let inputNode = engine.inputNode
            let format = inputNode.inputFormat(forBus: 0)
            guard format.sampleRate > 0, format.channelCount > 0 else {
                throw NSError(domain: "SpeechRecognition", code: -2, userInfo: [NSLocalizedDescriptionKey: "Microphone input is unavailable"])
            }

            let request = SFSpeechAudioBufferRecognitionRequest()
            sfRequest = request
            request.shouldReportPartialResults = true

            sfTask = recognizer.recognitionTask(with: request) { [weak self] result, error in
                guard let self else { return }
                if let result {
                    self.nlCommandField.text = result.bestTranscription.formattedString
                }
                if result?.isFinal == true || error != nil {
                    self.stopSpeechRecognition()
                }
            }

            inputNode.installTap(onBus: 0, bufferSize: 1024, format: format) { [weak self] buffer, _ in
                self?.sfRequest?.append(buffer)
            }

            engine.prepare()
            try engine.start()
            isRecording = true
            updateMicButtonAppearance()
        } catch {
            print("[NLCmd] Audio error: \(error)")
            stopSpeechRecognition()
        }
    }

    private func stopSpeechRecognition() {
        audioEngine?.stop()
        audioEngine?.inputNode.removeTap(onBus: 0)
        sfRequest?.endAudio()
        sfTask?.cancel()
        audioEngine = nil
        sfRequest = nil
        sfTask = nil
        isRecording = false
        try? AVAudioSession.sharedInstance().setActive(false, options: .notifyOthersOnDeactivation)
        updateMicButtonAppearance()
    }

    private func updateMicButtonAppearance() {
        var cfg = UIButton.Configuration.plain()
        cfg.image = UIImage(systemName: isRecording ? "mic.slash.fill" : "mic.fill")
        cfg.baseForegroundColor = isRecording ? UIColor(red: 1.0, green: 0.4, blue: 0.4, alpha: 1) : .white
        nlCommandMicButton.configuration = cfg
    }
}

extension RemoteControlViewController: UITextFieldDelegate {
    func textFieldShouldReturn(_ textField: UITextField) -> Bool {
        if textField === nlCommandField {
            sendNLCommand()
        }
        return true
    }
}

private final class RemoteControlSettingsViewController: PanelViewController {
    enum Mode { case combined, settings, servos }
    private let mode: Mode
    var onOpenServos: (() -> Void)?

    private let client: RemoteControlClientService
    private var discoveredHosts: [RemotePeer]
    private let keyboardDriveState = KeyboardDriveState()
    private let hostField = UITextField()
    private let connectionStatusLabel = UILabel()
    private let discoveredStack = UIStackView()
    private let joystickView = JoystickView()
    private let motorLabel = UILabel()
    private let servoFromField = UITextField()
    private let servoToField = UITextField()
    private let servoPositionField = UITextField()
    private let servoSpeedField = UITextField()
    private let servoStatusLabel = UILabel()
    private let servoListView = ServoControlListView()
    private var servoCommander: RemoteServoCommander?
    private weak var robotModelEditor: RobotModelViewController?
    private var modelServoIDs: [UInt8] = []

    override var canBecomeFirstResponder: Bool { true }

    override var keyCommands: [UIKeyCommand]? {
        [
            UIKeyCommand(input: UIKeyCommand.inputEscape, modifierFlags: [], action: #selector(doneTapped)),
            UIKeyCommand(input: "w", modifierFlags: .command, action: #selector(doneTapped))
        ]
    }

    init(client: RemoteControlClientService, discoveredHosts: [RemotePeer], mode: Mode = .combined) {
        self.client = client
        self.discoveredHosts = discoveredHosts
        self.mode = mode
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
        setConnected(client.isConnected)
    }

    override func viewDidAppear(_ animated: Bool) {
        super.viewDidAppear(animated)
        becomeFirstResponder()
        servoListView.setScreenActive(mode != .settings)
    }

    override func viewWillDisappear(_ animated: Bool) {
        super.viewWillDisappear(animated)
        stopKeyboardDriveIfNeeded()
        servoListView.setScreenActive(false)
    }

    override func panelVisibilityChanged(_ visible: Bool) {
        servoListView.setScreenActive(visible && mode != .settings)
    }

    override func panelResignedKey() {
        stopKeyboardDriveIfNeeded()
        if mode != .servos { client.sendStopDrive() }
        servoListView.stopAllJogs()
    }

    func setConnected(_ connected: Bool) {
        guard isViewLoaded else { return }
        robotModelEditor?.motors.connectionChanged()
        servoListView.setControlsEnabled(connected)
        joystickView.isUserInteractionEnabled = connected
        if !connected {
            servoListView.setServoIDs([])
            servoStatusLabel.text = "Disconnected"
        }
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

    func updateDiscoveredHosts(_ hosts: [RemotePeer]) {
        discoveredHosts = hosts
        if isViewLoaded {
            rebuildDiscoveredHosts()
        }
    }

    // MARK: - Servo data feed (from parent controller)

    func seedServoData(ids: [UInt8], positions: [UInt8: UInt16], states: [UInt8: ServoState], axisStatuses: [UInt8: ServoAxisStatus]) {
        loadViewIfNeeded()
        modelServoIDs = ids
        servoListView.setServoIDs(ids)
        if !positions.isEmpty {
            servoListView.apply(positions: positions)
        }
        states.values.forEach { servoListView.apply(state: $0) }
        axisStatuses.values.forEach { servoListView.apply(axisStatus: $0) }
    }

    func updateServoIDs(_ ids: [UInt8]) {
        guard isViewLoaded else { return }
        modelServoIDs = ids
        robotModelEditor?.motors.updateIDs(ids)
        servoListView.setServoIDs(ids)
        servoStatusLabel.text = ids.isEmpty ? "No servos found" : "\(ids.count) servo\(ids.count == 1 ? "" : "s") scanned"
    }

    func applyServoPositions(_ positions: [UInt8: UInt16]) {
        guard isViewLoaded else { return }
        servoListView.apply(positions: positions)
    }

    func applyServoState(_ state: ServoState) {
        guard isViewLoaded else { return }
        robotModelEditor?.motors.receive(state)
        servoListView.apply(state: state)
    }

    func applyServoAxisStatus(_ status: ServoAxisStatus) {
        guard isViewLoaded else { return }
        robotModelEditor?.motors.receive(status)
        servoListView.apply(axisStatus: status)
    }

    private func setupUI() {
        let outerScroll = UIScrollView()
        outerScroll.translatesAutoresizingMaskIntoConstraints = false
        outerScroll.keyboardDismissMode = .interactive
        #if targetEnvironment(macCatalyst)
        outerScroll.contentInsetAdjustmentBehavior = .never
        #endif
        self.view.addSubview(outerScroll)
        let view = UIView()
        view.translatesAutoresizingMaskIntoConstraints = false
        outerScroll.addSubview(view)
        let preferredContentHeight = view.heightAnchor.constraint(equalTo: outerScroll.frameLayoutGuide.heightAnchor)
        preferredContentHeight.priority = .defaultLow
        NSLayoutConstraint.activate([
            outerScroll.topAnchor.constraint(equalTo: self.view.safeAreaLayoutGuide.topAnchor),
            outerScroll.leadingAnchor.constraint(equalTo: self.view.leadingAnchor),
            outerScroll.trailingAnchor.constraint(equalTo: self.view.trailingAnchor),
            outerScroll.bottomAnchor.constraint(equalTo: self.view.keyboardLayoutGuide.topAnchor),
            view.topAnchor.constraint(equalTo: outerScroll.contentLayoutGuide.topAnchor),
            view.leadingAnchor.constraint(equalTo: outerScroll.contentLayoutGuide.leadingAnchor),
            view.trailingAnchor.constraint(equalTo: outerScroll.contentLayoutGuide.trailingAnchor),
            view.bottomAnchor.constraint(equalTo: outerScroll.contentLayoutGuide.bottomAnchor),
            view.widthAnchor.constraint(equalTo: outerScroll.frameLayoutGuide.widthAnchor),
            preferredContentHeight,
            view.heightAnchor.constraint(greaterThanOrEqualTo: outerScroll.frameLayoutGuide.heightAnchor)
        ])
        let title = UILabel()
        title.translatesAutoresizingMaskIntoConstraints = false
        title.text = mode == .combined ? "Remote Controls" : (mode == .settings ? "Settings" : "ST3215 Servos")
        title.textColor = .white
        title.font = .systemFont(ofSize: 22, weight: .bold)
        view.addSubview(title)

        let doneButton = makeIconButton("xmark.circle.fill", action: #selector(doneTapped))
        view.addSubview(doneButton)

        let switchModeButton = makeButton("Switch to Robot Host", systemImageName: "arrow.triangle.2.circlepath", action: #selector(switchToRobotTapped))
        view.addSubview(switchModeButton)

        let robotModelButton = makeButton("Model Editor", systemImageName: "cube.transparent", action: #selector(openRobotModelTapped))
        view.addSubview(robotModelButton)

        let connectionTitle = UILabel()
        connectionTitle.translatesAutoresizingMaskIntoConstraints = false
        connectionTitle.text = "Connection"
        connectionTitle.textColor = UIColor.white.withAlphaComponent(0.75)
        connectionTitle.font = .systemFont(ofSize: 15, weight: .semibold)
        view.addSubview(connectionTitle)

        hostField.translatesAutoresizingMaskIntoConstraints = false
        hostField.text = "Paired Devices"
        hostField.isUserInteractionEnabled = false
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

        let connectButton = makeIconButton("qrcode.viewfinder", action: #selector(connectTapped))
        connectButton.accessibilityLabel = "Manage paired devices"
        view.addSubview(connectButton)

        connectionStatusLabel.translatesAutoresizingMaskIntoConstraints = false
        connectionStatusLabel.text = "Remembered robots"
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

        servoStatusLabel.translatesAutoresizingMaskIntoConstraints = false
        servoStatusLabel.text = "Scan to list servos"
        servoStatusLabel.textColor = UIColor.cyan.withAlphaComponent(0.8)
        servoStatusLabel.font = .monospacedSystemFont(ofSize: 12, weight: .medium)
        servoStatusLabel.textAlignment = .right
        view.addSubview(servoStatusLabel)

        [servoFromField, servoToField, servoPositionField, servoSpeedField].forEach(configureField)
        servoFromField.placeholder = "From"
        servoToField.placeholder = "To"
        servoPositionField.placeholder = "Position"
        servoSpeedField.placeholder = "Speed"
        servoFromField.text = "1"
        servoToField.text = "20"
        servoPositionField.text = "2048"
        servoSpeedField.text = "1000"

        let scanButton = makeButton("Scan", systemImageName: "dot.radiowaves.left.and.right", action: #selector(scanServosTapped))
        let moveAllButton = makeButton("Move All", systemImageName: "arrow.up.and.down.square", action: #selector(moveAllTapped))

        let scanRow = UIStackView(arrangedSubviews: [servoFromField, servoToField, scanButton])
        let moveAllRow = UIStackView(arrangedSubviews: [servoPositionField, servoSpeedField, moveAllButton])
        [scanRow, moveAllRow].forEach { row in
            row.translatesAutoresizingMaskIntoConstraints = false
            row.axis = .horizontal
            row.spacing = 8
            row.distribution = .fillEqually
            view.addSubview(row)
        }

        let commander = RemoteServoCommander(client: client)
        servoCommander = commander
        servoListView.commander = commander
        servoListView.onStatus = { [weak self] text, isError in
            self?.servoStatusLabel.text = text
            self?.servoStatusLabel.textColor = isError
                ? UIColor(red: 1.0, green: 0.35, blue: 0.35, alpha: 1)
                : UIColor.cyan.withAlphaComponent(0.8)
        }
        servoListView.presentConfirmation = { [weak self] title, message, action in
            let alert = UIAlertController(title: title, message: message, preferredStyle: .alert)
            alert.addAction(UIAlertAction(title: "Cancel", style: .cancel))
            alert.addAction(UIAlertAction(title: "Confirm", style: .destructive) { _ in action() })
            self?.present(alert, animated: true)
        }

        let servoScrollView = UIScrollView()
        servoScrollView.translatesAutoresizingMaskIntoConstraints = false
        servoScrollView.alwaysBounceVertical = true
        servoScrollView.keyboardDismissMode = .interactive
        #if targetEnvironment(macCatalyst)
        servoScrollView.contentInsetAdjustmentBehavior = .never
        #endif
        view.addSubview(servoScrollView)
        servoScrollView.addSubview(servoListView)
        servoListView.translatesAutoresizingMaskIntoConstraints = false

        NSLayoutConstraint.activate([
            title.topAnchor.constraint(equalTo: view.topAnchor, constant: 24),
            title.leadingAnchor.constraint(equalTo: view.leadingAnchor, constant: 24),

            doneButton.centerYAnchor.constraint(equalTo: title.centerYAnchor),
            doneButton.trailingAnchor.constraint(equalTo: view.trailingAnchor, constant: -24),
            doneButton.widthAnchor.constraint(equalToConstant: 44),
            doneButton.heightAnchor.constraint(equalToConstant: 38),
            title.trailingAnchor.constraint(lessThanOrEqualTo: doneButton.leadingAnchor, constant: -8)
        ])

        let connectionViews: [UIView] = [robotModelButton, switchModeButton, connectionTitle, hostField, connectButton, connectionStatusLabel, discoveredStack, motorLabel, joystickView]
        connectionViews.forEach { $0.isHidden = mode == .servos }
        if mode != .servos {
            NSLayoutConstraint.activate([
            robotModelButton.topAnchor.constraint(equalTo: title.bottomAnchor, constant: 20),
            robotModelButton.leadingAnchor.constraint(equalTo: view.leadingAnchor, constant: 24),
            robotModelButton.trailingAnchor.constraint(equalTo: view.trailingAnchor, constant: -24),
            robotModelButton.heightAnchor.constraint(equalToConstant: 44),

            switchModeButton.topAnchor.constraint(equalTo: robotModelButton.bottomAnchor, constant: 12),
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
            joystickView.heightAnchor.constraint(equalToConstant: 190)
            ])
        }

        let servoViews: [UIView] = [servoTitle, servoStatusLabel, scanRow, moveAllRow, servoScrollView]
        servoViews.forEach { $0.isHidden = mode == .settings }
        if mode == .settings {
            let openServosButton = makeButton("Servos", systemImageName: "slider.horizontal.3", action: #selector(openServosTapped))
            view.addSubview(openServosButton)
            NSLayoutConstraint.activate([
                openServosButton.topAnchor.constraint(equalTo: joystickView.bottomAnchor, constant: 24),
                openServosButton.leadingAnchor.constraint(equalTo: view.leadingAnchor, constant: 24),
                openServosButton.trailingAnchor.constraint(equalTo: view.trailingAnchor, constant: -24),
                openServosButton.heightAnchor.constraint(equalToConstant: 44),
                view.bottomAnchor.constraint(greaterThanOrEqualTo: openServosButton.bottomAnchor, constant: 24)
            ])
        } else {
            NSLayoutConstraint.activate([
            servoTitle.topAnchor.constraint(equalTo: mode == .servos ? title.bottomAnchor : joystickView.bottomAnchor, constant: 28),
            servoTitle.leadingAnchor.constraint(equalTo: view.leadingAnchor, constant: 24),

            servoStatusLabel.centerYAnchor.constraint(equalTo: servoTitle.centerYAnchor),
            servoStatusLabel.leadingAnchor.constraint(equalTo: servoTitle.trailingAnchor, constant: 12),
            servoStatusLabel.trailingAnchor.constraint(equalTo: view.trailingAnchor, constant: -24),

            scanRow.topAnchor.constraint(equalTo: servoTitle.bottomAnchor, constant: 12),
            scanRow.leadingAnchor.constraint(equalTo: view.leadingAnchor, constant: 24),
            scanRow.trailingAnchor.constraint(equalTo: view.trailingAnchor, constant: -24),
            scanRow.heightAnchor.constraint(equalToConstant: 40),

            moveAllRow.topAnchor.constraint(equalTo: scanRow.bottomAnchor, constant: 10),
            moveAllRow.leadingAnchor.constraint(equalTo: scanRow.leadingAnchor),
            moveAllRow.trailingAnchor.constraint(equalTo: scanRow.trailingAnchor),
            moveAllRow.heightAnchor.constraint(equalTo: scanRow.heightAnchor),

            servoScrollView.topAnchor.constraint(equalTo: moveAllRow.bottomAnchor, constant: 12),
            servoScrollView.leadingAnchor.constraint(equalTo: view.leadingAnchor, constant: 24),
            servoScrollView.trailingAnchor.constraint(equalTo: view.trailingAnchor, constant: -24),
            servoScrollView.bottomAnchor.constraint(equalTo: view.bottomAnchor, constant: -12),
            servoScrollView.heightAnchor.constraint(greaterThanOrEqualToConstant: 280),

            servoListView.topAnchor.constraint(equalTo: servoScrollView.contentLayoutGuide.topAnchor),
            servoListView.leadingAnchor.constraint(equalTo: servoScrollView.contentLayoutGuide.leadingAnchor),
            servoListView.trailingAnchor.constraint(equalTo: servoScrollView.contentLayoutGuide.trailingAnchor),
            servoListView.bottomAnchor.constraint(equalTo: servoScrollView.contentLayoutGuide.bottomAnchor),
            servoListView.widthAnchor.constraint(equalTo: servoScrollView.frameLayoutGuide.widthAnchor)
        ])
        }
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
            connectionStatusLabel.text = "No paired robots"
            return
        }
        connectionStatusLabel.text = "Remembered robots"
        for (index, host) in discoveredHosts.prefix(3).enumerated() {
            let button = makeButton(host.name, systemImageName: "network", action: #selector(discoveredHostTapped(_:)))
            button.tag = index
            discoveredStack.addArrangedSubview(button)
        }
    }

    @objc private func doneTapped() {
        PanelPresentation.close(self)
    }

    @objc private func openServosTapped() { onOpenServos?() }

    @objc private func openRobotModelTapped() {
        guard let commander = servoCommander else { return }
        servoListView.setScreenActive(false)
        let motors = RobotRigMotorController(commander: commander, connectionAvailable: { [weak client] in client?.isConnected == true },
                             robotIdentity: { RemoteControlIrohSession.shared.connectedPeer.map { "iroh:\($0.id)" } },
                             connectionIdentity: { RemoteControlIrohSession.shared.sessionID.uuidString })
        motors.updateIDs(modelServoIDs)
        let editor = RobotModelViewController(motors: motors)
        robotModelEditor = editor
        editor.modalPresentationStyle = .fullScreen
        present(editor, animated: true)
    }

    @objc private func switchToRobotTapped() {
        requestAppRoleSwitch(.robot)
    }

    @objc private func connectTapped() {
        client.sendStopDrive()
        RemotePairingViewController.show(from: self)
    }

    @objc private func discoveredHostTapped(_ sender: UIButton) {
        guard discoveredHosts.indices.contains(sender.tag) else { return }
        client.connect(to: discoveredHosts[sender.tag])
    }

    @objc private func scanServosTapped() {
        view.endEditing(true)
        let from = UInt8(servoFromField.text ?? "") ?? 1
        let to = UInt8(servoToField.text ?? "") ?? 20
        servoCommander?.rescanServos(from: from, to: to)
        servoStatusLabel.text = "Scanning \(from)-\(to)"
    }

    @objc private func moveAllTapped() {
        view.endEditing(true)
        let position = UInt16(servoPositionField.text ?? "") ?? 2048
        let speed = UInt16(servoSpeedField.text ?? "") ?? 1000
        servoCommander?.moveDiscoveredServos(position: min(position, 4095), speed: min(speed, 4095), acceleration: 50)
        servoStatusLabel.text = "Moving all to \(min(position, 4095))"
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

/// Relays `ServoCommanding` operations to the robot host over the remote link.
private final class RemoteServoCommander: ServoCommanding {
    private let client: RemoteControlClientService

    init(client: RemoteControlClientService) {
        self.client = client
    }

    private func send(_ type: String, _ build: (inout RemoteMessage) -> Void = { _ in }) {
        var message = RemoteMessage(type: type)
        build(&message)
        client.send(message)
    }

    func rescanServos(from: UInt8, to: UInt8) { send("scanServos") { $0.from = from; $0.to = to } }
    func moveServo(id: UInt8, position: UInt16, speed: UInt16) { send("moveServo") { $0.id = id; $0.position = position; $0.speed = speed } }
    func moveDiscoveredServos(position: UInt16, speed: UInt16, acceleration: UInt8) { send("moveAllServos") { $0.position = position; $0.speed = speed; $0.acceleration = acceleration } }
    func setServoTorque(id: UInt8, enabled: Bool) { send("setServoTorque") { $0.id = id; $0.enabled = enabled } }
    func changeServoID(currentID: UInt8, newID: UInt8) { send("changeServoID") { $0.from = currentID; $0.to = newID } }
    func calibrateServoCenter(id: UInt8) { send("calibrateServoCenter") { $0.id = id } }
    func driveServoWheel(id: UInt8, speed: Int16, acceleration: UInt8) { send("driveServoWheel") { $0.id = id; $0.wheelSpeed = speed; $0.acceleration = acceleration } }
    func setServoPositionMode(id: UInt8) { send("setServoPositionMode") { $0.id = id } }
    func refreshServoState(id: UInt8) { send("refreshServoState") { $0.id = id } }
    func trackServoAxis(id: UInt8) { send("trackServoAxis") { $0.id = id } }
    func jogServo(id: UInt8, degrees: Double) { send("jogServo") { $0.id = id; $0.degrees = degrees } }
    func beginServoJog(id: UInt8, direction: ServoJogDirection, speed: UInt16) { send("beginServoJog") { $0.id = id; $0.direction = direction.rawValue; $0.speed = speed } }
    func stopServoMotion(id: UInt8) { send("stopServoMotion") { $0.id = id } }
    func markServoTravel(id: UInt8, _ mark: ServoTravelMark) { send("markServoTravel") { $0.id = id; $0.mark = mark.rawValue } }
    func moveServoToPercent(id: UInt8, percent: Double, speed: UInt16) { send("moveServoToPercent") { $0.id = id; $0.percent = percent; $0.speed = speed } }
    func moveServoToAngle(id: UInt8, degrees: Double) { send("moveServoToAngle") { $0.id = id; $0.degrees = degrees } }
    func refreshServoAxisStatus(id: UInt8) { send("refreshServoAxisStatus") { $0.id = id } }
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

