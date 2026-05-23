//
//  LiDARViewController.swift
//  RoboCar
//
//  Created by Sammy Yousif on 1/3/26.
//

import UIKit
import ARKit
import RealityKit
import Vision

class LiDARViewController: UIViewController {
    
    // MARK: - Views
    
    private var arView: ARView!
    private var gridMapView: GridMapView!
    private var statusLabel: UILabel!
    private var positionLabel: UILabel!
    private var resetButton: UIButton!
    private var settingsButton: UIButton!
    private var micButton: UIButton!
    private var voiceStatusLabel: UILabel!
    private var obstaclePointOverlay: ObstaclePointOverlayView!
    private var personBoundingBoxOverlay: PersonBoundingBoxOverlay!
    private var handJointOverlay: HandJointOverlayView!
    private var navigateButton: UIButton!
    private var clearWaypointsButton: UIButton!
    // MARK: - Voice Assistant
    
    private let voiceAssistant = VoiceAssistantManager.shared
    
    // MARK: - State
    
    /// The global occupancy grid storing all obstacles
    let occupancyGrid = OccupancyGrid(cellSize: 0.05, gridRadius: 500)  // 50m x 50m grid with 5cm cells
    
    /// Mesh processor for converting AR mesh to grid data
    private var meshProcessor: MeshProcessor!
    
    /// Obstacle detector for collision avoidance
    private let obstacleDetector = ObstacleDetector.shared
    
    /// Display link for smooth updates
    private var displayLink: CADisplayLink?
    
    /// Track processed mesh anchors to avoid reprocessing
    private var processedMeshVersions: [UUID: Int] = [:]
    
    /// Frame counter for throttling
    private var frameCount = 0
    
    /// Counter for mesh updates to ensure newer meshes overwrite older ones
    private var meshUpdateCounter: UInt64 = 0
    
    /// Flag to pause mesh processing during reset
    private var isResetting = false

    /// Anchor IDs that existed before the last grid reset.
    /// These are ignored until ARKit removes them, preventing stale mesh
    /// data from being re-ingested after a relocalizing reset.
    private var preResetAnchorIds: Set<UUID> = []
    
    /// Guard against concurrent replan dispatches that retain ARFrames
    private var isReplanning = false
    
    /// Planned path waypoints in world coordinates (grid X/Y)
    private var plannedPath: [(x: Float, y: Float)] = []
    
    /// Planned path waypoints as 3D points for AR overlay (ARKit world space)
    private var pathPoints3D: [simd_float3] = []
    
    /// Anchor entity for the 3D path mesh in the AR scene
    private var pathAnchor: AnchorEntity?
    
    /// Navigation controller for autonomous driving
    private let pathNavigator = PathNavigator.shared
    
    /// Person tracker for follow mode
    private let personTracker = PersonTracker.shared
    
    /// Follow button (shown when follow mode is not active)
    private var followButton: UIButton!
    
    /// Whether we're currently re-planning for follow mode
    private var isFollowReplanning = false

    /// Always-on person detection results with stable ReID-based IDs
    private var alwaysOnPersonBoxes: [PersonBoxInfo] = []
    /// Known person embeddings keyed by stable UUID
    private var knownPersonEmbeddings: [UUID: [Float]] = [:]
    /// Whether a background detection request is in flight
    private var personDetectionInFlight = false
    /// ReID match threshold for always-on scanner
    private let alwaysOnReidThreshold: Float = 0.55
    /// Per-person activation gesture phase for always-on scanner (peace→fist→peace)
    private var alwaysOnActivatePhase: [UUID: PersonTracker.ActivateGesturePhase] = [:]
    /// Timestamps for activation gesture phase transitions
    private var alwaysOnActivateTimestamp: [UUID: Date] = [:]
    /// Cancel gesture phase for always-on scanner (index wag)
    private var alwaysOnCancelPhase: PersonTracker.CancelGesturePhase = .idle
    /// Last wrist X position for wag detection
    private var alwaysOnCancelLastWristX: CGFloat?
    /// Timestamp of last cancel gesture phase transition
    private var alwaysOnCancelTimestamp: Date = .distantPast
    /// Gesture timeout (seconds)
    private let alwaysOnGestureTimeout: TimeInterval = 3.0
    /// Minimum fingertip X delta for wag detection
    private let alwaysOnWagMinDelta: CGFloat = 0.04
    
    /// Index of the current route waypoint being navigated to
    private var currentRouteWaypointIndex = 0
    
    /// Whether we're navigating a multi-point route
    private var isRouteActive = false
    
    /// Whether the route is paused (user-initiated pause, not obstacle pause)
    private var isRoutePaused = false
    
    // MARK: - Lifecycle
    
    override func viewDidLoad() {
        super.viewDidLoad()
        
        view.backgroundColor = .black
        
        setupARView()
        setupGridMapView()
        setupStatusLabels()
        setupResetButton()
        setupSettingsButton()
        setupMicButton()
        setupVoiceAssistant()
        setupObstacleOverlay()
        setupPersonBoundingBoxOverlay()
        setupNavigateButton()
        setupFollowButton()
        setupMeshProcessor()
        setupObstacleDetector()
        setupNavigationController()
        
        // Use CADisplayLink for smooth updates
        displayLink = CADisplayLink(target: self, selector: #selector(updateFrame))
        displayLink?.preferredFrameRateRange = CAFrameRateRange(minimum: 15, maximum: 30, __preferred: 20)
        displayLink?.add(to: .main, forMode: .common)
    }
    
    override func viewWillAppear(_ animated: Bool) {
        super.viewWillAppear(animated)
        UIApplication.shared.isIdleTimerDisabled = true
        startARSession()
    }
    
    override func viewDidAppear(_ animated: Bool) {
        super.viewDidAppear(animated)
        // Auto-connect to ESP32 on launch
        ESP32BLEManager.shared.autoConnect()
        // Auto-start voice assistant if permissions already granted
        voiceAssistant.startIfPermitted()
    }
    
    override func viewWillDisappear(_ animated: Bool) {
        super.viewWillDisappear(animated)
        UIApplication.shared.isIdleTimerDisabled = false
        arView.session.pause()
        displayLink?.invalidate()
    }
    
    override var prefersStatusBarHidden: Bool { true }
    override var prefersHomeIndicatorAutoHidden: Bool { true }
    
    // MARK: - Setup
    
    private func setupARView() {
        arView = ARView(frame: .zero)
        arView.translatesAutoresizingMaskIntoConstraints = false
        
        // Disable automatic session configuration
        arView.automaticallyConfigureSession = false
        
        // Enable scene understanding mesh visualization (built-in wireframe)
        arView.debugOptions = [.showSceneUnderstanding]
        
        // Optimize render options
        arView.renderOptions = [.disablePersonOcclusion, .disableDepthOfField, .disableMotionBlur]
        
        view.addSubview(arView)
        
        NSLayoutConstraint.activate([
            arView.topAnchor.constraint(equalTo: view.topAnchor),
            arView.leadingAnchor.constraint(equalTo: view.leadingAnchor),
            arView.trailingAnchor.constraint(equalTo: view.trailingAnchor),
            arView.heightAnchor.constraint(equalTo: view.heightAnchor, multiplier: 0.6)
        ])
        
        arView.session.delegate = self
    }
    
    private func setupGridMapView() {
        gridMapView = GridMapView(occupancyGrid: occupancyGrid)
        gridMapView.translatesAutoresizingMaskIntoConstraints = false
        gridMapView.backgroundColor = UIColor(white: 0.1, alpha: 1.0)
        
        // Handle tap for adding/removing route waypoints
        gridMapView.onTapWorldPosition = { [weak self] worldX, worldY in
            self?.handleGridTap(x: worldX, y: worldY)
        }
        
        view.addSubview(gridMapView)
        
        NSLayoutConstraint.activate([
            gridMapView.topAnchor.constraint(equalTo: arView.bottomAnchor),
            gridMapView.leadingAnchor.constraint(equalTo: view.leadingAnchor),
            gridMapView.trailingAnchor.constraint(equalTo: view.trailingAnchor),
            gridMapView.bottomAnchor.constraint(equalTo: view.bottomAnchor)
        ])
    }
    
    private func setupStatusLabels() {
        // Status label (top left)
        statusLabel = UILabel()
        statusLabel.translatesAutoresizingMaskIntoConstraints = false
        statusLabel.textColor = .white
        statusLabel.font = .monospacedSystemFont(ofSize: 12, weight: .medium)
        statusLabel.numberOfLines = 0
        statusLabel.text = "Initializing..."
        statusLabel.layer.shadowColor = UIColor.black.cgColor
        statusLabel.layer.shadowOffset = .zero
        statusLabel.layer.shadowRadius = 2
        statusLabel.layer.shadowOpacity = 1
        
        view.addSubview(statusLabel)
        
        NSLayoutConstraint.activate([
            statusLabel.topAnchor.constraint(equalTo: view.safeAreaLayoutGuide.topAnchor, constant: 8),
            statusLabel.leadingAnchor.constraint(equalTo: view.leadingAnchor, constant: 12)
        ])
        
        // Position label (top right)
        positionLabel = UILabel()
        positionLabel.translatesAutoresizingMaskIntoConstraints = false
        positionLabel.textColor = .cyan
        positionLabel.font = .monospacedSystemFont(ofSize: 12, weight: .medium)
        positionLabel.numberOfLines = 0
        positionLabel.textAlignment = .right
        positionLabel.text = "X: 0.00\nY: 0.00\nZ: 0.00"
        positionLabel.layer.shadowColor = UIColor.black.cgColor
        positionLabel.layer.shadowOffset = .zero
        positionLabel.layer.shadowRadius = 2
        positionLabel.layer.shadowOpacity = 1
        
        view.addSubview(positionLabel)
        
        NSLayoutConstraint.activate([
            positionLabel.trailingAnchor.constraint(equalTo: view.trailingAnchor, constant: -12)
        ])
    }
    
    private func setupResetButton() {
        resetButton = UIButton(type: .system)
        resetButton.translatesAutoresizingMaskIntoConstraints = false
        resetButton.setTitle("Reset", for: .normal)
        resetButton.setTitleColor(.white, for: .normal)
        resetButton.titleLabel?.font = .systemFont(ofSize: 16, weight: .semibold)
        resetButton.backgroundColor = UIColor(red: 0.8, green: 0.2, blue: 0.2, alpha: 0.9)
        resetButton.layer.cornerRadius = 8
        resetButton.contentEdgeInsets = UIEdgeInsets(top: 10, left: 24, bottom: 10, right: 24)
        resetButton.addTarget(self, action: #selector(resetButtonTapped), for: .touchUpInside)
        
        view.addSubview(resetButton)
        
        NSLayoutConstraint.activate([
            resetButton.centerXAnchor.constraint(equalTo: view.centerXAnchor),
            resetButton.bottomAnchor.constraint(equalTo: view.safeAreaLayoutGuide.bottomAnchor, constant: -16)
        ])
    }
    
    private func setupSettingsButton() {
        settingsButton = UIButton(type: .system)
        settingsButton.translatesAutoresizingMaskIntoConstraints = false
        
        let config = UIImage.SymbolConfiguration(pointSize: 22, weight: .medium)
        settingsButton.setImage(UIImage(systemName: "gear", withConfiguration: config), for: .normal)
        settingsButton.tintColor = .white
        settingsButton.backgroundColor = UIColor(white: 0.2, alpha: 0.9)
        settingsButton.layer.cornerRadius = 22
        settingsButton.addTarget(self, action: #selector(settingsButtonTapped), for: .touchUpInside)
        
        view.addSubview(settingsButton)
        
        NSLayoutConstraint.activate([
            settingsButton.widthAnchor.constraint(equalToConstant: 44),
            settingsButton.heightAnchor.constraint(equalToConstant: 44),
            settingsButton.trailingAnchor.constraint(equalTo: view.trailingAnchor, constant: -16),
            settingsButton.bottomAnchor.constraint(equalTo: view.safeAreaLayoutGuide.bottomAnchor, constant: -16)
        ])
    }
    
    private func setupMicButton() {
        micButton = UIButton(type: .system)
        micButton.translatesAutoresizingMaskIntoConstraints = false
        
        let config = UIImage.SymbolConfiguration(pointSize: 22, weight: .medium)
        micButton.setImage(UIImage(systemName: "mic.fill", withConfiguration: config), for: .normal)
        micButton.tintColor = .white
        micButton.backgroundColor = UIColor(white: 0.2, alpha: 0.9)
        micButton.layer.cornerRadius = 22
        micButton.addTarget(self, action: #selector(micButtonTapped), for: .touchUpInside)
        
        view.addSubview(micButton)
        
        // Voice status label (shows above mic button)
        voiceStatusLabel = UILabel()
        voiceStatusLabel.translatesAutoresizingMaskIntoConstraints = false
        voiceStatusLabel.font = .systemFont(ofSize: 12, weight: .medium)
        voiceStatusLabel.textColor = .white
        voiceStatusLabel.textAlignment = .center
        voiceStatusLabel.numberOfLines = 2
        voiceStatusLabel.alpha = 0
        voiceStatusLabel.layer.shadowColor = UIColor.black.cgColor
        voiceStatusLabel.layer.shadowOffset = .zero
        voiceStatusLabel.layer.shadowRadius = 3
        voiceStatusLabel.layer.shadowOpacity = 1
        
        view.addSubview(voiceStatusLabel)
        
        NSLayoutConstraint.activate([
            micButton.widthAnchor.constraint(equalToConstant: 44),
            micButton.heightAnchor.constraint(equalToConstant: 44),
            micButton.trailingAnchor.constraint(equalTo: view.trailingAnchor, constant: -12),
            micButton.topAnchor.constraint(equalTo: view.safeAreaLayoutGuide.topAnchor, constant: 8),
            
            voiceStatusLabel.trailingAnchor.constraint(equalTo: micButton.leadingAnchor, constant: -8),
            voiceStatusLabel.centerYAnchor.constraint(equalTo: micButton.centerYAnchor),
            voiceStatusLabel.widthAnchor.constraint(lessThanOrEqualToConstant: 200),
            
            positionLabel.topAnchor.constraint(equalTo: micButton.bottomAnchor, constant: 8)
        ])
    }
    
    private func setupVoiceAssistant() {
        voiceAssistant.onStatusChanged = { [weak self] status in
            self?.updateMicUI(status: status)
        }
        
        voiceAssistant.onPartialTranscript = { [weak self] text in
            self?.voiceStatusLabel.text = "\"\(text)\""
        }
        
        voiceAssistant.onModelResponse = { [weak self] response in
            self?.voiceStatusLabel.text = response
            // Fade out after 4 seconds
            UIView.animate(withDuration: 0.3, delay: 4.0) {
                self?.voiceStatusLabel.alpha = 0
            }
        }
        
        voiceAssistant.onError = { [weak self] error in
            self?.voiceStatusLabel.text = error
            self?.voiceStatusLabel.textColor = UIColor(red: 1.0, green: 0.4, blue: 0.4, alpha: 1)
        }
    }
    
    private func updateMicUI(status: SpeechRecognitionManager.ListeningStatus) {
        let config = UIImage.SymbolConfiguration(pointSize: 22, weight: .medium)
        
        switch status {
        case .idle:
            micButton.setImage(UIImage(systemName: "mic.fill", withConfiguration: config), for: .normal)
            micButton.tintColor = .white
            micButton.backgroundColor = UIColor(white: 0.2, alpha: 0.9)
            voiceStatusLabel.alpha = 0
            micButton.layer.removeAllAnimations()
            
        case .listening:
            micButton.setImage(UIImage(systemName: "mic.fill", withConfiguration: config), for: .normal)
            micButton.tintColor = .white
            micButton.backgroundColor = UIColor(red: 0.2, green: 0.6, blue: 1.0, alpha: 0.9)
            voiceStatusLabel.text = "Say \"iPhone\"…"
            voiceStatusLabel.textColor = .white
            voiceStatusLabel.alpha = 1
            
        case .capturing:
            micButton.setImage(UIImage(systemName: "waveform", withConfiguration: config), for: .normal)
            micButton.tintColor = .white
            micButton.backgroundColor = UIColor(red: 0.1, green: 0.8, blue: 0.3, alpha: 0.9)
            voiceStatusLabel.textColor = UIColor(red: 0.3, green: 1.0, blue: 0.5, alpha: 1)
            voiceStatusLabel.alpha = 1
            // Pulse animation
            UIView.animate(withDuration: 0.5, delay: 0, options: [.repeat, .autoreverse]) {
                self.micButton.transform = CGAffineTransform(scaleX: 1.15, y: 1.15)
            }
            
        case .processing:
            micButton.setImage(UIImage(systemName: "brain", withConfiguration: config), for: .normal)
            micButton.tintColor = .white
            micButton.backgroundColor = UIColor(red: 0.6, green: 0.3, blue: 1.0, alpha: 0.9)
            voiceStatusLabel.text = "Thinking…"
            voiceStatusLabel.textColor = UIColor(red: 0.7, green: 0.5, blue: 1.0, alpha: 1)
            voiceStatusLabel.alpha = 1
            micButton.layer.removeAllAnimations()
            micButton.transform = .identity
        }
    }
    
    @objc private func micButtonTapped() {
        voiceAssistant.toggle()
    }
    
    @objc private func settingsButtonTapped() {
        let controlPanel = ControlPanelViewController()
        if let sheet = controlPanel.sheetPresentationController {
            sheet.detents = [.medium(), .large()]
            if #available(iOS 16.0, *) {
                sheet.prefersGrabberVisible = false
            }
            sheet.preferredCornerRadius = 20
        }
        present(controlPanel, animated: true)
    }
    
    @objc private func resetButtonTapped() {
        let alert = UIAlertController(
            title: "Reset Map",
            message: "This will clear the grid and reset the starting position. Are you sure?",
            preferredStyle: .alert
        )
        
        alert.addAction(UIAlertAction(title: "Cancel", style: .cancel))
        alert.addAction(UIAlertAction(title: "Reset", style: .destructive) { [weak self] _ in
            self?.performReset()
        })
        
        present(alert, animated: true)
    }
    
    private func performReset() {
        // Pause mesh processing during reset
        isResetting = true
        
        // Clear grid and mesh state
        resetGridState()
        
        // Restart AR session to reset coordinate origin
        startARSession()
        
        statusLabel.text = "🔄 Map reset"
        statusLabel.textColor = .yellow
    }
    
    /// Clears the grid, mesh tracking, and view state without restarting the AR session.
    /// Used both by the manual reset button and automatic relocalization reset.
    private func resetGridState() {
        // Stop navigation if active
        pathNavigator.stopNavigation()
        
        // Invalidate any in-flight mesh processing BEFORE clearing
        // This increments generation so stale mesh data will be discarded
        meshProcessor.reset()
        
        // Clear the occupancy grid (resets originOffset and all data)
        occupancyGrid.clear()
        
        // Remember which anchors existed before the reset so we can
        // ignore them if ARKit updates them before removing them.
        preResetAnchorIds = Set(processedMeshVersions.keys)
        
        // Reset mesh tracking
        processedMeshVersions.removeAll()
        
        // Clear planned path
        plannedPath.removeAll()
        pathPoints3D.removeAll()
        pathAnchor?.removeFromParent()
        pathAnchor = nil
        
        // Reset the grid view's initial heading
        gridMapView.resetInitialHeading()
        
        // Force redraw
        gridMapView.setNeedsDisplay()
    }
    
    private func setupObstacleOverlay() {
        obstaclePointOverlay = ObstaclePointOverlayView()
        obstaclePointOverlay.translatesAutoresizingMaskIntoConstraints = false
        
        // Add as subview of arView so project() coordinates map directly
        arView.addSubview(obstaclePointOverlay)
        
        NSLayoutConstraint.activate([
            obstaclePointOverlay.topAnchor.constraint(equalTo: arView.topAnchor),
            obstaclePointOverlay.leadingAnchor.constraint(equalTo: arView.leadingAnchor),
            obstaclePointOverlay.trailingAnchor.constraint(equalTo: arView.trailingAnchor),
            obstaclePointOverlay.bottomAnchor.constraint(equalTo: arView.bottomAnchor),
        ])
    }
    
    private func setupPersonBoundingBoxOverlay() {
        personBoundingBoxOverlay = PersonBoundingBoxOverlay()
        personBoundingBoxOverlay.translatesAutoresizingMaskIntoConstraints = false
        arView.addSubview(personBoundingBoxOverlay)
        NSLayoutConstraint.activate([
            personBoundingBoxOverlay.topAnchor.constraint(equalTo: arView.topAnchor),
            personBoundingBoxOverlay.leadingAnchor.constraint(equalTo: arView.leadingAnchor),
            personBoundingBoxOverlay.trailingAnchor.constraint(equalTo: arView.trailingAnchor),
            personBoundingBoxOverlay.bottomAnchor.constraint(equalTo: arView.bottomAnchor),
        ])
        
        handJointOverlay = HandJointOverlayView()
        handJointOverlay.translatesAutoresizingMaskIntoConstraints = false
        arView.addSubview(handJointOverlay)
        NSLayoutConstraint.activate([
            handJointOverlay.topAnchor.constraint(equalTo: arView.topAnchor),
            handJointOverlay.leadingAnchor.constraint(equalTo: arView.leadingAnchor),
            handJointOverlay.trailingAnchor.constraint(equalTo: arView.trailingAnchor),
            handJointOverlay.bottomAnchor.constraint(equalTo: arView.bottomAnchor),
        ])
    }

    private func setupNavigateButton() {
        navigateButton = UIButton(type: .system)
        navigateButton.translatesAutoresizingMaskIntoConstraints = false
        navigateButton.setTitle("Navigate", for: .normal)
        navigateButton.setTitleColor(.white, for: .normal)
        navigateButton.titleLabel?.font = .systemFont(ofSize: 15, weight: .semibold)
        navigateButton.backgroundColor = UIColor(red: 0.1, green: 0.6, blue: 1.0, alpha: 0.9)
        navigateButton.layer.cornerRadius = 18
        navigateButton.contentEdgeInsets = UIEdgeInsets(top: 8, left: 24, bottom: 8, right: 24)
        navigateButton.addTarget(self, action: #selector(navigateButtonTapped), for: .touchUpInside)
        navigateButton.isHidden = true
        
        view.addSubview(navigateButton)
        
        // Clear waypoints button
        clearWaypointsButton = UIButton(type: .system)
        clearWaypointsButton.translatesAutoresizingMaskIntoConstraints = false
        clearWaypointsButton.setTitle("Clear", for: .normal)
        clearWaypointsButton.setTitleColor(.white, for: .normal)
        clearWaypointsButton.titleLabel?.font = .systemFont(ofSize: 14, weight: .medium)
        clearWaypointsButton.backgroundColor = UIColor(white: 0.3, alpha: 0.9)
        clearWaypointsButton.layer.cornerRadius = 18
        clearWaypointsButton.contentEdgeInsets = UIEdgeInsets(top: 8, left: 16, bottom: 8, right: 16)
        clearWaypointsButton.addTarget(self, action: #selector(clearWaypointsButtonTapped), for: .touchUpInside)
        clearWaypointsButton.isHidden = true
        
        view.addSubview(clearWaypointsButton)
        
        NSLayoutConstraint.activate([
            navigateButton.centerXAnchor.constraint(equalTo: view.centerXAnchor),
            navigateButton.topAnchor.constraint(equalTo: view.safeAreaLayoutGuide.topAnchor, constant: 8),
            navigateButton.heightAnchor.constraint(equalToConstant: 36),
            
            clearWaypointsButton.leadingAnchor.constraint(equalTo: navigateButton.trailingAnchor, constant: 8),
            clearWaypointsButton.centerYAnchor.constraint(equalTo: navigateButton.centerYAnchor),
            clearWaypointsButton.heightAnchor.constraint(equalToConstant: 36),
        ])
    }
    
    @objc private func clearWaypointsButtonTapped() {
        clearRoute()
    }
    
    private func setupFollowButton() {
        followButton = UIButton(type: .system)
        followButton.translatesAutoresizingMaskIntoConstraints = false
        
        let config = UIImage.SymbolConfiguration(pointSize: 22, weight: .medium)
        followButton.setImage(UIImage(systemName: "figure.walk", withConfiguration: config), for: .normal)
        followButton.tintColor = .white
        followButton.backgroundColor = UIColor(white: 0.2, alpha: 0.9)
        followButton.layer.cornerRadius = 22
        followButton.addTarget(self, action: #selector(followButtonTapped), for: .touchUpInside)
        
        view.addSubview(followButton)
        
        NSLayoutConstraint.activate([
            followButton.widthAnchor.constraint(equalToConstant: 44),
            followButton.heightAnchor.constraint(equalToConstant: 44),
            followButton.trailingAnchor.constraint(equalTo: view.trailingAnchor, constant: -12),
            followButton.topAnchor.constraint(equalTo: positionLabel.bottomAnchor, constant: 8),
        ])
    }
    
    @objc private func followButtonTapped() {
        if personTracker.state == .idle {
            startFollowMode()
        } else {
            stopFollowMode()
        }
    }
    
    private func updateFollowButtonAppearance(isFollowing: Bool) {
        let config = UIImage.SymbolConfiguration(pointSize: 22, weight: .medium)
        if isFollowing {
            followButton.setImage(UIImage(systemName: "xmark.circle.fill", withConfiguration: config), for: .normal)
            followButton.tintColor = .systemRed
        } else {
            followButton.setImage(UIImage(systemName: "figure.walk", withConfiguration: config), for: .normal)
            followButton.tintColor = .white
        }
    }
    
    // MARK: - Follow Mode
    
    private func startFollowMode() {
        // Stop any existing navigation
        pathNavigator.stopNavigation()
        
        // Start scanning for people — gesture activation will trigger tracking
        personTracker.startScanning()
        
        // Update button to show stop icon
        updateFollowButtonAppearance(isFollowing: true)
        
        // Wire cancel gesture to stop follow mode
        personTracker.onCancelGesture = { [weak self] in
            DispatchQueue.main.async {
                self?.stopFollowMode()
            }
        }
        
        // Wire up person tracker callbacks
        personTracker.onPeopleUpdated = { [weak self] people in
            guard let self = self else { return }
            // Emit telemetry for detected people (~2 Hz, throttled by scanDetectionInterval)
            TelemetryService.shared.logPersonTracking(
                event: "scan_update",
                message: "\(people.count) people visible",
                occupancyGrid: self.occupancyGrid,
                cameraTransform: TelemetryService.shared.lastCameraTransform
            )
        }
        
        personTracker.onPersonActivated = { [weak self] person in
            guard let self = self else { return }
            let idPrefix = String(person.id.uuidString.prefix(8))
            
            SpeechSynthesisManager.shared.speak("Following started")
            
            TelemetryService.shared.logPersonTracking(
                event: "person_activated",
                message: "Person \(idPrefix) activated via gesture",
                occupancyGrid: self.occupancyGrid,
                cameraTransform: TelemetryService.shared.lastCameraTransform
            )
            
            // Now that a person is activated, start the follow navigation
            DispatchQueue.main.async {
                if let personPos = self.personTracker.trackedWorldPosition {
                    self.obstacleDetector.excludedPersonPosition = simd_float2(personPos.x, personPos.y)
                    self.pathNavigator.updateFollowTarget(x: personPos.x, y: personPos.y)
                }
                self.pathNavigator.startFollowing()
                if let personPos = self.personTracker.trackedWorldPosition {
                    self.planFollowPath(toX: personPos.x, toY: personPos.y)
                }
            }
        }
        
        personTracker.onPositionUpdated = { [weak self] worldPos in
            guard let self = self else { return }
            self.pathNavigator.updateFollowTarget(x: worldPos.x, y: worldPos.y)
            self.obstacleDetector.excludedPersonPosition = worldPos
        }
        
        personTracker.onStateChanged = { [weak self] state in
            guard let self = self else { return }
            
            TelemetryService.shared.logPersonTracking(
                event: "state_change",
                message: "State → \(state.rawValue)",
                occupancyGrid: self.occupancyGrid,
                cameraTransform: TelemetryService.shared.lastCameraTransform
            )
            
            DispatchQueue.main.async {
                switch state {
                case .scanning:
                    self.voiceStatusLabel.text = "Scanning — raise open hand to follow"
                    self.voiceStatusLabel.textColor = .cyan
                    self.voiceStatusLabel.alpha = 1
                case .lost:
                    // Person lost — turn toward their last known position
                    // instead of stopping, to attempt visual reacquisition.
                    self.pathNavigator.handleFollowTargetLost()
                    self.voiceStatusLabel.text = "Turning to last known position"
                    self.voiceStatusLabel.textColor = .orange
                    self.voiceStatusLabel.alpha = 1
                case .tracking:
                    self.voiceStatusLabel.text = "Following person"
                    self.voiceStatusLabel.textColor = UIColor(red: 0.7, green: 0.5, blue: 1.0, alpha: 1)
                    self.voiceStatusLabel.alpha = 1
                    UIView.animate(withDuration: 0.3, delay: 2.0) {
                        self.voiceStatusLabel.alpha = 0
                    }
                case .reacquiring:
                    self.voiceStatusLabel.text = "Re-acquiring person..."
                    self.voiceStatusLabel.textColor = .yellow
                    self.voiceStatusLabel.alpha = 1
                case .idle:
                    break
                }
            }
        }
    }
    
    /// Start follow mode for a specific person identified by the always-on scanner.
    private func startFollowModeForPerson(id: UUID, embedding: [Float], boundingBox: CGRect, worldPosition: simd_float2?) {
        // Stop any existing navigation
        pathNavigator.stopNavigation()
        
        SpeechSynthesisManager.shared.speak("Following started")
        
        // Wire up callbacks (same as startFollowMode)
        personTracker.onCancelGesture = { [weak self] in
            DispatchQueue.main.async {
                self?.stopFollowMode()
            }
        }
        
        personTracker.onPeopleUpdated = { [weak self] people in
            guard let self = self else { return }
            TelemetryService.shared.logPersonTracking(
                event: "scan_update",
                message: "\(people.count) people visible",
                occupancyGrid: self.occupancyGrid,
                cameraTransform: TelemetryService.shared.lastCameraTransform
            )
        }
        
        personTracker.onPersonActivated = { [weak self] person in
            guard let self = self else { return }
            let idPrefix = String(person.id.uuidString.prefix(8))
            
            TelemetryService.shared.logPersonTracking(
                event: "person_activated",
                message: "Person \(idPrefix) activated via gesture",
                occupancyGrid: self.occupancyGrid,
                cameraTransform: TelemetryService.shared.lastCameraTransform
            )
            
            DispatchQueue.main.async {
                if let personPos = self.personTracker.trackedWorldPosition {
                    self.obstacleDetector.excludedPersonPosition = simd_float2(personPos.x, personPos.y)
                    self.pathNavigator.updateFollowTarget(x: personPos.x, y: personPos.y)
                }
                self.pathNavigator.startFollowing()
                if let personPos = self.personTracker.trackedWorldPosition {
                    self.planFollowPath(toX: personPos.x, toY: personPos.y)
                }
            }
        }
        
        personTracker.onPositionUpdated = { [weak self] worldPos in
            guard let self = self else { return }
            self.pathNavigator.updateFollowTarget(x: worldPos.x, y: worldPos.y)
            self.obstacleDetector.excludedPersonPosition = worldPos
        }
        
        personTracker.onStateChanged = { [weak self] state in
            guard let self = self else { return }
            
            TelemetryService.shared.logPersonTracking(
                event: "state_change",
                message: "State → \(state.rawValue)",
                occupancyGrid: self.occupancyGrid,
                cameraTransform: TelemetryService.shared.lastCameraTransform
            )
            
            DispatchQueue.main.async {
                switch state {
                case .scanning:
                    self.voiceStatusLabel.text = "Scanning — raise open hand to follow"
                    self.voiceStatusLabel.textColor = .cyan
                    self.voiceStatusLabel.alpha = 1
                case .lost:
                    // Person lost — turn toward their last known position
                    // instead of stopping, to attempt visual reacquisition.
                    self.pathNavigator.handleFollowTargetLost()
                    self.voiceStatusLabel.text = "Turning to last known position"
                    self.voiceStatusLabel.textColor = .orange
                    self.voiceStatusLabel.alpha = 1
                case .tracking:
                    self.voiceStatusLabel.text = "Following person"
                    self.voiceStatusLabel.textColor = UIColor(red: 0.7, green: 0.5, blue: 1.0, alpha: 1)
                    self.voiceStatusLabel.alpha = 1
                    UIView.animate(withDuration: 0.3, delay: 2.0) {
                        self.voiceStatusLabel.alpha = 0
                    }
                case .reacquiring:
                    self.voiceStatusLabel.text = "Re-acquiring person..."
                    self.voiceStatusLabel.textColor = .yellow
                    self.voiceStatusLabel.alpha = 1
                case .idle:
                    break
                }
            }
        }
        
        // Activate the person directly — skip scanning since we already identified them
        personTracker.activateExternalPerson(id: id, embedding: embedding, boundingBox: boundingBox, worldPosition: worldPosition)
        
        // Update button to show stop icon
        updateFollowButtonAppearance(isFollowing: true)
    }
    
    private func stopFollowMode() {
        SpeechSynthesisManager.shared.speak("Following stopped")
        
        TelemetryService.shared.logPersonTracking(
            event: "state_change",
            message: "Follow mode stopped by user",
            occupancyGrid: occupancyGrid,
            cameraTransform: TelemetryService.shared.lastCameraTransform
        )
        personTracker.stopTracking()
        pathNavigator.stopNavigation()
        obstacleDetector.excludedPersonPosition = nil
        personBoundingBoxOverlay.people = []
        personBoundingBoxOverlay.setNeedsDisplay()
        handJointOverlay.hands = []
        handJointOverlay.setNeedsDisplay()
        gridMapView.personPositions = []
        isFollowReplanning = false
        
        gridMapView.pathTargetPoint = nil
        plannedPath.removeAll()
        gridMapView.plannedPath = []
        pathAnchor?.removeFromParent()
        pathAnchor = nil
        gridMapView.setNeedsDisplay()
        
        updateFollowButtonAppearance(isFollowing: false)
        
        // Reset gesture state machines
        alwaysOnActivatePhase.removeAll()
        alwaysOnActivateTimestamp.removeAll()
        alwaysOnCancelPhase = .idle
        alwaysOnCancelLastWristX = nil
    }
    
    @objc private func handleStartFollowing() {
        startFollowMode()
    }
    
    @objc private func handleStopFollowing() {
        stopFollowMode()
    }
    
    /// Plan a path to the followed person's current position (with standoff).
    private func planFollowPath(toX: Float, toY: Float) {
        guard !isFollowReplanning else { return }
        isFollowReplanning = true
        
        let pos = occupancyGrid.devicePosition
        let capturedTransform = TelemetryService.shared.lastCameraTransform
        
        // Compute standoff target: stop short of the person
        let dx = pos.x - toX
        let dy = pos.y - toY
        let dist = sqrtf(dx * dx + dy * dy)
        let standoff = pathNavigator.followStandoff
        
        let targetX: Float, targetY: Float
        if dist > standoff {
            targetX = toX + (dx / dist) * standoff
            targetY = toY + (dy / dist) * standoff
        } else {
            targetX = pos.x  // Already close enough
            targetY = pos.y
        }
        
        DispatchQueue.global(qos: .userInitiated).async { [weak self] in
            guard let self = self else { return }
            
            let strictPath = self.occupancyGrid.findPath(fromX: pos.x, fromY: pos.y, toX: targetX, toY: targetY)
            let greedyPath = self.occupancyGrid.findPathGreedy(fromX: pos.x, fromY: pos.y, toX: targetX, toY: targetY)
            
            func pathLength(_ p: [(x: Float, y: Float)]) -> Float {
                var total: Float = 0
                for i in 1..<p.count {
                    let pdx = p[i].x - p[i-1].x
                    let pdy = p[i].y - p[i-1].y
                    total += sqrtf(pdx * pdx + pdy * pdy)
                }
                return total
            }
            
            let path: [(x: Float, y: Float)]
            if strictPath.isEmpty && greedyPath.isEmpty {
                path = []
            } else if strictPath.isEmpty {
                path = greedyPath
            } else if greedyPath.isEmpty {
                path = strictPath
            } else {
                let strictLen = pathLength(strictPath)
                let greedyLen = pathLength(greedyPath)
                path = greedyLen < strictLen * 0.75 ? greedyPath : strictPath
            }
            
            DispatchQueue.main.async {
                self.isFollowReplanning = false
                guard !path.isEmpty else { return }
                self.plannedPath = path
                self.gridMapView.plannedPath = path
                self.gridMapView.pathTargetPoint = (x: targetX, y: targetY)
                self.gridMapView.setNeedsDisplay()
                self.rebuildPathMesh(from: path)
                self.pathNavigator.updateFollowPath(path)
            }
        }
    }
    
    private func setupNavigationController() {
        pathNavigator.occupancyGrid = occupancyGrid
        
        // Start telemetry service
        TelemetryService.shared.start()
        
        // Listen for server-initiated navigation commands
        NotificationCenter.default.addObserver(self, selector: #selector(handleServerSetNavTarget(_:)), name: .serverSetNavTarget, object: nil)
        NotificationCenter.default.addObserver(self, selector: #selector(handleServerStartNavigation), name: .serverStartNavigation, object: nil)
        NotificationCenter.default.addObserver(self, selector: #selector(handleServerStopNavigation), name: .serverStopNavigation, object: nil)
        NotificationCenter.default.addObserver(self, selector: #selector(handleStartFollowing), name: .startFollowing, object: nil)
        NotificationCenter.default.addObserver(self, selector: #selector(handleStopFollowing), name: .stopFollowing, object: nil)
        NotificationCenter.default.addObserver(self, selector: #selector(handleServerAddRoutePoint(_:)), name: .serverAddRoutePoint, object: nil)
        NotificationCenter.default.addObserver(self, selector: #selector(handleServerRunRoute), name: .serverRunRoute, object: nil)
        NotificationCenter.default.addObserver(self, selector: #selector(handleServerPauseRoute), name: .serverPauseRoute, object: nil)
        NotificationCenter.default.addObserver(self, selector: #selector(handleServerClearRoute), name: .serverClearRoute, object: nil)
        
        let previousOnStateChanged = pathNavigator.onStateChanged
        pathNavigator.onStateChanged = { [weak self] state in
            // Telemetry: log nav state change
            if let oldState = self?.pathNavigator.state {
                // The didSet fires before the value is committed to callers,
                // but we get the new value from the callback. Use a simple reason.
                var reason = "unknown"
                switch state {
                case .idle: reason = "cancelled"
                case .navigating: reason = "started"
                case .paused: reason = "obstacle_blocking"
                case .arrived: reason = "arrived"
                case .following: reason = "following"
                case .followPaused: reason = "follow_paused"
                }
                // Use cached transform to avoid retaining an ARFrame
                let cachedTransform = TelemetryService.shared.lastCameraTransform
                TelemetryService.shared.logNavStateChange(
                    from: oldState, to: state, reason: reason,
                    occupancyGrid: self?.occupancyGrid,
                    cameraTransform: cachedTransform
                )
            }

            DispatchQueue.main.async {
                self?.updateNavigateButtonForState(state)
                // Handle route waypoint arrival
                if state == .arrived, let self = self, self.isRouteActive {
                    self.onRouteWaypointArrived()
                }
            }
            previousOnStateChanged?(state)
        }
        
        pathNavigator.onPathUpdated = { [weak self] path in
            DispatchQueue.main.async {
                guard let self = self else { return }
                self.plannedPath = path
                self.gridMapView.plannedPath = path
                self.gridMapView.setNeedsDisplay()
                self.rebuildPathMesh(from: path)
            }
        }
    }
    
    @objc private func navigateButtonTapped() {
        if pathNavigator.state == .navigating || pathNavigator.state == .paused {
            // Stop navigation
            stopRoute()
        } else if pathNavigator.state == .following || pathNavigator.state == .followPaused {
            // Stop following
            stopFollowMode()
        } else if isRoutePaused {
            // Resume paused route
            resumeRoute()
        } else if !gridMapView.routeWaypoints.isEmpty {
            // Start multi-waypoint route
            startRoute()
        } else if !plannedPath.isEmpty, let target = gridMapView.pathTargetPoint {
            // Start navigating along the current planned path
            pathNavigator.startNavigation(to: target, path: plannedPath)
        }
    }
    
    // MARK: - Multi-Waypoint Route
    
    /// Handle a tap on the grid map — add or remove a waypoint
    private func handleGridTap(x: Float, y: Float) {
        // Don't allow editing waypoints while actively navigating a route
        guard !isRouteActive else {
            print("[Route] Cannot modify waypoints while route is active")
            return
        }
        
        // Check if tap is near an existing waypoint → remove it
        let deleteThreshold: Float = 0.3  // 30cm tap radius for deletion
        if let removeIndex = gridMapView.routeWaypoints.firstIndex(where: { wp in
            let dx = wp.x - x
            let dy = wp.y - y
            return sqrt(dx * dx + dy * dy) < deleteThreshold
        }) {
            let removed = gridMapView.routeWaypoints.remove(at: removeIndex)
            gridMapView.setNeedsDisplay()
            updateNavigateButtonForRouteState()
            print("[Route] Removed waypoint #\(removeIndex + 1) at (\(String(format: "%.2f", removed.x)), \(String(format: "%.2f", removed.y)))")
            
            TelemetryService.shared.logNavCommandAck(
                cmd: "remove_route_point", success: true,
                message: "Waypoint #\(removeIndex + 1) removed",
                waypointCount: gridMapView.routeWaypoints.count
            )
            
            // Re-plan route preview with remaining waypoints
            planRoutePreview()
            return
        }
        
        // Otherwise add a new waypoint
        addRouteWaypoint(x: x, y: y)
    }
    
    /// Add a waypoint to the multi-point route
    private func addRouteWaypoint(x: Float, y: Float) {
        // Don't allow adding waypoints while actively navigating a route
        guard !isRouteActive else {
            print("[Route] Cannot add waypoints while route is active")
            return
        }
        
        gridMapView.routeWaypoints.append((x: x, y: y))
        gridMapView.setNeedsDisplay()
        updateNavigateButtonForRouteState()
        
        print("[Route] Added waypoint #\(gridMapView.routeWaypoints.count) at (\(String(format: "%.2f", x)), \(String(format: "%.2f", y)))")
        
        TelemetryService.shared.logNavCommandAck(
            cmd: "add_route_point", success: true,
            message: "Waypoint #\(gridMapView.routeWaypoints.count) added",
            target: Vec2(x: x, y: y),
            waypointCount: gridMapView.routeWaypoints.count
        )
        
        // Plan route preview including the new waypoint
        planRoutePreview()
    }
    
    /// Plan paths between all consecutive route waypoints for preview on grid + AR
    private func planRoutePreview() {
        let waypoints = gridMapView.routeWaypoints
        guard !waypoints.isEmpty else {
            gridMapView.routePreviewPaths = []
            gridMapView.plannedPath = []
            gridMapView.setNeedsDisplay()
            pathAnchor?.removeFromParent()
            pathAnchor = nil
            return
        }
        
        let devicePos = occupancyGrid.devicePosition
        
        // Build list of segment pairs: device → WP1, WP1 → WP2, ...
        var origins: [(x: Float, y: Float)] = [(x: devicePos.x, y: devicePos.y)]
        origins.append(contentsOf: waypoints.dropLast())
        let destinations = waypoints
        
        let previewId = UUID().uuidString
        let segmentCount = destinations.count
        let routeWPs = waypoints  // capture for telemetry
        
        // Plan all segments off main thread
        DispatchQueue.global(qos: .userInitiated).async { [weak self] in
            guard let self = self else { return }
            
            var segmentPaths: [[(x: Float, y: Float)]] = []
            var fullPath: [(x: Float, y: Float)] = []
            
            for i in 0..<destinations.count {
                let from = origins[i]
                let to = destinations[i]
                
                // Snap target out of obstacles
                var targetX = to.x
                var targetY = to.y
                if self.occupancyGrid.getStateAtWorld(x: to.x, y: to.y) == .occupied {
                    if let freePoint = self.occupancyGrid.nearestFreeWorldPoint(x: to.x, y: to.y) {
                        targetX = freePoint.x
                        targetY = freePoint.y
                    }
                }
                
                let strict = self.occupancyGrid.findPath(fromX: from.x, fromY: from.y, toX: targetX, toY: targetY)
                let greedy = self.occupancyGrid.findPathGreedy(fromX: from.x, fromY: from.y, toX: targetX, toY: targetY)
                
                let segPath: [(x: Float, y: Float)]
                if strict.isEmpty && greedy.isEmpty {
                    // No path found — use direct line as fallback
                    segPath = [from, to]
                } else if strict.isEmpty {
                    segPath = greedy
                } else if greedy.isEmpty {
                    segPath = strict
                } else {
                    func pathLen(_ p: [(x: Float, y: Float)]) -> Float {
                        var t: Float = 0
                        for j in 1..<p.count { let dx = p[j].x - p[j-1].x; let dy = p[j].y - p[j-1].y; t += sqrtf(dx*dx+dy*dy) }
                        return t
                    }
                    segPath = pathLen(greedy) < pathLen(strict) * 0.75 ? greedy : strict
                }
                
                // Determine which algorithm was chosen
                let segAlgorithm: String
                if strict.isEmpty && greedy.isEmpty {
                    segAlgorithm = "direct_fallback"
                } else if strict.isEmpty {
                    segAlgorithm = "greedy"
                } else if greedy.isEmpty {
                    segAlgorithm = "astar"
                } else {
                    func segPathLen(_ p: [(x: Float, y: Float)]) -> Float {
                        var t: Float = 0
                        for j in 1..<p.count { let dx = p[j].x - p[j-1].x; let dy = p[j].y - p[j-1].y; t += sqrtf(dx*dx+dy*dy) }
                        return t
                    }
                    segAlgorithm = segPathLen(greedy) < segPathLen(strict) * 0.75 ? "greedy_shorter" : "astar"
                }
                
                // Emit telemetry for this segment
                TelemetryService.shared.logRoutePreviewSegment(
                    previewId: previewId,
                    segmentIndex: i,
                    segmentCount: segmentCount,
                    origin: from,
                    target: (x: targetX, y: targetY),
                    waypoints: segPath,
                    algorithm: segAlgorithm,
                    routeWaypoints: routeWPs,
                    occupancyGrid: self.occupancyGrid,
                    cameraTransform: TelemetryService.shared.lastCameraTransform
                )
                
                segmentPaths.append(segPath)
                // Append to full path (skip first point of subsequent segments to avoid duplication)
                if i == 0 {
                    fullPath.append(contentsOf: segPath)
                } else if segPath.count > 1 {
                    fullPath.append(contentsOf: segPath.dropFirst())
                }
            }
            
            // Densify full path for AR mesh
            var densePath: [(x: Float, y: Float)] = []
            let spacing: Float = 0.05
            for i in 0..<fullPath.count {
                let pt = fullPath[i]
                densePath.append(pt)
                if i < fullPath.count - 1 {
                    let next = fullPath[i + 1]
                    let dx = next.x - pt.x
                    let dy = next.y - pt.y
                    let dist = sqrt(dx * dx + dy * dy)
                    let steps = Int(dist / spacing)
                    guard steps > 1 else { continue }
                    for s in 1..<steps {
                        let t = Float(s) / Float(steps)
                        densePath.append((x: pt.x + dx * t, y: pt.y + dy * t))
                    }
                }
            }
            
            let floorY = self.meshProcessor.floorHeight
            let points3D = densePath.map { pt in
                simd_float3(pt.x, floorY + 0.02, pt.y)
            }
            let pathMesh = self.createPathMesh(from: points3D, width: 0.05)
            
            DispatchQueue.main.async { [weak self] in
                guard let self = self else { return }
                self.gridMapView.routePreviewPaths = segmentPaths
                self.gridMapView.plannedPath = fullPath
                self.gridMapView.setNeedsDisplay()
                
                // Update AR path mesh
                self.pathAnchor?.removeFromParent()
                self.pathAnchor = nil
                
                if let mesh = pathMesh {
                    var material = UnlitMaterial()
                    material.color = .init(tint: UIColor(red: 0.0, green: 0.9, blue: 0.3, alpha: 0.9))
                    let entity = ModelEntity(mesh: mesh, materials: [material])
                    let anchor = AnchorEntity(world: .zero)
                    anchor.addChild(entity)
                    self.arView.scene.addAnchor(anchor)
                    self.pathAnchor = anchor
                }
                
                print("[Route] Preview planned: \(segmentPaths.count) segments, \(fullPath.count) total waypoints")
            }
        }
    }
    
    
    /// Start navigating the multi-point route from the beginning
    private func startRoute() {
        let waypoints = gridMapView.routeWaypoints
        guard !waypoints.isEmpty else {
            print("[Route] No waypoints to navigate")
            return
        }
        
        currentRouteWaypointIndex = 0
        isRouteActive = true
        isRoutePaused = false
        gridMapView.activeRouteWaypointIndex = 0
        gridMapView.setNeedsDisplay()
        
        print("[Route] Starting route with \(waypoints.count) waypoints")
        navigateToCurrentRouteWaypoint()
    }
    
    /// Pause the current route (stops the robot but remembers position in route)
    private func pauseRoute() {
        guard isRouteActive else { return }
        isRouteActive = false
        isRoutePaused = true
        pathNavigator.stopNavigation()
        
        // Keep visual state but stop driving
        gridMapView.setNeedsDisplay()
        updateNavigateButtonForRouteState()
        
        print("[Route] Route paused at waypoint #\(currentRouteWaypointIndex + 1)")
        TelemetryService.shared.logNavCommandAck(
            cmd: "pause_route", success: true,
            message: "Route paused at waypoint #\(currentRouteWaypointIndex + 1)"
        )
    }
    
    /// Resume a paused route from where it left off
    private func resumeRoute() {
        guard isRoutePaused, !gridMapView.routeWaypoints.isEmpty else { return }
        isRouteActive = true
        isRoutePaused = false
        
        print("[Route] Resuming route from waypoint #\(currentRouteWaypointIndex + 1)")
        navigateToCurrentRouteWaypoint()
        
        TelemetryService.shared.logNavCommandAck(
            cmd: "run_route", success: true,
            message: "Route resumed from waypoint #\(currentRouteWaypointIndex + 1)",
            waypointCount: gridMapView.routeWaypoints.count
        )
    }
    
    /// Stop and clear the route entirely
    private func stopRoute() {
        pathNavigator.stopNavigation()
        isRouteActive = false
        isRoutePaused = false
        currentRouteWaypointIndex = 0
        gridMapView.pathTargetPoint = nil
        plannedPath.removeAll()
        gridMapView.plannedPath = []
        gridMapView.routePreviewPaths = []
        pathAnchor?.removeFromParent()
        pathAnchor = nil
        gridMapView.setNeedsDisplay()
        updateNavigateButtonForRouteState()
    }
    
    /// Clear all route waypoints
    private func clearRoute() {
        stopRoute()
        gridMapView.clearRouteWaypoints()
        updateNavigateButtonForRouteState()
        
        print("[Route] Route cleared")
        TelemetryService.shared.logNavCommandAck(
            cmd: "clear_route", success: true,
            message: "Route cleared"
        )
    }
    
    /// Navigate to the current waypoint in the route
    private func navigateToCurrentRouteWaypoint() {
        let waypoints = gridMapView.routeWaypoints
        guard currentRouteWaypointIndex < waypoints.count else {
            // All waypoints reached!
            isRouteActive = false
            isRoutePaused = false
            print("[Route] ✅ All \(waypoints.count) waypoints reached!")
            updateNavigateButtonForRouteState()
            return
        }
        
        let wp = waypoints[currentRouteWaypointIndex]
        gridMapView.activeRouteWaypointIndex = currentRouteWaypointIndex
        gridMapView.setNeedsDisplay()
        
        print("[Route] Navigating to waypoint #\(currentRouteWaypointIndex + 1)/\(waypoints.count) at (\(String(format: "%.2f", wp.x)), \(String(format: "%.2f", wp.y)))")
        
        // Plan path to this waypoint
        planPath(toX: wp.x, toY: wp.y, reason: "route_waypoint")
    }
    
    /// Called when the robot arrives at a waypoint — advance to next
    private func onRouteWaypointArrived() {
        currentRouteWaypointIndex += 1
        gridMapView.activeRouteWaypointIndex = currentRouteWaypointIndex
        gridMapView.setNeedsDisplay()
        
        if currentRouteWaypointIndex < gridMapView.routeWaypoints.count {
            // Brief pause then navigate to next waypoint
            DispatchQueue.main.asyncAfter(deadline: .now() + 0.5) { [weak self] in
                guard let self = self, self.isRouteActive else { return }
                self.navigateToCurrentRouteWaypoint()
            }
        } else {
            // Route complete
            isRouteActive = false
            isRoutePaused = false
            print("[Route] ✅ Route complete!")
            updateNavigateButtonForRouteState()
        }
    }
    
    /// Update the navigate button state for route mode
    private func updateNavigateButtonForRouteState() {
        if isRouteActive {
            navigateButton.setTitle("Stop Route", for: .normal)
            navigateButton.backgroundColor = UIColor(red: 0.9, green: 0.2, blue: 0.2, alpha: 0.9)
            navigateButton.isHidden = false
            clearWaypointsButton.isHidden = true
        } else if isRoutePaused {
            navigateButton.setTitle("Resume Route", for: .normal)
            navigateButton.backgroundColor = UIColor(red: 0.2, green: 0.7, blue: 1.0, alpha: 0.9)
            navigateButton.isHidden = false
            clearWaypointsButton.isHidden = false
        } else if !gridMapView.routeWaypoints.isEmpty {
            navigateButton.setTitle("Run Route (\(gridMapView.routeWaypoints.count))", for: .normal)
            navigateButton.backgroundColor = UIColor(red: 0.1, green: 0.6, blue: 1.0, alpha: 0.9)
            navigateButton.isHidden = false
            clearWaypointsButton.isHidden = false
        } else {
            navigateButton.isHidden = true
            clearWaypointsButton.isHidden = true
        }
    }
    
    // MARK: - Server Navigation Commands
    
    @objc private func handleServerSetNavTarget(_ notification: Notification) {
        guard let info = notification.userInfo,
              let x = info["x"] as? Float,
              let y = info["y"] as? Float else {
            TelemetryService.shared.logNavCommandAck(cmd: "set_nav_target", success: false, message: "Invalid parameters")
            return
        }
        
        guard occupancyGrid.freeCount > 0 else {
            TelemetryService.shared.logNavCommandAck(
                cmd: "set_nav_target", success: false,
                message: "No mesh data in grid",
                target: Vec2(x: x, y: y)
            )
            return
        }
        
        // If navigation is active, stop it before planning a new path
        if pathNavigator.state == .navigating || pathNavigator.state == .paused {
            pathNavigator.stopNavigation()
        }
        
        planPath(toX: x, toY: y, reason: "server")
    }
    
    @objc private func handleServerStartNavigation() {
        guard !plannedPath.isEmpty, let target = gridMapView.pathTargetPoint else {
            TelemetryService.shared.logNavCommandAck(
                cmd: "start_navigation", success: false,
                message: "No planned path — send set_nav_target first"
            )
            return
        }
        
        pathNavigator.startNavigation(to: target, path: plannedPath)
        TelemetryService.shared.logNavCommandAck(
            cmd: "start_navigation", success: true,
            message: "Navigation started to (\(String(format: "%.2f", target.x)), \(String(format: "%.2f", target.y)))",
            target: Vec2(x: target.x, y: target.y),
            waypointCount: plannedPath.count
        )
    }
    
    @objc private func handleServerStopNavigation() {
        let wasNavigating = pathNavigator.state == .navigating || pathNavigator.state == .paused
        stopRoute()
        
        TelemetryService.shared.logNavCommandAck(
            cmd: "stop_navigation", success: true,
            message: wasNavigating ? "Navigation stopped" : "Already idle"
        )
    }
    
    @objc private func handleServerAddRoutePoint(_ notification: Notification) {
        guard let info = notification.userInfo,
              let x = info["x"] as? Float,
              let y = info["y"] as? Float else {
            TelemetryService.shared.logNavCommandAck(cmd: "add_route_point", success: false, message: "Invalid parameters")
            return
        }
        addRouteWaypoint(x: x, y: y)
    }
    
    @objc private func handleServerRunRoute() {
        let waypoints = gridMapView.routeWaypoints
        guard !waypoints.isEmpty else {
            TelemetryService.shared.logNavCommandAck(
                cmd: "run_route", success: false,
                message: "No route waypoints — send add_route_point first"
            )
            return
        }
        
        if isRoutePaused {
            resumeRoute()
        } else {
            startRoute()
            TelemetryService.shared.logNavCommandAck(
                cmd: "run_route", success: true,
                message: "Route started with \(waypoints.count) waypoints",
                waypointCount: waypoints.count
            )
        }
    }
    
    @objc private func handleServerPauseRoute() {
        guard isRouteActive else {
            TelemetryService.shared.logNavCommandAck(
                cmd: "pause_route", success: false,
                message: "No active route to pause"
            )
            return
        }
        pauseRoute()
    }
    
    @objc private func handleServerClearRoute() {
        clearRoute()
    }
    
    private func updateNavigateButtonForState(_ state: NavigationState) {
        // If a route is active, use route-specific button states
        if isRouteActive || isRoutePaused || !gridMapView.routeWaypoints.isEmpty {
            updateNavigateButtonForRouteState()
            // Override with nav-specific states if actively navigating within a route
            if isRouteActive {
                switch state {
                case .navigating:
                    navigateButton.setTitle("Stop Route (\(currentRouteWaypointIndex + 1)/\(gridMapView.routeWaypoints.count))", for: .normal)
                    navigateButton.backgroundColor = UIColor(red: 0.9, green: 0.2, blue: 0.2, alpha: 0.9)
                    navigateButton.isHidden = false
                case .paused:
                    navigateButton.setTitle("Stop Route (paused)", for: .normal)
                    navigateButton.backgroundColor = UIColor(red: 0.9, green: 0.6, blue: 0.1, alpha: 0.9)
                    navigateButton.isHidden = false
                default:
                    break
                }
            }
            return
        }
        
        switch state {
        case .idle:
            if gridMapView.pathTargetPoint != nil && !plannedPath.isEmpty {
                navigateButton.setTitle("Navigate", for: .normal)
                navigateButton.backgroundColor = UIColor(red: 0.1, green: 0.6, blue: 1.0, alpha: 0.9)
                navigateButton.isHidden = false
            } else {
                navigateButton.isHidden = true
            }
        case .navigating:
            navigateButton.setTitle("Stop", for: .normal)
            navigateButton.backgroundColor = UIColor(red: 0.9, green: 0.2, blue: 0.2, alpha: 0.9)
            navigateButton.isHidden = false
        case .paused:
            navigateButton.setTitle("Stop", for: .normal)
            navigateButton.backgroundColor = UIColor(red: 0.9, green: 0.6, blue: 0.1, alpha: 0.9)
            navigateButton.isHidden = false
        case .arrived:
            navigateButton.setTitle("Arrived ✓", for: .normal)
            navigateButton.backgroundColor = UIColor(red: 0.1, green: 0.8, blue: 0.3, alpha: 0.9)
            navigateButton.isHidden = false
            // Auto-hide after a delay
            DispatchQueue.main.asyncAfter(deadline: .now() + 3) { [weak self] in
                guard self?.pathNavigator.state == .arrived else { return }
                self?.navigateButton.isHidden = true
                self?.gridMapView.pathTargetPoint = nil
                self?.plannedPath.removeAll()
                self?.gridMapView.plannedPath = []
                self?.pathAnchor?.removeFromParent()
                self?.pathAnchor = nil
                self?.gridMapView.setNeedsDisplay()
            }
            
        case .following:
            navigateButton.isHidden = true
            updateFollowButtonAppearance(isFollowing: true)
            
        case .followPaused:
            navigateButton.isHidden = true
            updateFollowButtonAppearance(isFollowing: true)
        }
    }
    
    private func setupMeshProcessor() {
        meshProcessor = MeshProcessor(occupancyGrid: occupancyGrid)
    }
    
    private func setupObstacleDetector() {
        obstacleDetector.occupancyGrid = occupancyGrid
        MotorCalibrator.shared.occupancyGrid = occupancyGrid
        gridMapView.obstacleDetector = obstacleDetector
        
        // Telemetry: log obstacle state changes
        obstacleDetector.onObstacleStateChanged = { [weak self] detected, distance in
            guard let self = self else { return }
            let action = detected ? "detected" : "cleared"
            let motor = MotorSnapshot(fromBLEData: ESP32BLEManager.shared.lastMotorDataPublic)
            // Use cached transform to avoid retaining an ARFrame
            let cachedTransform = TelemetryService.shared.lastCameraTransform
            TelemetryService.shared.logObstacleEvent(
                action: action,
                obstacles: self.obstacleDetector.nearbyObstacles,
                motorBefore: motor,
                motorAfter: detected ? .zero : motor,
                triggerSource: "mesh_grid",
                occupancyGrid: self.occupancyGrid,
                cameraTransform: cachedTransform
            )
        }
    }
    
    // MARK: - AR Session
    
    private func startARSession() {
        guard ARWorldTrackingConfiguration.supportsSceneReconstruction(.meshWithClassification) else {
            statusLabel.text = "⚠️ LiDAR not available"
            statusLabel.textColor = .red
            return
        }
        
        let config = ARWorldTrackingConfiguration()
        config.sceneReconstruction = .meshWithClassification
        config.environmentTexturing = .automatic
        config.planeDetection = [.horizontal, .vertical]
        
        // Enable frame semantics for better mesh
        if ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth) {
            config.frameSemantics.insert(.sceneDepth)
        }
        
        arView.session.run(config, options: [.resetTracking, .removeExistingAnchors])
        
        statusLabel.text = "🔄 Starting AR..."
        statusLabel.textColor = .yellow
    }
    
    // MARK: - Pathfinding
    
    /// Plan a path from the current device position to the tapped world position.
    /// Runs A* on a background thread, then updates the grid and AR overlays on main.
    /// Falls back to greedy search (through unknown cells) if A* finds nothing.
    /// - Parameter reason: Why the path is being planned ("user_tap", "server", "voice", etc.)
    private func planPath(toX: Float, toY: Float, reason: String = "user_tap") {
        // Don't attempt pathfinding when the grid has no scanned data
        guard occupancyGrid.freeCount > 0 else {
            print("[Path] ⚠️ No mesh data in grid — ignoring tap")
            return
        }
        
        // If navigation is active, stop it before re-planning
        if pathNavigator.state == .navigating || pathNavigator.state == .paused {
            pathNavigator.stopNavigation()
        }
        
        // If the target is inside an obstacle (occupied cell), snap it to the
        // nearest free point. Leave unknown-territory targets as-is so the
        // greedy pathfinder can route through unmapped areas.
        var targetX = toX
        var targetY = toY
        if occupancyGrid.getStateAtWorld(x: toX, y: toY) == .occupied {
            if let freePoint = occupancyGrid.nearestFreeWorldPoint(x: toX, y: toY) {
                targetX = freePoint.x
                targetY = freePoint.y
            }
        }
        
        let devicePos = occupancyGrid.devicePosition
        let startX = devicePos.x
        let startY = devicePos.y
        
        // Set target point immediately for visual feedback
        gridMapView.pathTargetPoint = (x: targetX, y: targetY)
        gridMapView.plannedPath = []
        gridMapView.setNeedsDisplay()
        
        // Run pathfinding off the main thread
        DispatchQueue.global(qos: .userInitiated).async { [weak self] in
            guard let self = self else { return }
            
            let planStart = CFAbsoluteTimeGetCurrent()
            
            // Helper: total Euclidean length of a path in meters
            func pathLength(_ p: [(x: Float, y: Float)]) -> Float {
                var total: Float = 0
                for i in 1..<p.count {
                    let dx = p[i].x - p[i-1].x
                    let dy = p[i].y - p[i-1].y
                    total += sqrtf(dx * dx + dy * dy)
                }
                return total
            }
            
            // Run both pathfinders: strict (free-only) and greedy (allows unknown)
            let strictPath = self.occupancyGrid.findPath(fromX: startX, fromY: startY, toX: targetX, toY: targetY)
            let greedyPath = self.occupancyGrid.findPathGreedy(fromX: startX, fromY: startY, toX: targetX, toY: targetY)
            
            // Pick the best path:
            //  - If only one succeeded, use it
            //  - If both succeeded, prefer the greedy (exploration) path when it's
            //    meaningfully shorter — the car will scan unknown cells as it drives.
            //    Threshold: use greedy if it's < 75% of the strict path length.
            let path: [(x: Float, y: Float)]
            let algorithm: String
            if strictPath.isEmpty && greedyPath.isEmpty {
                path = []
                algorithm = "none"
            } else if strictPath.isEmpty {
                path = greedyPath
                algorithm = "greedy"
            } else if greedyPath.isEmpty {
                path = strictPath
                algorithm = "astar"
            } else {
                let strictLen = pathLength(strictPath)
                let greedyLen = pathLength(greedyPath)
                if greedyLen < strictLen * 0.75 {
                    path = greedyPath
                    algorithm = "greedy_shorter"
                } else {
                    path = strictPath
                    algorithm = "astar"
                }
            }
            
            let planDurationMs = Float((CFAbsoluteTimeGetCurrent() - planStart) * 1000)
            
            // Log route to telemetry (use cached transform to avoid retaining ARFrame)
            if !path.isEmpty {
                TelemetryService.shared.logRoutePlanned(
                    target: (x: targetX, y: targetY),
                    origin: (x: startX, y: startY),
                    waypoints: path,
                    algorithm: algorithm,
                    reason: reason,
                    planDurationMs: planDurationMs,
                    occupancyGrid: self.occupancyGrid,
                    cameraTransform: TelemetryService.shared.lastCameraTransform
                )
            }
            
            // Densify the path: interpolate points along each segment
            var densePath: [(x: Float, y: Float)] = []
            let spacing: Float = 0.05  // 5cm between points
            
            for i in 0..<path.count {
                let pt = path[i]
                densePath.append(pt)
                
                if i < path.count - 1 {
                    let next = path[i + 1]
                    let dx = next.x - pt.x
                    let dy = next.y - pt.y
                    let dist = sqrt(dx * dx + dy * dy)
                    let steps = Int(dist / spacing)
                    guard steps > 1 else { continue }
                    
                    for s in 1..<steps {
                        let t = Float(s) / Float(steps)
                        densePath.append((x: pt.x + dx * t, y: pt.y + dy * t))
                    }
                }
            }
            
            // Convert to 3D points for AR overlay
            // Grid worldX = ARKit X, grid worldY = ARKit Z, height = floor plane Y
            let floorY = self.meshProcessor.floorHeight
            let points3D = densePath.map { pt in
                simd_float3(pt.x, floorY + 0.02, pt.y)  // Slightly above floor
            }
            
            // Generate 3D ribbon mesh on background thread
            let pathMesh = self.createPathMesh(from: points3D, width: 0.05)
            
            DispatchQueue.main.async { [weak self] in
                guard let self = self else { return }
                self.plannedPath = path
                self.pathPoints3D = points3D
                self.gridMapView.plannedPath = path
                self.gridMapView.pathTargetPoint = (x: toX, y: toY)
                self.gridMapView.setNeedsDisplay()
                
                // Update 3D path mesh in AR scene
                self.pathAnchor?.removeFromParent()
                self.pathAnchor = nil
                
                if let mesh = pathMesh {
                    var material = UnlitMaterial()
                    material.color = .init(tint: UIColor(red: 0.0, green: 0.9, blue: 0.3, alpha: 0.9))
                    let entity = ModelEntity(mesh: mesh, materials: [material])
                    let anchor = AnchorEntity(world: .zero)
                    anchor.addChild(entity)
                    self.arView.scene.addAnchor(anchor)
                    self.pathAnchor = anchor
                }
                
                if path.isEmpty {
                    print("[Path] ❌ No path found to (\(String(format: "%.2f", toX)), \(String(format: "%.2f", toY)))")
                    self.navigateButton.isHidden = true
                    // Ack server command with failure
                    if reason == "server" {
                        TelemetryService.shared.logNavCommandAck(
                            cmd: "set_nav_target", success: false,
                            message: "No path found to (\(String(format: "%.2f", toX)), \(String(format: "%.2f", toY)))",
                            target: Vec2(x: toX, y: toY)
                        )
                    }
                } else {
                    print("[Path] ✅ Path found: \(path.count) waypoints, \(densePath.count) mesh points")
                    // Show navigate button
                    self.navigateButton.setTitle("Navigate", for: .normal)
                    self.navigateButton.backgroundColor = UIColor(red: 0.1, green: 0.6, blue: 1.0, alpha: 0.9)
                    self.navigateButton.isHidden = false
                    // Ack server command with success
                    if reason == "server" {
                        TelemetryService.shared.logNavCommandAck(
                            cmd: "set_nav_target", success: true,
                            message: "Path planned — \(path.count) waypoints to (\(String(format: "%.2f", toX)), \(String(format: "%.2f", toY)))",
                            target: Vec2(x: toX, y: toY),
                            waypointCount: path.count
                        )
                    }
                    
                    // Auto-start navigation if this is a route waypoint
                    if reason == "route_waypoint" && self.isRouteActive {
                        let target = (x: toX, y: toY)
                        self.pathNavigator.startNavigation(to: target, path: path)
                    }
                }
            }
        }
    }
    
    /// Rebuild the 3D AR path mesh from a set of world-coordinate waypoints.
    private func rebuildPathMesh(from path: [(x: Float, y: Float)]) {
        // Densify
        var densePath: [(x: Float, y: Float)] = []
        let spacing: Float = 0.05
        for i in 0..<path.count {
            let pt = path[i]
            densePath.append(pt)
            if i < path.count - 1 {
                let next = path[i + 1]
                let dx = next.x - pt.x
                let dy = next.y - pt.y
                let dist = sqrt(dx * dx + dy * dy)
                let steps = Int(dist / spacing)
                guard steps > 1 else { continue }
                for s in 1..<steps {
                    let t = Float(s) / Float(steps)
                    densePath.append((x: pt.x + dx * t, y: pt.y + dy * t))
                }
            }
        }
        let floorY = self.meshProcessor.floorHeight
        let points3D = densePath.map { pt in
            simd_float3(pt.x, floorY + 0.02, pt.y)
        }
        self.pathPoints3D = points3D
        
        pathAnchor?.removeFromParent()
        pathAnchor = nil
        
        if let mesh = createPathMesh(from: points3D, width: 0.05) {
            var material = UnlitMaterial()
            material.color = .init(tint: UIColor(red: 0.0, green: 0.9, blue: 0.3, alpha: 0.9))
            let entity = ModelEntity(mesh: mesh, materials: [material])
            let anchor = AnchorEntity(world: .zero)
            anchor.addChild(entity)
            arView.scene.addAnchor(anchor)
            pathAnchor = anchor
        }
    }
    
    // MARK: - 3D Path Mesh
    
    /// Smooth points using Catmull-Rom spline interpolation
    private func catmullRomSmooth(_ points: [simd_float3], subdivisions: Int = 4) -> [simd_float3] {
        guard points.count >= 2 else { return points }
        var result: [simd_float3] = []
        result.reserveCapacity(points.count * subdivisions)
        
        for i in 0..<points.count - 1 {
            let p0 = i > 0 ? points[i - 1] : points[i]
            let p1 = points[i]
            let p2 = points[i + 1]
            let p3 = i < points.count - 2 ? points[i + 2] : points[i + 1]
            
            for s in 0..<subdivisions {
                let t = Float(s) / Float(subdivisions)
                let tt = t * t
                let ttt = tt * t

                let twoP1 = p1 * 2.0
                let term1 = twoP1
                let term2 = (p2 - p0) * t
                let term3 = ((p0 * 2.0) - (p1 * 5.0) + (p2 * 4.0) - p3) * tt
                let term4 = (-p0 + (p1 * 3.0) - (p2 * 3.0) + p3) * ttt
                let sum = term1 + term2 + term3 + term4
                let v = sum * 0.5
                result.append(v)
            }
        }
        result.append(points.last!)
        return result
    }
    
    /// Create a ribbon mesh from 3D path points for AR overlay
    private func createPathMesh(from points: [simd_float3], width: Float = 0.05) -> MeshResource? {
        guard points.count >= 2 else { return nil }
        
        // Smooth the path with Catmull-Rom spline
        let smoothPoints = catmullRomSmooth(points, subdivisions: 4)
        guard smoothPoints.count >= 2 else { return nil }
        
        var vertices: [simd_float3] = []
        var normals: [simd_float3] = []
        var indices: [UInt32] = []
        
        let halfWidth = width / 2
        let up = simd_float3(0, 1, 0)
        
        vertices.reserveCapacity(smoothPoints.count * 2)
        normals.reserveCapacity(smoothPoints.count * 2)
        indices.reserveCapacity((smoothPoints.count - 1) * 12)
        
        var pairCount: Int = 0  // Number of vertex pairs added so far
        
        for (i, point) in smoothPoints.enumerated() {
            // Compute forward direction
            let rawForward: simd_float3
            if i == 0 {
                rawForward = smoothPoints[1] - smoothPoints[0]
            } else if i == smoothPoints.count - 1 {
                rawForward = smoothPoints[i] - smoothPoints[i - 1]
            } else {
                rawForward = smoothPoints[i + 1] - smoothPoints[i - 1]
            }
            
            let forwardLen = simd_length(rawForward)
            guard forwardLen > 0.0001 else { continue }  // Skip degenerate points
            let forward = rawForward / forwardLen
            
            // Right vector perpendicular to forward and up
            var right = simd_cross(forward, up)
            let rightLen = simd_length(right)
            if rightLen > 0.001 {
                right = right / rightLen
            } else {
                right = simd_float3(1, 0, 0)  // Fallback for vertical segments
            }
            
            // Left and right edge vertices
            vertices.append(point - right * halfWidth)
            vertices.append(point + right * halfWidth)
            normals.append(up)
            normals.append(up)
            
            // Add triangles connecting to previous pair
            if pairCount > 0 {
                let base = UInt32((pairCount - 1) * 2)
                // Front face (visible from above)
                indices.append(contentsOf: [base, base + 2, base + 1])
                indices.append(contentsOf: [base + 1, base + 2, base + 3])
                // Back face (visible from below)
                indices.append(contentsOf: [base, base + 1, base + 2])
                indices.append(contentsOf: [base + 1, base + 3, base + 2])
            }
            pairCount += 1
        }
        
        guard pairCount >= 2 else { return nil }
        
        var descriptor = MeshDescriptor(name: "pathLine")
        descriptor.positions = MeshBuffer(vertices)
        descriptor.normals = MeshBuffer(normals)
        descriptor.primitives = .triangles(indices)
        
        return try? MeshResource.generate(from: [descriptor])
    }
    
    // MARK: - UI Updates
    
    @objc private func updateFrame() {
        frameCount += 1
        
        // Extract only what we need from the frame in a minimal scope
        // to avoid holding AR frame references longer than necessary.
        // autoreleasepool ensures the ARFrame and its buffers are released
        // promptly, preventing the "retaining N ARFrames" warning.
        var cameraTransform: simd_float4x4 = matrix_identity_float4x4
        var meshCount: Int = 0
        var obstaclePoints3D: [simd_float3] = []
        var meshAnchorSnapshots: [MeshAnchorSnapshot]?

        let frameAvailable: Bool = autoreleasepool {
            guard let frame = arView.session.currentFrame else { return false }
            cameraTransform = frame.camera.transform
            let meshAnchors = frame.anchors.compactMap { $0 as? ARMeshAnchor }
            meshCount = meshAnchors.count
            // Feed floor height from mesh processor so depth detection can filter floor/ceiling
            obstacleDetector.floorHeight = meshProcessor.floorHeight
            obstacleDetector.floorHeightEstimated = meshProcessor.floorInitialized
            // Update depth-based proximity detection (catches dynamic objects like hands)
            obstacleDetector.updateDepth(frame: frame)
            // Always-on person detection with ReID (~10-15 Hz, every 2 frames)
            if frameCount % 2 == 0 && !personDetectionInFlight && personTracker.state == .idle {
                personDetectionInFlight = true
                let pixelBuffer = frame.capturedImage
                let capturedFrame = frame
                let tracker = self.personTracker
                let knownEmbeddings = self.knownPersonEmbeddings
                let threshold = self.alwaysOnReidThreshold
                DispatchQueue.global(qos: .userInitiated).async { [weak self] in
                    let request = VNDetectHumanRectanglesRequest()
                    request.revision = VNDetectHumanRectanglesRequestRevision2
                    let handPoseRequest = VNDetectHumanHandPoseRequest()
                    handPoseRequest.maximumHandCount = 6
                    let handler = VNImageRequestHandler(cvPixelBuffer: pixelBuffer, orientation: .right, options: [:])
                    try? handler.perform([request, handPoseRequest])
                    let observations = (request.results as? [VNHumanObservation]) ?? []
                    let handResults = handPoseRequest.results ?? []
                    
                    // Extract hand joint data for overlay (before gesture classification)
                    let handJoints: [HandJointData] = handResults.compactMap {
                        HandJointOverlayView.extractJoints(from: $0)
                    }
                    
                    // Generate embeddings and match against known people
                    var boxes: [PersonBoxInfo] = []
                    var updatedEmbeddings = knownEmbeddings
                    var matchedKnownIDs = Set<UUID>()
                    
                    for obs in observations {
                        let embedding = tracker.generateEmbedding(pixelBuffer: pixelBuffer, boundingBox: obs.boundingBox)
                        
                        // Try to match against known embeddings
                        var bestMatch: (id: UUID, sim: Float)?
                        if let embed = embedding {
                            for (knownID, knownEmbed) in knownEmbeddings where !matchedKnownIDs.contains(knownID) {
                                let sim = tracker.cosineSimilarity(embed, knownEmbed)
                                if sim > threshold && (bestMatch == nil || sim > bestMatch!.sim) {
                                    bestMatch = (knownID, sim)
                                }
                            }
                        }
                        
                        let personID: UUID
                        if let match = bestMatch {
                            personID = match.id
                            matchedKnownIDs.insert(match.id)
                            // Update embedding with latest
                            if let embed = embedding {
                                updatedEmbeddings[personID] = embed
                            }
                        } else {
                            // New person
                            personID = UUID()
                            if let embed = embedding {
                                updatedEmbeddings[personID] = embed
                            }
                        }
                        
                        boxes.append(PersonBoxInfo(
                            id: personID,
                            boundingBox: obs.boundingBox,
                            isActive: false,
                            isGesturing: false,
                            label: String(personID.uuidString.prefix(4)),
                            worldPosition: tracker.projectToWorld(boundingBox: obs.boundingBox, frame: capturedFrame)
                        ))
                    }
                    
                    // Detect gestures and match to people
                    var peaceHands: [(wrist: CGPoint, hand: VNHumanHandPoseObservation)] = []
                    var fistHands: [CGPoint] = []
                    var pointerHands: [(wrist: CGPoint, wristX: CGFloat)] = []
                    
                    for hand in handResults {
                        if let wrist = tracker.detectPeaceSign(hand: hand) {
                            peaceHands.append((wrist, hand))
                        } else if let wrist = tracker.detectClosedHand(hand: hand) {
                            fistHands.append(wrist)
                        }
                        if let info = tracker.detectIndexPointer(hand: hand) {
                            pointerHands.append(info)
                        }
                    }
                    
                    // Mark peace-sign gesturing people (hand must be raised — upper 40% of bbox)
                    var gesturedIDs = Set<UUID>()
                    for (wrist, _) in peaceHands {
                        for i in boxes.indices {
                            let expandedBBox = boxes[i].boundingBox.insetBy(dx: -0.08, dy: -0.08)
                            guard expandedBBox.contains(wrist) else { continue }
                            let bbox = boxes[i].boundingBox
                            let upperThreshold = bbox.origin.y + bbox.size.height * 0.2
                            guard wrist.y >= upperThreshold else { continue }
                            gesturedIDs.insert(boxes[i].id)
                            boxes[i] = PersonBoxInfo(
                                id: boxes[i].id,
                                boundingBox: boxes[i].boundingBox,
                                isActive: false,
                                isGesturing: true,
                                label: boxes[i].label,
                                worldPosition: boxes[i].worldPosition
                            )
                            break
                        }
                    }
                    
                    // Collect fist person IDs (hand must be raised — upper 40% of bbox)
                    var fistIDs = Set<UUID>()
                    for wrist in fistHands {
                        for box in boxes {
                            let expandedBBox = box.boundingBox.insetBy(dx: -0.08, dy: -0.08)
                            guard expandedBBox.contains(wrist) else { continue }
                            let upperThreshold = box.boundingBox.origin.y + box.boundingBox.size.height * 0.2
                            guard wrist.y >= upperThreshold else { continue }
                            fistIDs.insert(box.id)
                            break
                        }
                    }
                    
                    // Prune embeddings for people not seen (keep for a few cycles via main thread)
                    let seenIDs = Set(boxes.map { $0.id })
                    
                    DispatchQueue.main.async {
                        guard let self = self else { return }
                        self.alwaysOnPersonBoxes = boxes
                        self.knownPersonEmbeddings = updatedEmbeddings.filter { seenIDs.contains($0.key) || knownEmbeddings.keys.contains($0.key) }
                        self.personDetectionInFlight = false
                        
                        // Update hand joint overlay
                        self.handJointOverlay.hands = handJoints
                        self.handJointOverlay.setNeedsDisplay()
                        
                        let now = Date()
                        let timeout = self.alwaysOnGestureTimeout
                        
                        // --- Activation gesture state machine: peace → fist → peace ---
                        for personID in gesturedIDs {
                            let phase = self.alwaysOnActivatePhase[personID] ?? .idle
                            let phaseTime = self.alwaysOnActivateTimestamp[personID] ?? .distantPast
                            let elapsed = now.timeIntervalSince(phaseTime)
                            
                            switch phase {
                            case .idle:
                                self.alwaysOnActivatePhase[personID] = .peaceOpen1
                                self.alwaysOnActivateTimestamp[personID] = now
                            case .peaceOpen1:
                                break // still holding peace — wait for fist
                            case .peaceClosed1:
                                if elapsed < timeout {
                                    // Second peace sign detected — activate!
                                    self.alwaysOnActivatePhase.removeAll()
                                    self.alwaysOnActivateTimestamp.removeAll()
                                    if let embed = self.knownPersonEmbeddings[personID],
                                       let box = boxes.first(where: { $0.id == personID }) {
                                        self.startFollowModeForPerson(id: personID, embedding: embed, boundingBox: box.boundingBox, worldPosition: box.worldPosition)
                                    }
                                    return
                                } else {
                                    // Timeout — restart
                                    self.alwaysOnActivatePhase[personID] = .peaceOpen1
                                    self.alwaysOnActivateTimestamp[personID] = now
                                }
                            case .peaceOpen2:
                                break
                            }
                        }
                        
                        // Transition peaceOpen1 → peaceClosed1 for fist detections
                        for personID in fistIDs {
                            let phase = self.alwaysOnActivatePhase[personID] ?? .idle
                            let phaseTime = self.alwaysOnActivateTimestamp[personID] ?? .distantPast
                            let elapsed = now.timeIntervalSince(phaseTime)
                            
                            if phase == .peaceOpen1 && elapsed < timeout {
                                self.alwaysOnActivatePhase[personID] = .peaceClosed1
                                self.alwaysOnActivateTimestamp[personID] = now
                            }
                        }
                        
                        // Prune stale activate phases
                        for (pid, ts) in self.alwaysOnActivateTimestamp {
                            if now.timeIntervalSince(ts) > timeout {
                                self.alwaysOnActivatePhase.removeValue(forKey: pid)
                                self.alwaysOnActivateTimestamp.removeValue(forKey: pid)
                            }
                        }
                        
                        // --- Cancel gesture state machine: finger wag (one full sweep) ---
                        if let (_, tipX) = pointerHands.first {
                            let elapsed = now.timeIntervalSince(self.alwaysOnCancelTimestamp)
                            if elapsed > timeout {
                                self.alwaysOnCancelPhase = .idle
                                self.alwaysOnCancelLastWristX = nil
                            }
                            
                            if let lastX = self.alwaysOnCancelLastWristX {
                                let delta = tipX - lastX
                                let minDelta = self.alwaysOnWagMinDelta
                                
                                switch self.alwaysOnCancelPhase {
                                case .idle:
                                    if abs(delta) > minDelta {
                                        self.alwaysOnCancelPhase = delta > 0 ? .movedRight1 : .movedLeft1
                                        self.alwaysOnCancelTimestamp = now
                                        self.alwaysOnCancelLastWristX = tipX
                                    }
                                case .movedRight1:
                                    if delta < -minDelta {
                                        self.alwaysOnCancelPhase = .idle
                                        self.alwaysOnCancelLastWristX = nil
                                        self.stopFollowMode()
                                    } else if delta > minDelta {
                                        self.alwaysOnCancelLastWristX = tipX
                                    }
                                case .movedLeft1:
                                    if delta > minDelta {
                                        self.alwaysOnCancelPhase = .idle
                                        self.alwaysOnCancelLastWristX = nil
                                        self.stopFollowMode()
                                    } else if delta < -minDelta {
                                        self.alwaysOnCancelLastWristX = tipX
                                    }
                                case .movedRight2, .movedLeft2:
                                    break
                                }
                            } else {
                                self.alwaysOnCancelLastWristX = tipX
                                self.alwaysOnCancelTimestamp = now
                            }
                        }
                    }
                }
            }
            // Cancel gesture detection during follow mode (~10-15 Hz, hand-only, lightweight)
            if frameCount % 2 == 0 && personTracker.state != .idle {
                let pixelBuffer = frame.capturedImage
                let tracker = self.personTracker
                DispatchQueue.global(qos: .userInitiated).async { [weak self] in
                    let handPoseRequest = VNDetectHumanHandPoseRequest()
                    handPoseRequest.maximumHandCount = 2
                    let handler = VNImageRequestHandler(cvPixelBuffer: pixelBuffer, orientation: .right, options: [:])
                    try? handler.perform([handPoseRequest])
                    let handResults = handPoseRequest.results ?? []
                    
                    // Extract hand joint data for overlay
                    let handJoints: [HandJointData] = handResults.compactMap {
                        HandJointOverlayView.extractJoints(from: $0)
                    }
                    
                    var pointerHands: [(wrist: CGPoint, wristX: CGFloat)] = []
                    for hand in handResults {
                        if let info = tracker.detectIndexPointer(hand: hand) {
                            pointerHands.append(info)
                        }
                    }
                    
                    guard !pointerHands.isEmpty || !handJoints.isEmpty else { return }
                    
                    DispatchQueue.main.async {
                        guard let self = self else { return }
                        
                        // Update hand joint overlay during follow mode
                        self.handJointOverlay.hands = handJoints
                        self.handJointOverlay.setNeedsDisplay()
                        
                        guard !pointerHands.isEmpty else { return }
                        let now = Date()
                        let timeout = self.alwaysOnGestureTimeout
                        
                        if let (_, tipX) = pointerHands.first {
                            let elapsed = now.timeIntervalSince(self.alwaysOnCancelTimestamp)
                            if elapsed > timeout {
                                self.alwaysOnCancelPhase = .idle
                                self.alwaysOnCancelLastWristX = nil
                            }
                            
                            if let lastX = self.alwaysOnCancelLastWristX {
                                let delta = tipX - lastX
                                let minDelta = self.alwaysOnWagMinDelta
                                
                                switch self.alwaysOnCancelPhase {
                                case .idle:
                                    if abs(delta) > minDelta {
                                        self.alwaysOnCancelPhase = delta > 0 ? .movedRight1 : .movedLeft1
                                        self.alwaysOnCancelTimestamp = now
                                        self.alwaysOnCancelLastWristX = tipX
                                    }
                                case .movedRight1:
                                    if delta < -minDelta {
                                        self.alwaysOnCancelPhase = .idle
                                        self.alwaysOnCancelLastWristX = nil
                                        self.stopFollowMode()
                                    } else if delta > minDelta {
                                        self.alwaysOnCancelLastWristX = tipX
                                    }
                                case .movedLeft1:
                                    if delta > minDelta {
                                        self.alwaysOnCancelPhase = .idle
                                        self.alwaysOnCancelLastWristX = nil
                                        self.stopFollowMode()
                                    } else if delta < -minDelta {
                                        self.alwaysOnCancelLastWristX = tipX
                                    }
                                case .movedRight2, .movedLeft2:
                                    break
                                }
                            } else {
                                self.alwaysOnCancelLastWristX = tipX
                                self.alwaysOnCancelTimestamp = now
                            }
                        }
                    }
                }
            }
            // Update person tracker for follow mode (every frame)
            if personTracker.state != .idle {
                personTracker.update(frame: frame)
                // Emit person tracking telemetry at ~2 Hz (every 10 frames at ~20fps)
                if frameCount % 10 == 0 {
                    let event = personTracker.state == .scanning ? "scan_update" : "tracking_update"
                    TelemetryService.shared.logPersonTracking(
                        event: event,
                        message: "\(personTracker.detectedPeople.count) people, state=\(personTracker.state.rawValue)",
                        occupancyGrid: self.occupancyGrid,
                        cameraTransform: cameraTransform
                    )
                }
            }
            // Collect 3D obstacle points for white dot overlay every 2 frames
            if frameCount % 2 == 0 {
                obstaclePoints3D = obstacleDetector.collectObstaclePoints3D(frame: frame)
            }
            // Extract mesh anchor geometry for telemetry at ~1 Hz
            if frameCount % 30 == 0 {
                let gen = meshProcessor.currentGeneration()
                meshAnchorSnapshots = meshAnchors.compactMap { MeshProcessor.extractAnchorSnapshot(from: $0, generation: gen) }
            }
            // frame is released here when autoreleasepool drains
            return true
        }
        guard frameAvailable else { return }
        
        // Update device position every frame
        occupancyGrid.updateDevicePosition(transform: cameraTransform)
        obstacleDetector.cameraTransform = cameraTransform
        
        // Check for obstacles every frame (~20-30Hz)
        obstacleDetector.update()
        
        // Update obstacle point overlay every 2 frames
        if frameCount % 2 == 0 {
            let camPos = simd_float3(cameraTransform.columns.3.x,
                                     cameraTransform.columns.3.y,
                                     cameraTransform.columns.3.z)
            var projected: [ProjectedObstaclePoint] = []
            projected.reserveCapacity(obstaclePoints3D.count)
            for pt in obstaclePoints3D {
                if let sp = arView.project(pt) {
                    let dist = simd_distance(pt, camPos)
                    projected.append(ProjectedObstaclePoint(position: sp, distance: dist))
                }
            }
            obstaclePointOverlay.projectedPoints = projected
            obstaclePointOverlay.setNeedsDisplay()

            // Update person bounding box overlay
            if personTracker.state != .idle {
                // Use PersonTracker's rich data (stable IDs, gestures, active person)
                let activeID = personTracker.activePersonID
                let boxes: [PersonBoxInfo] = personTracker.detectedPeople.map { p in
                    PersonBoxInfo(
                        id: p.id,
                        boundingBox: p.boundingBox,
                        isActive: p.id == activeID,
                        isGesturing: p.isGesturing,
                        label: String(p.id.uuidString.prefix(4)),
                        worldPosition: p.worldPosition
                    )
                }
                // If tracking a single person, include the tracked bbox too
                if let activeID = activeID,
                   let trackedBox = personTracker.trackedBoundingBox,
                   !boxes.contains(where: { $0.id == activeID }) {
                    var allBoxes = boxes
                    allBoxes.append(PersonBoxInfo(
                        id: activeID,
                        boundingBox: trackedBox,
                        isActive: true,
                        isGesturing: false,
                        label: String(activeID.uuidString.prefix(4)),
                        worldPosition: personTracker.trackedWorldPosition
                    ))
                    personBoundingBoxOverlay.people = allBoxes
                } else {
                    personBoundingBoxOverlay.people = boxes
                }
                personBoundingBoxOverlay.setNeedsDisplay()
                // Feed positions to 2D grid
                gridMapView.personPositions = personBoundingBoxOverlay.people
                gridMapView.setNeedsDisplay()
            } else if !alwaysOnPersonBoxes.isEmpty {
                // Idle mode — use ReID-matched results (stable IDs & colors)
                personBoundingBoxOverlay.people = alwaysOnPersonBoxes
                personBoundingBoxOverlay.setNeedsDisplay()
                gridMapView.personPositions = alwaysOnPersonBoxes
                gridMapView.setNeedsDisplay()
            } else if !personBoundingBoxOverlay.people.isEmpty {
                personBoundingBoxOverlay.people = []
                personBoundingBoxOverlay.setNeedsDisplay()
                handJointOverlay.hands = []
                handJointOverlay.setNeedsDisplay()
                gridMapView.personPositions = []
                gridMapView.setNeedsDisplay()
            }
        }
        
        // Update labels every 3 frames (~10Hz at 30fps)
        if frameCount % 3 == 0 {
            let pos = occupancyGrid.devicePosition
            positionLabel.text = String(format: "X: %.2f\nY: %.2f\nZ: %.2f\nθ: %.0f°",
                                        pos.x, pos.y, pos.z, pos.heading * 180 / .pi)
            
            statusLabel.text = "📡 Meshes: \(meshCount)\n🔲 Occupied: \(occupancyGrid.occupiedCount)\n⬜ Free: \(occupancyGrid.freeCount)"
            statusLabel.textColor = .green
            
            // Telemetry: emit nav frame at ~10 Hz
            TelemetryService.shared.lastCameraTransform = cameraTransform
            TelemetryService.shared.tick(occupancyGrid: occupancyGrid, cameraTransform: cameraTransform)
        }
        
        // Telemetry: emit mesh anchor geometry at ~1 Hz
        if let snapshots = meshAnchorSnapshots {
            TelemetryService.shared.emitMeshAnchors(snapshots, occupancyGrid: occupancyGrid, cameraTransform: cameraTransform)
        }
        
        // Redraw grid every 6 frames (~5Hz at 30fps)
        if frameCount % 6 == 0 {
            gridMapView.setNeedsDisplay()
        }
        
        // Navigation re-planning: if navigation is active and a re-plan is due,
        // re-run pathfinding from current position to the original target.
        // Guard against concurrent dispatches to avoid retaining multiple ARFrames.
        if !isReplanning, pathNavigator.needsReplan, let target = pathNavigator.targetPoint {
            isReplanning = true
            let pos = occupancyGrid.devicePosition
            // Snap target to nearest free point only if the map has updated
            // and the original target is now inside an obstacle (occupied cell).
            // Leave unknown-territory targets as-is for greedy pathfinder.
            var adjustedTarget = target
            if occupancyGrid.getStateAtWorld(x: target.x, y: target.y) == .occupied {
                if let freePoint = occupancyGrid.nearestFreeWorldPoint(x: target.x, y: target.y) {
                    adjustedTarget = freePoint
                }
            }
            // Capture camera transform now — do NOT access currentFrame from background thread
            let capturedTransform = cameraTransform
            DispatchQueue.global(qos: .userInitiated).async { [weak self] in
                guard let self = self else { return }
                let replanStart = CFAbsoluteTimeGetCurrent()
                
                // Run both pathfinders and pick the better one.
                // During re-planning (robot already moving), strongly prefer
                // the strict (free-cells-only) path to stay on known floor.
                // Only use greedy if strict finds nothing.
                let strictPath = self.occupancyGrid.findPath(fromX: pos.x, fromY: pos.y, toX: adjustedTarget.x, toY: adjustedTarget.y)
                
                let path: [(x: Float, y: Float)]
                let algorithm: String
                if !strictPath.isEmpty {
                    path = strictPath
                    algorithm = "astar"
                } else {
                    let greedyPath = self.occupancyGrid.findPathGreedy(fromX: pos.x, fromY: pos.y, toX: adjustedTarget.x, toY: adjustedTarget.y)
                    if !greedyPath.isEmpty {
                        path = greedyPath
                        algorithm = "greedy_fallback"
                    } else {
                        path = []
                        algorithm = "none"
                    }
                }
                let replanDurationMs = Float((CFAbsoluteTimeGetCurrent() - replanStart) * 1000)
                
                if !path.isEmpty {
                    // Log replan to telemetry
                    TelemetryService.shared.logRoutePlanned(
                        target: (x: adjustedTarget.x, y: adjustedTarget.y),
                        origin: (x: pos.x, y: pos.y),
                        waypoints: path,
                        algorithm: algorithm,
                        reason: "replan_periodic",
                        planDurationMs: replanDurationMs,
                        occupancyGrid: self.occupancyGrid,
                        cameraTransform: capturedTransform
                    )
                }
                
                DispatchQueue.main.async {
                    self.isReplanning = false
                    guard !path.isEmpty else {
                        // Replan failed (no path found) — clear the awaiting
                        // flag so obstacle checks resume and the robot can
                        // try escape maneuvers instead of waiting forever.
                        self.pathNavigator.clearAwaitingReplan()
                        return
                    }
                    self.plannedPath = path
                    self.gridMapView.plannedPath = path
                    self.gridMapView.setNeedsDisplay()
                    self.rebuildPathMesh(from: path)
                    self.pathNavigator.updatePath(path)
                }
            }
        }
        
        // Follow-mode re-planning: continuously update path as person moves
        if !isFollowReplanning, pathNavigator.isFollowMode, pathNavigator.needsFollowReplan,
           let personPos = personTracker.trackedWorldPosition,
           personTracker.state == .tracking {
            planFollowPath(toX: personPos.x, toY: personPos.y)
        }
    }
    

}

// MARK: - ARSessionDelegate

extension LiDARViewController: ARSessionDelegate {
    
    func session(_ session: ARSession, didAdd anchors: [ARAnchor]) {
        processAnchors(anchors)
    }
    
    func session(_ session: ARSession, didUpdate anchors: [ARAnchor]) {
        processAnchors(anchors)
    }
    
    private func processAnchors(_ anchors: [ARAnchor]) {
        // Skip processing if we're in the middle of a reset
        guard !isResetting else { return }
        
        for anchor in anchors {
            guard let meshAnchor = anchor as? ARMeshAnchor else { continue }
            
            // Skip anchors that existed before the last grid reset —
            // their world-space data is stale.
            guard !preResetAnchorIds.contains(anchor.identifier) else { continue }
            
            // Check if geometry changed
            let geometry = meshAnchor.geometry
            let version = geometry.vertices.count + geometry.faces.count
            
            if let lastVersion = processedMeshVersions[anchor.identifier], lastVersion == version {
                continue
            }
            processedMeshVersions[anchor.identifier] = version
            
            meshUpdateCounter += 1
            let currentUpdateId = meshUpdateCounter
            
            // IMPORTANT: Extract data synchronously on main thread to avoid retaining ARFrame
            let meshData = meshProcessor.extractMeshData(from: meshAnchor, updateId: currentUpdateId)
            
            // Process extracted data asynchronously (no ARFrame reference retained)
            DispatchQueue.global(qos: .utility).async { [weak self] in
                self?.meshProcessor.processExtractedMesh(meshData)
            }
        }
    }
    
    func session(_ session: ARSession, didRemove anchors: [ARAnchor]) {
        for anchor in anchors {
            processedMeshVersions.removeValue(forKey: anchor.identifier)
            preResetAnchorIds.remove(anchor.identifier)
            // Clear grid cells that belonged to this anchor
            if anchor is ARMeshAnchor {
                occupancyGrid.removeAnchorData(anchor.identifier)
            }
        }
    }
    
    func session(_ session: ARSession, cameraDidChangeTrackingState camera: ARCamera) {
        DispatchQueue.main.async { [weak self] in
            switch camera.trackingState {
            case .notAvailable:
                self?.statusLabel.text = "❌ Tracking unavailable"
                self?.statusLabel.textColor = .red
            case .limited(let reason):
                var reasonText = ""
                switch reason {
                case .initializing: reasonText = "Initializing"
                case .excessiveMotion: reasonText = "Too much motion"
                case .insufficientFeatures: reasonText = "Low features"
                case .relocalizing:
                    reasonText = "Relocalizing"
                    // World origin has changed — clear the grid to avoid stale data
                    self?.isResetting = true
                    self?.resetGridState()
                    print("[AR] 🔄 Relocalizing — grid reset")
                @unknown default: reasonText = "Unknown"
                }
                self?.statusLabel.text = "⚠️ Limited: \(reasonText)"
                self?.statusLabel.textColor = .yellow
            case .normal:
                self?.isResetting = false  // Safe to process meshes now
                self?.statusLabel.text = "✅ Tracking normal"
                self?.statusLabel.textColor = .green
            }
        }
    }
    
    func session(_ session: ARSession, didFailWithError error: Error) {
        DispatchQueue.main.async { [weak self] in
            self?.statusLabel.text = "❌ Error: \(error.localizedDescription)"
            self?.statusLabel.textColor = .red
        }
    }
}

