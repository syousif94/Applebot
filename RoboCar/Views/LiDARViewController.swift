//
//  LiDARViewController.swift
//  RoboCar
//
//  Created by Sammy Yousif on 1/3/26.
//

import UIKit
import ARKit
import RealityKit

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
    private var obstacleBannerView: UIView!
    private var obstacleBannerContent: ObstacleBannerContentView!
    
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
    
    /// Flag to pause mesh processing during reset
    private var isResetting = false
    
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
        setupObstacleBanner()
        setupMeshProcessor()
        setupObstacleDetector()
        
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
        // Invalidate any in-flight mesh processing BEFORE clearing
        // This increments generation so stale mesh data will be discarded
        meshProcessor.reset()
        
        // Clear the occupancy grid (resets originOffset and all data)
        occupancyGrid.clear()
        
        // Reset mesh tracking
        processedMeshVersions.removeAll()
        
        // Reset the grid view's initial heading
        gridMapView.resetInitialHeading()
        
        // Force redraw
        gridMapView.setNeedsDisplay()
    }
    
    private func setupObstacleBanner() {
        // Container
        obstacleBannerView = UIView()
        obstacleBannerView.translatesAutoresizingMaskIntoConstraints = false
        obstacleBannerView.backgroundColor = UIColor(red: 0.9, green: 0.15, blue: 0.1, alpha: 0.92)
        obstacleBannerView.layer.cornerRadius = 16
        obstacleBannerView.layer.shadowColor = UIColor.black.cgColor
        obstacleBannerView.layer.shadowOffset = CGSize(width: 0, height: 2)
        obstacleBannerView.layer.shadowRadius = 6
        obstacleBannerView.layer.shadowOpacity = 0.4
        obstacleBannerView.alpha = 0
        obstacleBannerView.isUserInteractionEnabled = false
        
        view.addSubview(obstacleBannerView)
        
        // Custom-drawn content with chevrons
        obstacleBannerContent = ObstacleBannerContentView()
        obstacleBannerContent.translatesAutoresizingMaskIntoConstraints = false
        obstacleBannerContent.backgroundColor = .clear
        obstacleBannerView.addSubview(obstacleBannerContent)
        
        NSLayoutConstraint.activate([
            obstacleBannerView.centerXAnchor.constraint(equalTo: view.centerXAnchor),
            obstacleBannerView.topAnchor.constraint(equalTo: arView.bottomAnchor, constant: 6),
            obstacleBannerView.leadingAnchor.constraint(greaterThanOrEqualTo: view.leadingAnchor, constant: 8),
            obstacleBannerView.trailingAnchor.constraint(lessThanOrEqualTo: view.trailingAnchor, constant: -8),
            obstacleBannerView.heightAnchor.constraint(equalToConstant: 32),
            
            obstacleBannerContent.leadingAnchor.constraint(equalTo: obstacleBannerView.leadingAnchor),
            obstacleBannerContent.trailingAnchor.constraint(equalTo: obstacleBannerView.trailingAnchor),
            obstacleBannerContent.topAnchor.constraint(equalTo: obstacleBannerView.topAnchor),
            obstacleBannerContent.bottomAnchor.constraint(equalTo: obstacleBannerView.bottomAnchor),
        ])
    }
    
    private func setupMeshProcessor() {
        meshProcessor = MeshProcessor(occupancyGrid: occupancyGrid)
    }
    
    private func setupObstacleDetector() {
        obstacleDetector.occupancyGrid = occupancyGrid
        MotorCalibrator.shared.occupancyGrid = occupancyGrid
        gridMapView.obstacleDetector = obstacleDetector
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
    
    // MARK: - UI Updates
    
    @objc private func updateFrame() {
        frameCount += 1
        
        // Extract only what we need from the frame in a minimal scope
        // to avoid holding AR frame references longer than necessary
        let cameraTransform: simd_float4x4
        let meshCount: Int
        
        if let frame = arView.session.currentFrame {
            cameraTransform = frame.camera.transform
            meshCount = frame.anchors.lazy.compactMap { $0 as? ARMeshAnchor }.count
            // Feed floor height from mesh processor so depth detection can filter floor/ceiling
            obstacleDetector.floorHeight = meshProcessor.floorHeight
            obstacleDetector.floorHeightEstimated = meshProcessor.floorInitialized
            // Update depth-based proximity detection (catches dynamic objects like hands)
            obstacleDetector.updateDepth(frame: frame)
            // frame goes out of scope here, releasing the reference
        } else {
            return
        }
        
        // Update device position every frame
        occupancyGrid.updateDevicePosition(transform: cameraTransform)
        obstacleDetector.cameraTransform = cameraTransform
        
        // Check for obstacles every frame (~20-30Hz)
        obstacleDetector.update()
        
        // Update obstacle banner every 3 frames
        if frameCount % 3 == 0 {
            updateObstacleBanner()
        }
        
        // Update labels every 3 frames (~10Hz at 30fps)
        if frameCount % 3 == 0 {
            let pos = occupancyGrid.devicePosition
            positionLabel.text = String(format: "X: %.2f\nY: %.2f\nZ: %.2f\nθ: %.0f°",
                                        pos.x, pos.y, pos.z, pos.heading * 180 / .pi)
            
            statusLabel.text = "📡 Meshes: \(meshCount)\n🔲 Occupied: \(occupancyGrid.occupiedCount)\n⬜ Free: \(occupancyGrid.freeCount)"
            statusLabel.textColor = .green
        }
        
        // Redraw grid every 6 frames (~5Hz at 30fps)
        if frameCount % 6 == 0 {
            gridMapView.setNeedsDisplay()
        }
    }
    
    // MARK: - Obstacle Banner
    
    private var bannerIsVisible = false
    
    private func updateObstacleBanner() {
        let detector = obstacleDetector
        
        if detector.obstacleDetected, let distance = detector.nearestObstacleDistance {
            // Feed obstacles to the custom content view
            obstacleBannerContent.obstacles = Array(detector.nearbyObstacles.prefix(3))
            obstacleBannerContent.setNeedsDisplay()
            
            // Tint: interpolate from yellow (far) to red (very close)
            let urgency = max(0, min(1, 1.0 - CGFloat(distance) / CGFloat(detector.stopRadius)))
            let r: CGFloat = 0.9
            let g: CGFloat = 0.6 * (1.0 - urgency)  // yellow → red
            let b: CGFloat = 0.05
            obstacleBannerView.backgroundColor = UIColor(red: r, green: g, blue: b, alpha: 0.92)
            
            if !bannerIsVisible {
                bannerIsVisible = true
                UIView.animate(withDuration: 0.2) {
                    self.obstacleBannerView.alpha = 1
                }
            }
        } else {
            if bannerIsVisible {
                bannerIsVisible = false
                UIView.animate(withDuration: 0.3) {
                    self.obstacleBannerView.alpha = 0
                }
            }
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
            
            // Check if geometry changed
            let geometry = meshAnchor.geometry
            let version = geometry.vertices.count + geometry.faces.count
            
            if let lastVersion = processedMeshVersions[anchor.identifier], lastVersion == version {
                continue
            }
            processedMeshVersions[anchor.identifier] = version
            
            // IMPORTANT: Extract data synchronously on main thread to avoid retaining ARFrame
            let meshData = meshProcessor.extractMeshData(from: meshAnchor)
            
            // Process extracted data asynchronously (no ARFrame reference retained)
            DispatchQueue.global(qos: .utility).async { [weak self] in
                self?.meshProcessor.processExtractedMesh(meshData)
            }
        }
    }
    
    func session(_ session: ARSession, didRemove anchors: [ARAnchor]) {
        for anchor in anchors {
            processedMeshVersions.removeValue(forKey: anchor.identifier)
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

