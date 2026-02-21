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
    private var obstaclePointOverlay: ObstaclePointOverlayView!
    
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
    
    /// Planned path waypoints in world coordinates (grid X/Y)
    private var plannedPath: [(x: Float, y: Float)] = []
    
    /// Planned path waypoints as 3D points for AR overlay (ARKit world space)
    private var pathPoints3D: [simd_float3] = []
    
    /// Anchor entity for the 3D path mesh in the AR scene
    private var pathAnchor: AnchorEntity?
    
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
        
        // Handle tap for pathfinding
        gridMapView.onTapWorldPosition = { [weak self] worldX, worldY in
            self?.planPath(toX: worldX, toY: worldY)
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
        // Invalidate any in-flight mesh processing BEFORE clearing
        // This increments generation so stale mesh data will be discarded
        meshProcessor.reset()
        
        // Clear the occupancy grid (resets originOffset and all data)
        occupancyGrid.clear()
        
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
    
    // MARK: - Pathfinding
    
    /// Plan a path from the current device position to the tapped world position.
    /// Runs A* on a background thread, then updates the grid and AR overlays on main.
    private func planPath(toX: Float, toY: Float) {
        let devicePos = occupancyGrid.devicePosition
        let startX = devicePos.x
        let startY = devicePos.y
        
        // Set target point immediately for visual feedback
        gridMapView.pathTargetPoint = (x: toX, y: toY)
        gridMapView.plannedPath = []
        gridMapView.setNeedsDisplay()
        
        // Run pathfinding off the main thread
        DispatchQueue.global(qos: .userInitiated).async { [weak self] in
            guard let self = self else { return }
            
            let path = self.occupancyGrid.findPath(fromX: startX, fromY: startY, toX: toX, toY: toY)
            
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
                } else {
                    print("[Path] ✅ Path found: \(path.count) waypoints, \(densePath.count) mesh points")
                }
            }
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
        // to avoid holding AR frame references longer than necessary
        let cameraTransform: simd_float4x4
        let meshCount: Int
        var obstaclePoints3D: [simd_float3] = []
        
        if let frame = arView.session.currentFrame {
            cameraTransform = frame.camera.transform
            meshCount = frame.anchors.lazy.compactMap { $0 as? ARMeshAnchor }.count
            // Feed floor height from mesh processor so depth detection can filter floor/ceiling
            obstacleDetector.floorHeight = meshProcessor.floorHeight
            obstacleDetector.floorHeightEstimated = meshProcessor.floorInitialized
            // Update depth-based proximity detection (catches dynamic objects like hands)
            obstacleDetector.updateDepth(frame: frame)
            // Collect 3D obstacle points for white dot overlay every 2 frames
            if frameCount % 2 == 0 {
                obstaclePoints3D = obstacleDetector.collectObstaclePoints3D(frame: frame)
            }
            // frame goes out of scope here, releasing the reference
        } else {
            return
        }
        
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

