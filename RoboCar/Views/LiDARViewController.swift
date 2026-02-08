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
    
    // MARK: - State
    
    /// The global occupancy grid storing all obstacles
    let occupancyGrid = OccupancyGrid(cellSize: 0.05, gridRadius: 500)  // 50m x 50m grid with 5cm cells
    
    /// Mesh processor for converting AR mesh to grid data
    private var meshProcessor: MeshProcessor!
    
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
        setupMeshProcessor()
        
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
            positionLabel.topAnchor.constraint(equalTo: view.safeAreaLayoutGuide.topAnchor, constant: 8),
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
        
        // Invalidate any in-flight mesh processing BEFORE clearing
        // This increments generation so stale mesh data will be discarded
        meshProcessor.reset()
        
        // Clear the occupancy grid (resets originOffset and all data)
        occupancyGrid.clear()
        
        // Reset mesh tracking
        processedMeshVersions.removeAll()
        
        // Reset the grid view's initial heading
        gridMapView.resetInitialHeading()
        
        // Restart AR session to reset coordinate origin
        startARSession()
        
        // Force redraw
        gridMapView.setNeedsDisplay()
        
        statusLabel.text = "🔄 Map reset"
        statusLabel.textColor = .yellow
    }
    
    private func setupMeshProcessor() {
        meshProcessor = MeshProcessor(occupancyGrid: occupancyGrid)
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
            // frame goes out of scope here, releasing the reference
        } else {
            return
        }
        
        // Update device position every frame
        occupancyGrid.updateDevicePosition(transform: cameraTransform)
        
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
                case .relocalizing: reasonText = "Relocalizing"
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

