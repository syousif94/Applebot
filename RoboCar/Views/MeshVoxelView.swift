//
//  MeshVoxelView.swift
//  RoboCar

import UIKit
import SceneKit
import simd

final class MeshVoxelView: UIView {

    var occupancyGrid: OccupancyGrid?

    private var meshAnchors: [MeshAnchorSnapshot] = []
    private let scnView = SCNView()
    private let scene = SCNScene()
    private let geometryRoot = SCNNode()
    private let deviceNode = SCNNode()
    private let cameraNode = SCNNode()

    private var cameraAzimuth: Float = 0
    private var cameraElevation: Float = 0.85
    private var cameraDistance: Float = 8.0
    private var deviceHeight: Float = 0.3   // ARKit Y of the device; updated from devicePosition.z

    private var isBuilding = false
    private var needsRebuild = false
    private var hasAutoOrientedCamera = false

    override init(frame: CGRect) {
        super.init(frame: frame)
        setupSceneView()
        setupScene()
        setupGestures()
        setupLegend()
    }

    required init?(coder: NSCoder) { fatalError() }

    // MARK: - Setup

    private func setupSceneView() {
        scnView.translatesAutoresizingMaskIntoConstraints = false
        scnView.scene = scene
        scnView.backgroundColor = UIColor(white: 0.06, alpha: 1)
        scnView.allowsCameraControl = false
        scnView.antialiasingMode = .multisampling2X
        addSubview(scnView)
        NSLayoutConstraint.activate([
            scnView.topAnchor.constraint(equalTo: topAnchor),
            scnView.leadingAnchor.constraint(equalTo: leadingAnchor),
            scnView.trailingAnchor.constraint(equalTo: trailingAnchor),
            scnView.bottomAnchor.constraint(equalTo: bottomAnchor),
        ])
    }

    private func setupScene() {
        let ambient = SCNNode()
        ambient.light = SCNLight()
        ambient.light!.type = .ambient
        ambient.light!.color = UIColor(white: 0.45, alpha: 1)
        scene.rootNode.addChildNode(ambient)

        let sun = SCNNode()
        sun.light = SCNLight()
        sun.light!.type = .directional
        sun.light!.color = UIColor(white: 0.75, alpha: 1)
        sun.eulerAngles = SCNVector3(-Float.pi / 3.2, Float.pi / 4.5, 0)
        scene.rootNode.addChildNode(sun)

        scene.rootNode.addChildNode(geometryRoot)

        let coneGeo = SCNCone(topRadius: 0, bottomRadius: 0.09, height: 0.20)
        coneGeo.firstMaterial!.diffuse.contents = UIColor.cyan
        coneGeo.firstMaterial!.lightingModel = .lambert
        deviceNode.geometry = coneGeo
        scene.rootNode.addChildNode(deviceNode)

        cameraNode.camera = SCNCamera()
        cameraNode.camera!.zFar = 200
        cameraNode.camera!.fieldOfView = 58
        scene.rootNode.addChildNode(cameraNode)
        updateCamera(cx: 0, cz: 0)
    }

    private func setupGestures() {
        let pan = UIPanGestureRecognizer(target: self, action: #selector(handlePan))
        pan.maximumNumberOfTouches = 1
        if #available(iOS 13.4, *) {
            pan.allowedScrollTypesMask = .all
        }
        addGestureRecognizer(pan)

        let pinch = UIPinchGestureRecognizer(target: self, action: #selector(handlePinch))
        addGestureRecognizer(pinch)
    }

    private func setupLegend() {
        let container = UIView()
        container.translatesAutoresizingMaskIntoConstraints = false
        container.backgroundColor = UIColor(white: 0, alpha: 0.55)
        container.layer.cornerRadius = 8
        addSubview(container)

        let stack = UIStackView()
        stack.translatesAutoresizingMaskIntoConstraints = false
        stack.axis = .vertical
        stack.spacing = 5
        container.addSubview(stack)

        let entries: [MeshClassification] = [.wall, .floor, .table, .seat, .window, .door]
        for cls in entries {
            let row = UIStackView()
            row.axis = .horizontal
            row.spacing = 5
            row.alignment = .center

            let swatch = UIView()
            swatch.translatesAutoresizingMaskIntoConstraints = false
            swatch.layer.cornerRadius = 3
            let c = cls.color
            swatch.backgroundColor = UIColor(red: c.r, green: c.g, blue: c.b, alpha: 1)
            NSLayoutConstraint.activate([
                swatch.widthAnchor.constraint(equalToConstant: 10),
                swatch.heightAnchor.constraint(equalToConstant: 10),
            ])

            let lbl = UILabel()
            lbl.text = cls.label
            lbl.textColor = UIColor(white: 0.9, alpha: 1)
            lbl.font = .systemFont(ofSize: 10, weight: .medium)

            row.addArrangedSubview(swatch)
            row.addArrangedSubview(lbl)
            stack.addArrangedSubview(row)
        }

        NSLayoutConstraint.activate([
            stack.topAnchor.constraint(equalTo: container.topAnchor, constant: 8),
            stack.leadingAnchor.constraint(equalTo: container.leadingAnchor, constant: 8),
            stack.trailingAnchor.constraint(equalTo: container.trailingAnchor, constant: -8),
            stack.bottomAnchor.constraint(equalTo: container.bottomAnchor, constant: -8),

            container.leadingAnchor.constraint(equalTo: leadingAnchor, constant: 10),
            container.bottomAnchor.constraint(equalTo: bottomAnchor, constant: -10),
        ])
    }

    // MARK: - Camera

    @objc private func handlePan(_ g: UIPanGestureRecognizer) {
        guard g.state == .began || g.state == .changed else { return }

        let delta = g.translation(in: self)
        guard delta != .zero else { return }

        // Positive delta_x → orbit right; positive delta_y (mouse moves down/scrolls down) → raise elevation
        cameraAzimuth -= Float(delta.x) * 0.013
        cameraElevation = max(0.1, min(.pi / 2 - 0.05, cameraElevation + Float(delta.y) * 0.013))
        g.setTranslation(.zero, in: self)

        let pos = occupancyGrid?.devicePosition ?? .zero
        updateCamera(cx: pos.x, cz: pos.y)
    }

    @objc private func handlePinch(_ g: UIPinchGestureRecognizer) {
        guard g.state == .changed else { return }
        cameraDistance = max(2.5, min(22, cameraDistance / Float(g.scale)))
        g.scale = 1
        let pos = occupancyGrid?.devicePosition ?? .zero
        updateCamera(cx: pos.x, cz: pos.y)
    }

    // cx/cz are the ARKit X and Z of the scene pivot (device position).
    // Camera orbits around (cx, deviceHeight, cz) at the current azimuth/elevation.
    private func updateCamera(cx: Float, cz: Float) {
        let d = cameraDistance
        let camX = cx + d * cos(cameraElevation) * sin(cameraAzimuth)
        let camZ = cz + d * cos(cameraElevation) * cos(cameraAzimuth)
        let camY = deviceHeight + d * sin(cameraElevation)
        cameraNode.position = SCNVector3(camX, camY, camZ)
        cameraNode.look(at: SCNVector3(cx, deviceHeight, cz))
    }

    // MARK: - Public Interface

    /// Update device cone and camera without rebuilding mesh geometry.
    func refresh() {
        guard let pos = occupancyGrid?.devicePosition else { return }
        deviceHeight = pos.z
        deviceNode.position = SCNVector3(pos.x, pos.z, pos.y)
        deviceNode.eulerAngles = SCNVector3(-.pi / 2, pos.heading, 0)
        updateCamera(cx: pos.x, cz: pos.y)
    }

    /// Replace stored mesh anchors and trigger an async geometry rebuild.
    func updateMeshAnchors(_ anchors: [MeshAnchorSnapshot]) {
        meshAnchors = anchors
        if !hasAutoOrientedCamera && !anchors.isEmpty {
            hasAutoOrientedCamera = true
            autoOrientCamera(anchors: anchors)
        }
        rebuildMesh()
    }

    /// Orient the camera so the mesh appears in front of the device rather than behind it.
    /// Uses the mean anchor translation as a quick proxy for the mesh centroid.
    private func autoOrientCamera(anchors: [MeshAnchorSnapshot]) {
        let pos = occupancyGrid?.devicePosition ?? .zero
        var sumX: Float = 0, sumZ: Float = 0, cnt = 0
        for anchor in anchors where anchor.transform.count == 16 {
            sumX += anchor.transform[12]   // column-3 x = ARKit world X
            sumZ += anchor.transform[14]   // column-3 z = ARKit world Z
            cnt += 1
        }
        guard cnt > 0 else { return }
        let centX = sumX / Float(cnt)
        let centZ = sumZ / Float(cnt)   // SceneKit Z = ARKit Z

        // Vector from device to mesh centroid in the XZ plane
        let dx = centX - pos.x
        let dz = centZ - pos.y    // pos.y holds ARKit Z
        let len = sqrt(dx*dx + dz*dz)
        guard len > 0.1 else { return }

        // Place camera on the opposite side: camera → device → centroid are collinear,
        // so sin(az) = -dx/len, cos(az) = -dz/len  →  az = atan2(-dx, -dz)
        cameraAzimuth = atan2(-dx, -dz)
        updateCamera(cx: pos.x, cz: pos.y)
    }

    // MARK: - Mesh Building

    private func rebuildMesh() {
        guard !isBuilding else {
            needsRebuild = true
            return
        }
        isBuilding = true
        needsRebuild = false

        let anchors = meshAnchors
        let pos = occupancyGrid?.devicePosition ?? .zero

        DispatchQueue.global(qos: .userInitiated).async { [weak self] in
            guard let self else { return }
            let nodes = anchors.compactMap { self.makeAnchorNode($0) }
            DispatchQueue.main.async {
                self.geometryRoot.childNodes.forEach { $0.removeFromParentNode() }
                nodes.forEach { self.geometryRoot.addChildNode($0) }
                self.deviceHeight = pos.z
                self.deviceNode.position = SCNVector3(pos.x, pos.z, pos.y)
                self.deviceNode.eulerAngles = SCNVector3(-.pi / 2, pos.heading, 0)
                self.updateCamera(cx: pos.x, cz: pos.y)
                self.isBuilding = false
                if self.needsRebuild { self.rebuildMesh() }
            }
        }
    }

    private func makeAnchorNode(_ anchor: MeshAnchorSnapshot) -> SCNNode? {
        guard let vData = Data(base64Encoded: anchor.verticesB64),
              let iData = Data(base64Encoded: anchor.indicesB64),
              let cData = Data(base64Encoded: anchor.classificationsB64),
              anchor.transform.count == 16,
              anchor.triangleCount > 0 else { return nil }

        let vertexCount = anchor.vertexCount
        let triangleCount = anchor.triangleCount

        guard vData.count == vertexCount * 12,
              iData.count == triangleCount * 12 else { return nil }

        let localVerts: [Float] = vData.withUnsafeBytes { ptr in
            Array(ptr.bindMemory(to: Float.self))
        }
        let indices: [UInt32] = iData.withUnsafeBytes { ptr in
            Array(ptr.bindMemory(to: UInt32.self))
        }
        let classBytes = Array(cData)
        let vertexColorBytes = anchor.vertexColorsB64.flatMap { Data(base64Encoded: $0) }
        let hasVertexColors = vertexColorBytes?.count == vertexCount * 4

        // Reconstruct column-major 4x4 transform
        let t = anchor.transform
        let mat = simd_float4x4(columns: (
            simd_float4(t[0],  t[1],  t[2],  t[3]),
            simd_float4(t[4],  t[5],  t[6],  t[7]),
            simd_float4(t[8],  t[9],  t[10], t[11]),
            simd_float4(t[12], t[13], t[14], t[15])
        ))

        // Transform vertices to ARKit world space (which maps directly to SceneKit space)
        var worldVerts = [Float](repeating: 0, count: vertexCount * 3)
        for i in 0..<vertexCount {
            let lv = simd_float4(localVerts[i*3], localVerts[i*3+1], localVerts[i*3+2], 1)
            let wv = mat * lv
            worldVerts[i*3]   = wv.x
            worldVerts[i*3+1] = wv.y
            worldVerts[i*3+2] = wv.z
        }

        // Accumulate face normals per vertex for smooth shading
        var accNormals = [simd_float3](repeating: .zero, count: vertexCount)
        for f in 0..<triangleCount {
            let i0 = Int(indices[f*3]), i1 = Int(indices[f*3+1]), i2 = Int(indices[f*3+2])
            guard i0 < vertexCount, i1 < vertexCount, i2 < vertexCount else { continue }
            let v0 = simd_float3(worldVerts[i0*3], worldVerts[i0*3+1], worldVerts[i0*3+2])
            let v1 = simd_float3(worldVerts[i1*3], worldVerts[i1*3+1], worldVerts[i1*3+2])
            let v2 = simd_float3(worldVerts[i2*3], worldVerts[i2*3+1], worldVerts[i2*3+2])
            let faceNorm = simd_normalize(simd_cross(v1 - v0, v2 - v0))
            guard !faceNorm.x.isNaN else { continue }
            accNormals[i0] += faceNorm
            accNormals[i1] += faceNorm
            accNormals[i2] += faceNorm
        }
        var normalFloats = [Float](repeating: 0, count: vertexCount * 3)
        for i in 0..<vertexCount {
            let len = simd_length(accNormals[i])
            let n = len > 0.0001 ? accNormals[i] / len : simd_float3(0, 1, 0)
            normalFloats[i*3]   = n.x
            normalFloats[i*3+1] = n.y
            normalFloats[i*3+2] = n.z
        }

        // Group triangles by classification (skip ceiling = 3)
        var classGroups: [UInt8: [UInt32]] = [:]
        for f in 0..<triangleCount {
            let cls: UInt8 = f < classBytes.count ? classBytes[f] : 0
            guard cls != 3 else { continue }
            if classGroups[cls] == nil { classGroups[cls] = [] }
            classGroups[cls]!.append(contentsOf: [indices[f*3], indices[f*3+1], indices[f*3+2]])
        }
        guard !classGroups.isEmpty else { return nil }

        let posData  = worldVerts.withUnsafeBufferPointer { Data(buffer: $0) }
        let normData = normalFloats.withUnsafeBufferPointer { Data(buffer: $0) }

        let posSource = SCNGeometrySource(data: posData, semantic: .vertex,
            vectorCount: vertexCount, usesFloatComponents: true,
            componentsPerVector: 3, bytesPerComponent: 4, dataOffset: 0, dataStride: 12)
        let normSource = SCNGeometrySource(data: normData, semantic: .normal,
            vectorCount: vertexCount, usesFloatComponents: true,
            componentsPerVector: 3, bytesPerComponent: 4, dataOffset: 0, dataStride: 12)

        var sources = [posSource, normSource]
        if hasVertexColors, let vertexColorBytes {
            let colorFloats = makeNormalizedColorFloats(from: vertexColorBytes)
            let colorData = colorFloats.withUnsafeBufferPointer { Data(buffer: $0) }
            let colorSource = SCNGeometrySource(data: colorData, semantic: .color,
                vectorCount: vertexCount, usesFloatComponents: true,
                componentsPerVector: 4, bytesPerComponent: 4, dataOffset: 0, dataStride: 16)
            sources.append(colorSource)
        }

        var elements:  [SCNGeometryElement] = []
        var materials: [SCNMaterial] = []

        for (cls, triIndices) in classGroups.sorted(by: { $0.key < $1.key }) {
            let idxData = triIndices.withUnsafeBufferPointer { Data(buffer: $0) }
            let element = SCNGeometryElement(data: idxData, primitiveType: .triangles,
                primitiveCount: triIndices.count / 3, bytesPerIndex: 4)
            elements.append(element)

            let mat = SCNMaterial()
            if hasVertexColors {
                mat.diffuse.contents = UIColor.white
            } else {
                let classification = MeshClassification(rawValue: cls) ?? .none
                let c = classification.color
                mat.diffuse.contents = UIColor(red: c.r, green: c.g, blue: c.b, alpha: 0.9)
            }
            mat.lightingModel = .lambert
            mat.isDoubleSided = true
            materials.append(mat)
        }

        let geo = SCNGeometry(sources: sources, elements: elements)
        geo.materials = materials
        return SCNNode(geometry: geo)
    }

    private func makeNormalizedColorFloats(from data: Data) -> [Float] {
        var colors: [Float] = []
        colors.reserveCapacity(data.count)
        for byte in data {
            colors.append(Float(byte) / 255.0)
        }
        return colors
    }
}
