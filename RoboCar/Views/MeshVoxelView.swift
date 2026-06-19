//
//  MeshVoxelView.swift
//  RoboCar
//

import UIKit
import SceneKit

final class MeshVoxelView: UIView {

    var occupancyGrid: OccupancyGrid?
    var viewRadiusMeters: Float = 5.0

    private let scnView = SCNView()
    private let scene = SCNScene()
    private let geometryRoot = SCNNode()
    private let deviceNode = SCNNode()
    private let cameraNode = SCNNode()

    private var cameraAzimuth: Float = 0
    private var cameraElevation: Float = 0.55
    private var cameraDistance: Float = 8.0
    private var panStart: CGPoint = .zero

    private var isBuilding = false
    private var needsRebuild = false

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
        switch g.state {
        case .began:
            panStart = g.location(in: self)
        case .changed:
            let loc = g.location(in: self)
            cameraAzimuth -= Float(loc.x - panStart.x) * 0.013
            cameraElevation = max(0.1, min(.pi / 2 - 0.05, cameraElevation + Float(loc.y - panStart.y) * 0.013))
            panStart = loc
            let pos = occupancyGrid?.devicePosition ?? .zero
            updateCamera(cx: pos.x, cz: pos.y)
        default: break
        }
    }

    @objc private func handlePinch(_ g: UIPinchGestureRecognizer) {
        guard g.state == .changed else { return }
        cameraDistance = max(2.5, min(22, cameraDistance / Float(g.scale)))
        g.scale = 1
        let pos = occupancyGrid?.devicePosition ?? .zero
        updateCamera(cx: pos.x, cz: pos.y)
    }

    private func updateCamera(cx: Float, cz: Float) {
        let d = cameraDistance
        let camX = cx + d * cos(cameraElevation) * sin(cameraAzimuth)
        let camZ = cz + d * cos(cameraElevation) * cos(cameraAzimuth)
        let camY = d * sin(cameraElevation)
        cameraNode.position = SCNVector3(camX, camY, camZ)
        cameraNode.look(at: SCNVector3(cx, 0.3, cz))
    }

    // MARK: - Geometry Refresh

    func refresh() {
        guard !isBuilding else {
            needsRebuild = true
            return
        }
        guard let grid = occupancyGrid else { return }
        isBuilding = true
        needsRebuild = false

        let pos = grid.devicePosition
        let radius = viewRadiusMeters

        DispatchQueue.global(qos: .userInitiated).async { [weak self] in
            guard let self else { return }
            let region = grid.getRegion(centerX: pos.x, centerY: pos.y, radiusMeters: radius)
            let items = self.buildGeometry(region: region)
            DispatchQueue.main.async {
                self.applyGeometry(items, devicePos: pos)
                self.isBuilding = false
                if self.needsRebuild { self.refresh() }
            }
        }
    }

    // MARK: - Geometry Building

    private typealias GeomData = (color: UIColor, positions: [Float], normals: [Float], indices: [UInt32], vertexCount: Int)

    private func buildGeometry(region: (cells: [[CellState]], heights: [[Float]], classifications: [[MeshClassification]], originX: Float, originY: Float, cellSize: Float, minHeight: Float, maxHeight: Float)) -> [GeomData] {
        let cs = region.cellSize
        let ox = region.originX
        let oz = region.originY
        let sizeX = region.cells.count
        guard sizeX > 0, region.cells[0].count > 0 else { return [] }
        let sizeY = region.cells[0].count

        // Normalize heights relative to floor
        let floorLevel: Float = region.minHeight < 1e10 ? region.minHeight : 0

        var floorBuf = GeomBuf()
        var classBufs: [MeshClassification: GeomBuf] = [:]

        for ix in 0..<sizeX {
            for iy in 0..<sizeY {
                let state = region.cells[ix][iy]
                guard state != .unknown else { continue }
                let wx = ox + Float(ix) * cs
                let wz = oz + Float(iy) * cs
                let cls = region.classifications[ix][iy]

                switch state {
                case .free:
                    floorBuf.addQuad(x: wx, z: wz, w: cs * 0.96, d: cs * 0.96)

                case .occupied:
                    let rawH = region.heights[ix][iy]
                    let visualH = max(0.06, rawH - floorLevel)
                    if classBufs[cls] == nil { classBufs[cls] = GeomBuf() }
                    classBufs[cls]!.addBox(x: wx, y: visualH / 2, z: wz, w: cs * 0.88, h: visualH, d: cs * 0.88)

                case .unknown:
                    break
                }
            }
        }

        var result: [GeomData] = []

        if !floorBuf.positions.isEmpty {
            let c = MeshClassification.floor.color
            result.append((UIColor(red: c.r, green: c.g, blue: c.b, alpha: 0.8),
                           floorBuf.positions, floorBuf.normals, floorBuf.indices, floorBuf.vertexCount))
        }

        for (cls, buf) in classBufs {
            guard !buf.positions.isEmpty else { continue }
            let c = cls.color
            result.append((UIColor(red: c.r, green: c.g, blue: c.b, alpha: 1),
                           buf.positions, buf.normals, buf.indices, buf.vertexCount))
        }

        return result
    }

    private func applyGeometry(_ items: [GeomData], devicePos: DevicePosition) {
        geometryRoot.childNodes.forEach { $0.removeFromParentNode() }
        for item in items {
            if let node = makeNode(item) { geometryRoot.addChildNode(node) }
        }
        deviceNode.position = SCNVector3(devicePos.x, 0.15, devicePos.y)
        deviceNode.eulerAngles = SCNVector3(-.pi / 2, -devicePos.heading, 0)
        updateCamera(cx: devicePos.x, cz: devicePos.y)
    }

    private func makeNode(_ item: GeomData) -> SCNNode? {
        guard !item.positions.isEmpty else { return nil }

        let posData = item.positions.withUnsafeBufferPointer { Data(buffer: $0) }
        let normData = item.normals.withUnsafeBufferPointer { Data(buffer: $0) }
        let idxData = item.indices.withUnsafeBufferPointer { Data(buffer: $0) }

        let posSource = SCNGeometrySource(data: posData, semantic: .vertex,
            vectorCount: item.vertexCount, usesFloatComponents: true,
            componentsPerVector: 3, bytesPerComponent: 4, dataOffset: 0, dataStride: 12)
        let normSource = SCNGeometrySource(data: normData, semantic: .normal,
            vectorCount: item.vertexCount, usesFloatComponents: true,
            componentsPerVector: 3, bytesPerComponent: 4, dataOffset: 0, dataStride: 12)
        let element = SCNGeometryElement(data: idxData, primitiveType: .triangles,
            primitiveCount: item.indices.count / 3, bytesPerIndex: 4)

        let geo = SCNGeometry(sources: [posSource, normSource], elements: [element])
        let mat = SCNMaterial()
        mat.diffuse.contents = item.color
        mat.lightingModel = .lambert
        mat.isDoubleSided = false
        geo.materials = [mat]

        return SCNNode(geometry: geo)
    }
}

// MARK: - Geometry Buffer

private struct GeomBuf {
    var positions: [Float] = []
    var normals: [Float] = []
    var indices: [UInt32] = []
    var vertexCount: Int = 0

    mutating func addQuad(x: Float, z: Float, w: Float, d: Float) {
        let b = UInt32(vertexCount)
        let hw = w / 2, hd = d / 2
        positions += [x-hw, 0, z-hd,  x+hw, 0, z-hd,  x+hw, 0, z+hd,  x-hw, 0, z+hd]
        normals   += [0, 1, 0,  0, 1, 0,  0, 1, 0,  0, 1, 0]
        indices   += [b, b+1, b+2,  b, b+2, b+3]
        vertexCount += 4
    }

    mutating func addBox(x: Float, y: Float, z: Float, w: Float, h: Float, d: Float) {
        let hw = w/2, hh = h/2, hd = d/2
        let b = UInt32(vertexCount)

        // Top
        positions += [x-hw, y+hh, z-hd,  x+hw, y+hh, z-hd,  x+hw, y+hh, z+hd,  x-hw, y+hh, z+hd]
        normals   += [0,1,0,  0,1,0,  0,1,0,  0,1,0]
        indices   += [b, b+1, b+2,  b, b+2, b+3]

        // Front (+Z)
        positions += [x-hw, y-hh, z+hd,  x+hw, y-hh, z+hd,  x+hw, y+hh, z+hd,  x-hw, y+hh, z+hd]
        normals   += [0,0,1,  0,0,1,  0,0,1,  0,0,1]
        indices   += [b+4, b+5, b+6,  b+4, b+6, b+7]

        // Back (-Z)
        positions += [x+hw, y-hh, z-hd,  x-hw, y-hh, z-hd,  x-hw, y+hh, z-hd,  x+hw, y+hh, z-hd]
        normals   += [0,0,-1,  0,0,-1,  0,0,-1,  0,0,-1]
        indices   += [b+8, b+9, b+10,  b+8, b+10, b+11]

        // Right (+X)
        positions += [x+hw, y-hh, z+hd,  x+hw, y-hh, z-hd,  x+hw, y+hh, z-hd,  x+hw, y+hh, z+hd]
        normals   += [1,0,0,  1,0,0,  1,0,0,  1,0,0]
        indices   += [b+12, b+13, b+14,  b+12, b+14, b+15]

        // Left (-X)
        positions += [x-hw, y-hh, z-hd,  x-hw, y-hh, z+hd,  x-hw, y+hh, z+hd,  x-hw, y+hh, z-hd]
        normals   += [-1,0,0,  -1,0,0,  -1,0,0,  -1,0,0]
        indices   += [b+16, b+17, b+18,  b+16, b+18, b+19]

        vertexCount += 20
    }
}
