import UIKit
import SceneKit
import GLTFKit2
import CryptoKit

final class RobotModelSceneView: SCNView, UIGestureRecognizerDelegate {
    struct Part {
        let id: String
        let name: String
        let node: SCNNode
        let rest: simd_float4x4
        let mesh: RobotRigGeometry
        let materials: [SCNMaterial]
    }

    struct ModelNode {
        let id: String
        let name: String
        let parts: Set<String>
        let children: [ModelNode]

        var allParts: Set<String> {
            children.reduce(into: parts) { $0.formUnion($1.allParts) }
        }
    }

    private(set) var parts: [Part] = []
    private(set) var modelHierarchy: ModelNode?
    private let modelRoot = SCNNode()
    private let overlayRoot = SCNNode()
    private let gizmoRoot = SCNNode()
    private let ikRoot = SCNNode()
    private var ikTarget: SIMD3<Float>?
    private var ikParts = Set<String>()
    private var ikPlaneDrag: (target: SIMD3<Float>, screen: CGPoint, depth: Float)?
    private var movableAxis: RobotRigAxis?
    private var axisDrag: (axis: RobotRigAxis, direction: SIMD3<Float>, parameter: Float)?
    private lazy var axisPan = UIPanGestureRecognizer(target: self, action: #selector(dragAxis(_:)))
    private let cameraNode = SCNNode()
    private var radius: Float = 1
    private var modelCenter = SIMD3<Float>.zero
    var onPick: ((String, Int) -> Void)?
    var onAxisMove: ((RobotRigAxis, UIGestureRecognizer.State) -> Void)?
    var onHelperPick: ((String, SIMD3<Float>) -> Void)?
    var onIKDragBegan: (() -> Void)?
    var onIKMove: ((SIMD3<Float>, UIGestureRecognizer.State) -> Void)?

    init() {
        super.init(frame: .zero, options: nil)
        scene = SCNScene()
        scene?.rootNode.addChildNode(modelRoot)
        scene?.rootNode.addChildNode(overlayRoot)
        scene?.rootNode.addChildNode(gizmoRoot)
        scene?.rootNode.addChildNode(ikRoot)
        scene?.rootNode.addChildNode(cameraNode)
        cameraNode.camera = SCNCamera()
        pointOfView = cameraNode
        backgroundColor = UIColor(white: 0.10, alpha: 1)
        autoenablesDefaultLighting = true
        allowsCameraControl = true
        antialiasingMode = .multisampling4X
        defaultCameraController.interactionMode = .orbitTurntable
        axisPan.maximumNumberOfTouches = 1
        axisPan.delegate = self
        for gesture in gestureRecognizers ?? [] { gesture.require(toFail: axisPan) }
        addGestureRecognizer(axisPan)
        let tap = UITapGestureRecognizer(target: self, action: #selector(pick(_:)))
        tap.require(toFail: axisPan)
        addGestureRecognizer(tap)
        accessibilityLabel = "Robot model"
    }

    required init?(coder: NSCoder) { fatalError("init(coder:) has not been implemented") }

    func install(_ imported: SCNScene) throws {
        var collected: [Part] = []
        var triangleCount = 0
        func visit(_ node: SCNNode, path: String) throws -> ModelNode {
            var nodeParts = Set<String>()
            guard node.skinner == nil, node.morpher == nil else { throw RobotRigError.invalid("Skinned or morphing meshes are not supported") }
            if let geometry = node.geometry {
                guard let source = geometry.sources(for: .vertex).first else { throw RobotRigError.invalid("Mesh has no positions") }
                let positions = try Self.positions(source)
                var triangles: [RobotRigTriangle] = []
                var indicesByElement: [[UInt32]] = []
                let rest = node.simdWorldTransform
                guard abs(simd_determinant(rest)) > 1e-12 else { throw RobotRigError.invalid("Mesh has a singular transform") }
                for (elementIndex, element) in geometry.elements.enumerated() {
                    let indices = try Self.triangleIndices(element, vertexCount: positions.count)
                    indicesByElement.append(indices)
                    for start in stride(from: 0, to: indices.count, by: 3) {
                        let points = indices[start..<(start + 3)].map { index -> SIMD3<Float> in
                            let point = rest * SIMD4(positions[Int(index)], 1)
                            return SIMD3(point.x, point.y, point.z)
                        }
                        triangles.append(RobotRigTriangle(first: points[0], second: points[1], third: points[2], element: elementIndex, face: start / 3))
                    }
                }
                triangleCount += triangles.count
                guard triangleCount <= 1_000_000 else { throw RobotRigError.invalid("Model exceeds 1,000,000 triangles after mesh instances are expanded") }
                if !triangles.isEmpty {
                    let mesh = try RobotRigGeometry(triangles: triangles)
                    let components = mesh.components()
                    guard collected.count + components.count <= 2000 else { throw RobotRigError.invalid("Model exceeds 2,000 selectable parts") }
                    for (componentIndex, component) in components.enumerated() {
                        var elements: [SCNGeometryElement] = []
                        var componentTriangles: [RobotRigTriangle] = []
                        for elementIndex in geometry.elements.indices {
                            var subset: [UInt32] = []
                            for triangleIndex in component where triangles[triangleIndex].element == elementIndex {
                                var triangle = triangles[triangleIndex]
                                let offset = triangle.face * 3
                                subset.append(contentsOf: indicesByElement[elementIndex][offset..<(offset + 3)])
                                triangle.face = subset.count / 3 - 1
                                componentTriangles.append(triangle)
                            }
                            elements.append(SCNGeometryElement(indices: subset, primitiveType: .triangles))
                        }
                        let partGeometry = SCNGeometry(sources: geometry.sources, elements: elements)
                        partGeometry.materials = geometry.materials.map { $0.copy() as! SCNMaterial }
                        let partNode = SCNNode(geometry: partGeometry)
                        let id = "\(path)/component/\(componentIndex)"
                        nodeParts.insert(id)
                        partNode.name = id
                        partNode.simdTransform = rest
                        let name = (node.name?.isEmpty == false ? node.name! : "Mesh \(collected.count + 1)") + (components.count > 1 ? " [\(componentIndex + 1)]" : "")
                        collected.append(Part(id: id, name: name, node: partNode, rest: rest,
                                              mesh: try RobotRigGeometry(triangles: componentTriangles), materials: partGeometry.materials))
                    }
                }
            }
            let children = try node.childNodes.enumerated().map { index, child in
                try visit(child, path: "\(path)/\(index)")
            }.filter { !$0.allParts.isEmpty }
            return ModelNode(id: path, name: node.name?.isEmpty == false ? node.name! : (path == "scene" ? "Model" : "Node \(path.split(separator: "/").last ?? "")"), parts: nodeParts, children: children)
        }
        let hierarchy = try visit(imported.rootNode, path: "scene")
        guard !collected.isEmpty else { throw RobotRigError.invalid("No selectable triangle meshes") }
        parts = collected
        modelHierarchy = hierarchy
        modelRoot.childNodes.forEach { $0.removeFromParentNode() }
        overlayRoot.childNodes.forEach { $0.removeFromParentNode() }
        for part in parts { modelRoot.addChildNode(part.node) }
        frameAll()
    }

    func frameAll() {
        var lower = SIMD3<Float>(repeating: .greatestFiniteMagnitude)
        var upper = -lower
        for part in parts {
            for triangle in part.mesh.triangles {
                for point in [triangle.first, triangle.second, triangle.third] {
                    lower = simd_min(lower, point)
                    upper = simd_max(upper, point)
                }
            }
        }
        guard !parts.isEmpty else { return }
        modelCenter = (lower + upper) / 2
        radius = max(simd_length(upper - lower) / 2, 0.001)
        cameraNode.camera?.zNear = Double(radius / 1000)
        cameraNode.camera?.zFar = Double(radius * 100)
        cameraNode.simdPosition = modelCenter + SIMD3(0.5, 0.3, 1) * radius * 3.8
        cameraNode.look(at: SCNVector3(modelCenter))
        defaultCameraController.target = SCNVector3(modelCenter)
        defaultCameraController.pointOfView = cameraNode
    }

    func update(document: RobotRigDocument, angles: [UUID: Float], selected: Set<String>, isolated: Bool) {
        let transforms = document.transforms(angles: angles)
        for part in parts {
            let owner = document.groups.first { $0.parts.contains(part.id) }
            part.node.simdTransform = (owner.flatMap { transforms[$0.id] } ?? matrix_identity_float4x4) * part.rest
            part.node.isHidden = isolated && !selected.contains(part.id)
            part.node.geometry?.materials = part.materials.map { material in
                let copy = material.copy() as! SCNMaterial
                if selected.contains(part.id) { copy.emission.contents = UIColor(red: 0.05, green: 0.35, blue: 0.3, alpha: 1) }
                return copy
            }
        }
    }

    func showAxis(_ axis: RobotRigAxis?, surface: RobotRigSurface? = nil, partID: String? = nil, transform: simd_float4x4 = matrix_identity_float4x4, movable: Bool = false) {
        guard axisDrag == nil, ikPlaneDrag == nil else { return }
        ikTarget = nil
        ikParts.removeAll()
        ikRoot.childNodes.forEach { $0.removeFromParentNode() }
        overlayRoot.childNodes.forEach { $0.removeFromParentNode() }
        gizmoRoot.childNodes.forEach { $0.removeFromParentNode() }
        movableAxis = movable ? axis : nil
        overlayRoot.simdTransform = transform
        guard let axis else { return }
        let length = radius * 1.2
        let shaft = SCNNode(geometry: SCNCylinder(radius: CGFloat(radius * 0.006), height: CGFloat(length)))
        shaft.simdPosition = axis.origin
        shaft.simdOrientation = simd_quatf(from: SIMD3(0, 1, 0), to: axis.direction)
        shaft.geometry?.firstMaterial?.diffuse.contents = UIColor.systemYellow
        overlayRoot.addChildNode(shaft)
        let arrow = SCNNode(geometry: SCNCone(topRadius: 0, bottomRadius: CGFloat(radius * 0.035), height: CGFloat(radius * 0.12)))
        arrow.simdPosition = axis.origin + axis.direction * length / 2
        arrow.simdOrientation = shaft.simdOrientation
        arrow.geometry?.firstMaterial?.diffuse.contents = UIColor.systemYellow
        overlayRoot.addChildNode(arrow)
        let pivot = SCNNode(geometry: SCNSphere(radius: CGFloat(radius * 0.025)))
        pivot.simdPosition = axis.origin
        pivot.geometry?.firstMaterial?.diffuse.contents = UIColor.systemRed
        overlayRoot.addChildNode(pivot)
        if movable { showMoveHandles(axis) }
        if let surface, let part = parts.first(where: { $0.id == partID }) {
            let vertices = surface.triangles.sorted().flatMap { index in
                let triangle = part.mesh.triangles[index]
                return [triangle.first, triangle.second, triangle.third].map { SCNVector3($0 + axis.direction * radius * 0.0002) }
            }
            let geometry = SCNGeometry(sources: [SCNGeometrySource(vertices: vertices)], elements: [SCNGeometryElement(indices: Array(0..<UInt32(vertices.count)), primitiveType: .triangles)])
            geometry.firstMaterial?.diffuse.contents = UIColor.systemYellow.withAlphaComponent(0.7)
            geometry.firstMaterial?.isDoubleSided = true
            overlayRoot.addChildNode(SCNNode(geometry: geometry))
        }
    }

    private func showMoveHandles(_ axis: RobotRigAxis) {
        let directions: [SIMD3<Float>] = [SIMD3(1, 0, 0), SIMD3(0, 1, 0), SIMD3(0, 0, 1)]
        let colors: [UIColor] = [.systemRed, .systemGreen, .systemBlue]
        let names = ["X", "Y", "Z"]
        gizmoRoot.simdPosition = axis.origin
        for index in directions.indices {
            let handle = SCNNode()
            handle.name = names[index]
            handle.simdOrientation = simd_quatf(from: SIMD3(0, 1, 0), to: directions[index])
            let shaft = SCNNode(geometry: SCNCylinder(radius: CGFloat(radius * 0.012), height: CGFloat(radius * 0.42)))
            shaft.position.y = radius * 0.25
            let tip = SCNNode(geometry: SCNCone(topRadius: 0, bottomRadius: CGFloat(radius * 0.045), height: CGFloat(radius * 0.16)))
            tip.position.y = radius * 0.54
            for node in [shaft, tip] {
                node.name = names[index]
                let material = SCNMaterial()
                material.lightingModel = .constant
                material.diffuse.contents = colors[index]
                material.readsFromDepthBuffer = false
                material.writesToDepthBuffer = false
                node.geometry?.materials = [material]
                node.renderingOrder = 100
                handle.addChildNode(node)
            }
            gizmoRoot.addChildNode(handle)
            let text = SCNText(string: names[index], extrusionDepth: 0)
            text.font = .systemFont(ofSize: 1, weight: .bold)
            text.firstMaterial = tip.geometry?.firstMaterial
            let label = SCNNode(geometry: text)
            label.simdScale = SIMD3(repeating: radius * 0.07)
            label.simdPosition = directions[index] * radius * 0.69
            label.constraints = [SCNBillboardConstraint()]
            label.renderingOrder = 100
            gizmoRoot.addChildNode(label)
        }
    }

    override func gestureRecognizerShouldBegin(_ gestureRecognizer: UIGestureRecognizer) -> Bool {
        guard gestureRecognizer === axisPan else { return super.gestureRecognizerShouldBegin(gestureRecognizer) }
        let location = axisPan.location(in: self)
        let translation = axisPan.translation(in: self)
        return beginAxisDrag(at: CGPoint(x: location.x - translation.x, y: location.y - translation.y))
    }

    func showIK(target: SIMD3<Float>, endpoint: SIMD3<Float>, parts: Set<String>) {
        ikTarget = target
        ikParts = parts
        movableAxis = RobotRigAxis(origin: target, direction: SIMD3(0, 1, 0))
        overlayRoot.childNodes.forEach { $0.removeFromParentNode() }
        gizmoRoot.childNodes.forEach { $0.removeFromParentNode() }
        ikRoot.childNodes.forEach { $0.removeFromParentNode() }
        showMoveHandles(movableAxis!)
        for (point, color, name) in [(target, UIColor.systemPink, "IK Target"), (endpoint, UIColor.systemCyan, "IK Helper")] {
            let node = SCNNode(geometry: SCNSphere(radius: CGFloat(radius * 0.035)))
            node.simdPosition = point
            node.name = name
            node.geometry?.firstMaterial?.lightingModel = .constant
            node.geometry?.firstMaterial?.diffuse.contents = color
            node.geometry?.firstMaterial?.readsFromDepthBuffer = false
            node.renderingOrder = 101
            ikRoot.addChildNode(node)
        }
        let line = SCNGeometry(sources: [SCNGeometrySource(vertices: [SCNVector3(endpoint), SCNVector3(target)])],
                               elements: [SCNGeometryElement(indices: [Int32(0), 1], primitiveType: .line)])
        line.firstMaterial?.lightingModel = .constant
        line.firstMaterial?.diffuse.contents = UIColor.systemPink
        line.firstMaterial?.readsFromDepthBuffer = false
        let node = SCNNode(geometry: line)
        node.renderingOrder = 100
        ikRoot.addChildNode(node)
    }

    func beginAxisDrag(at point: CGPoint) -> Bool {
        guard let axis = movableAxis, axisDrag == nil, ikPlaneDrag == nil else { return false }
        guard let hit = hitTest(point, options: [.rootNode: gizmoRoot, .searchMode: SCNHitTestSearchMode.all.rawValue]).first(where: { ["X", "Y", "Z"].contains($0.node.name ?? "") }) else {
            guard let target = ikTarget else { return false }
            let markerHit = !hitTest(point, options: [.rootNode: ikRoot]).filter { $0.node.geometry is SCNSphere }.isEmpty
            let modelHit = hitTest(point, options: [.rootNode: modelRoot, .searchMode: SCNHitTestSearchMode.closest.rawValue]).first
            guard markerHit || modelHit.map({ ikParts.contains($0.node.name ?? "") }) == true else { return false }
            ikPlaneDrag = (target, point, projectPoint(SCNVector3(target)).z)
            onIKDragBegan?()
            return true
        }
        let direction: SIMD3<Float>
        switch hit.node.name {
        case "X": direction = SIMD3(1, 0, 0)
        case "Y": direction = SIMD3(0, 1, 0)
        default: direction = SIMD3(0, 0, 1)
        }
        guard let parameter = axisParameter(at: point, origin: axis.origin, direction: direction) else { return false }
        axisDrag = (axis, direction, parameter)
        if ikTarget != nil { onIKDragBegan?() }
        return true
    }

    private func axisParameter(at point: CGPoint, origin: SIMD3<Float>, direction: SIMD3<Float>) -> Float? {
        let near = SIMD3<Float>(unprojectPoint(SCNVector3(Float(point.x), Float(point.y), 0)))
        let far = SIMD3<Float>(unprojectPoint(SCNVector3(Float(point.x), Float(point.y), 1)))
        let ray = simd_normalize(far - near)
        let alignment = simd_dot(ray, direction)
        let denominator = 1 - alignment * alignment
        guard denominator > 0.005 else { return nil }
        let offset = origin - near
        let parameter = (alignment * simd_dot(ray, offset) - simd_dot(direction, offset)) / denominator
        return parameter.isFinite ? parameter : nil
    }

    @objc private func dragAxis(_ gesture: UIPanGestureRecognizer) {
        moveAxisDrag(to: gesture.location(in: self), state: gesture.state)
    }

    func moveAxisDrag(to point: CGPoint, state: UIGestureRecognizer.State) {
        if let drag = ikPlaneDrag {
            let finished = state == .ended || state == .cancelled || state == .failed
            let start = SIMD3<Float>(unprojectPoint(SCNVector3(Float(drag.screen.x), Float(drag.screen.y), drag.depth)))
            let end = SIMD3<Float>(unprojectPoint(SCNVector3(Float(point.x), Float(point.y), drag.depth)))
            let target = state == .cancelled || state == .failed ? drag.target : drag.target + end - start
            if finished { ikPlaneDrag = nil }
            onIKMove?(target, state)
            return
        }
        guard let drag = axisDrag else { return }
        var axis = drag.axis
        if state != .cancelled && state != .failed,
           let parameter = axisParameter(at: point, origin: axis.origin, direction: drag.direction) {
            axis.origin += drag.direction * (parameter - drag.parameter)
        }
        let finished = state == .ended || state == .cancelled || state == .failed
        if ikTarget != nil {
            if finished { axisDrag = nil }
            onIKMove?(axis.origin, state)
            return
        }
        axisDrag = nil
        showAxis(axis, movable: true)
        if !finished { axisDrag = drag }
        onAxisMove?(axis, state)
    }

    func cancelAxisDrag() {
        if let drag = ikPlaneDrag {
            ikPlaneDrag = nil
            onIKMove?(drag.target, .cancelled)
            return
        }
        guard let drag = axisDrag else { return }
        axisDrag = nil
        if ikTarget != nil {
            onIKMove?(drag.axis.origin, .cancelled)
            return
        }
        showAxis(drag.axis, movable: true)
        onAxisMove?(drag.axis, .cancelled)
    }

    @objc private func pick(_ gesture: UITapGestureRecognizer) {
        pick(at: gesture.location(in: self))
    }

    func pick(at point: CGPoint) {
        guard movableAxis == nil else { return }
        guard let hit = hitTest(point, options: [.rootNode: modelRoot, .searchMode: SCNHitTestSearchMode.closest.rawValue]).first,
              let part = parts.first(where: { $0.node === hit.node }),
              let triangle = part.mesh.triangles.firstIndex(where: { $0.element == hit.geometryIndex && $0.face == hit.faceIndex }) else { return }
        if let onHelperPick {
            let local = part.node.simdConvertPosition(SIMD3<Float>(hit.worldCoordinates), from: nil)
            let rest = part.rest * SIMD4(local, 1)
            onHelperPick(part.id, SIMD3(rest.x, rest.y, rest.z))
            return
        }
        onPick?(part.id, triangle)
    }

    private static func positions(_ source: SCNGeometrySource) throws -> [SIMD3<Float>] {
        guard source.usesFloatComponents, source.bytesPerComponent == 4, source.componentsPerVector >= 3,
              source.vectorCount <= 1_500_000, source.dataStride >= 12, source.dataOffset >= 0 else { throw RobotRigError.invalid("Unsupported position buffer") }
        return try source.data.withUnsafeBytes { bytes in
            try (0..<source.vectorCount).map { index in
                let offset = source.dataOffset + index * source.dataStride
                guard offset <= bytes.count - 12 else { throw RobotRigError.invalid("Truncated position buffer") }
                return SIMD3(bytes.loadUnaligned(fromByteOffset: offset, as: Float.self),
                             bytes.loadUnaligned(fromByteOffset: offset + 4, as: Float.self),
                             bytes.loadUnaligned(fromByteOffset: offset + 8, as: Float.self))
            }
        }
    }

    private static func triangleIndices(_ element: SCNGeometryElement, vertexCount: Int) throws -> [UInt32] {
        guard element.primitiveType == .triangles, element.primitiveCount <= 500_000,
              [1, 2, 4].contains(element.bytesPerIndex) else { throw RobotRigError.invalid("Only triangle primitives are supported") }
        let count = element.primitiveCount * 3
        guard element.data.count >= count * element.bytesPerIndex else { throw RobotRigError.invalid("Truncated index buffer") }
        return try element.data.withUnsafeBytes { bytes in
            try (0..<count).map { index in
                let offset = index * element.bytesPerIndex
                let value: UInt32
                switch element.bytesPerIndex {
                case 1: value = UInt32(bytes.load(fromByteOffset: offset, as: UInt8.self))
                case 2: value = UInt32(UInt16(littleEndian: bytes.loadUnaligned(fromByteOffset: offset, as: UInt16.self)))
                default: value = UInt32(littleEndian: bytes.loadUnaligned(fromByteOffset: offset, as: UInt32.self))
                }
                guard value < vertexCount else { throw RobotRigError.invalid("Index outside vertex buffer") }
                return value
            }
        }
    }
}

enum RobotModelStorage {
    static var root: URL {
        FileManager.default.urls(for: .applicationSupportDirectory, in: .userDomainMask)[0].appendingPathComponent("RobotModels", isDirectory: true)
    }

    static func stage(_ url: URL) throws -> (URL, String) {
        let access = url.startAccessingSecurityScopedResource()
        defer { if access { url.stopAccessingSecurityScopedResource() } }
        let size = try url.resourceValues(forKeys: [.fileSizeKey]).fileSize ?? 0
        guard size >= 20, size <= 100 * 1024 * 1024 else { throw RobotRigError.invalid("GLB must be between 20 bytes and 100 MB") }
        let data = try Data(contentsOf: url)
        try validateGLB(data)
        let hash = SHA256.hash(data: data).map { String(format: "%02x", $0) }.joined()
        let directory = root.appendingPathComponent(hash, isDirectory: true)
        try FileManager.default.createDirectory(at: directory, withIntermediateDirectories: true)
        let target = directory.appendingPathComponent("model.glb")
        try data.write(to: target, options: .atomic)
        return (target, hash)
    }

    static func validateGLB(_ data: Data) throws {
        guard data.count >= 20 else { throw RobotRigError.invalid("Truncated GLB") }
        let header = data.withUnsafeBytes { bytes in
            stride(from: 0, to: 20, by: 4).map { UInt32(littleEndian: bytes.loadUnaligned(fromByteOffset: $0, as: UInt32.self)) }
        }
        guard header[0] == 0x46546C67, header[1] == 2, header[2] == data.count,
              header[4] == 0x4E4F534A, header[3] <= data.count - 20,
              let json = try JSONSerialization.jsonObject(with: data.subdata(in: 20..<(20 + Int(header[3])))) as? [String: Any] else {
            throw RobotRigError.invalid("Invalid GLB 2.0 file")
        }
        let extensions = json["extensionsRequired"] as? [String] ?? []
        let supported: Set<String> = ["KHR_materials_unlit", "KHR_texture_transform", "EXT_meshopt_compression", "KHR_mesh_quantization", "EXT_texture_webp"]
        guard Set(extensions).isSubset(of: supported) else { throw RobotRigError.invalid("Unsupported required GLB extensions: \(extensions.joined(separator: ", "))") }
        guard (json["skins"] as? [Any] ?? []).isEmpty else { throw RobotRigError.invalid("Rig editor requires rigid meshes, not skinning") }
        for key in ["buffers", "images"] {
            for resource in json[key] as? [[String: Any]] ?? [] {
                guard resource["uri"] == nil else { throw RobotRigError.invalid("GLB resources must be embedded in its binary buffer") }
                if let length = resource["byteLength"] as? Int {
                    guard length >= 0, length <= 128 * 1024 * 1024 else { throw RobotRigError.invalid("Decoded buffer exceeds 128 MB") }
                }
            }
        }
        for buffer in json["bufferViews"] as? [[String: Any]] ?? [] {
            if let extensions = buffer["extensions"] as? [String: Any],
               let compression = extensions["EXT_meshopt_compression"] as? [String: Any] {
                guard let count = compression["count"] as? Int, let stride = compression["byteStride"] as? Int,
                      count >= 0, count <= 1_500_000, stride > 0, stride <= 256,
                      count * stride <= 128 * 1024 * 1024 else { throw RobotRigError.invalid("Compressed buffer exceeds decoding limits") }
            }
        }
        var count = 0
        for accessor in json["accessors"] as? [[String: Any]] ?? [] {
            let length = accessor["count"] as? Int ?? 0
            guard length >= 0, length <= 1_500_000 else { throw RobotRigError.invalid("Oversized geometry accessor") }
            count += length
            guard count < 12_000_000 else { throw RobotRigError.invalid("Model geometry is too large") }
        }
        for mesh in json["meshes"] as? [[String: Any]] ?? [] {
            for primitive in mesh["primitives"] as? [[String: Any]] ?? [] {
                guard (primitive["mode"] as? Int ?? 4) == 4,
                      (primitive["targets"] as? [Any] ?? []).isEmpty else { throw RobotRigError.invalid("Only rigid triangle meshes are supported") }
            }
        }
    }

    static func loadDocument(at url: URL, hash: String) throws -> RobotRigDocument {
        let state = url.deletingLastPathComponent().appendingPathComponent("rig.json")
        guard FileManager.default.fileExists(atPath: state.path) else { return RobotRigDocument(assetHash: hash) }
        let document = try JSONDecoder().decode(RobotRigDocument.self, from: Data(contentsOf: state))
        guard document.assetHash == hash else { throw RobotRigError.invalid("Rig does not match the model") }
        return document
    }

    static func save(_ document: RobotRigDocument, at url: URL) throws {
        let encoder = JSONEncoder()
        encoder.outputFormatting = [.prettyPrinted, .sortedKeys]
        try encoder.encode(document).write(to: url.deletingLastPathComponent().appendingPathComponent("rig.json"), options: .atomic)
        UserDefaults.standard.set(document.assetHash, forKey: "RobotModel.lastAsset")
    }
}