//
//  MeshProcessor.swift
//  RoboCar
//
//  Created by Sammy Yousif on 1/3/26.
//

import ARKit
import UIKit
import simd

/// Extracted mesh data that can be processed without retaining ARMeshAnchor
struct ExtractedMeshData {
    let vertices: [simd_float3]
    let classifications: [MeshClassification]
    let transform: simd_float4x4
    let generation: Int  // Generation counter to detect stale data after reset
    let updateId: UInt64 // ID to ensure newer meshes overwrite older ones
    let anchorId: UUID   // Anchor identity for per-anchor cell tracking
}

/// Processes ARMeshAnchor data and updates the occupancy grid
class MeshProcessor {
    private static var vertexColorCache: [String: [UInt8]] = [:]
    
    // MARK: - Configuration
    
    /// Height threshold for obstacles (meters above floor)
    var minObstacleHeight: Float = 0.1  // 10cm minimum
    var maxObstacleHeight: Float = 1.8  // 1.8m maximum (ignore ceiling - typical door height)
    
    /// Absolute ceiling height threshold (fallback if floor estimate is wrong)
    var absoluteMaxHeight: Float = 2.5  // Ignore anything above 2.5m absolute
    
    /// Estimated floor height (updated dynamically)
    var floorHeight: Float = 0.0
    
    /// Whether floor height has been initialized from device position
    private(set) var floorInitialized: Bool = false
    
    /// Sampling rate for mesh vertices (skip every N vertices for performance)
    var vertexSamplingRate: Int = 4  // Process every 4th vertex for better performance
    
    // MARK: - State
    
    private weak var occupancyGrid: OccupancyGrid?
    
    /// Track floor height estimates
    private var floorHeightSamples: [Float] = []
    
    /// Generation counter to discard stale mesh data after reset
    private var generation: Int = 0
    
    /// Lock for thread-safe access to floor samples and generation
    private let lock = NSLock()
    
    // MARK: - Initialization
    
    init(occupancyGrid: OccupancyGrid) {
        self.occupancyGrid = occupancyGrid
    }
    
    /// Increment generation to invalidate any in-flight mesh processing and reset floor estimation
    func reset() {
        lock.lock()
        generation += 1
        floorHeightSamples.removeAll()
        floorHeight = 0.0
        floorInitialized = false
        lock.unlock()
    }
    
    /// Get current generation (call on main thread before async processing)
    func currentGeneration() -> Int {
        lock.lock()
        defer { lock.unlock() }
        return generation
    }
    
    // MARK: - Data Extraction (call on main thread to avoid retaining ARFrame)
    
    /// Extract vertex data from mesh anchor synchronously (call before async processing)
    func extractMeshData(from anchor: ARMeshAnchor, updateId: UInt64) -> ExtractedMeshData {
        let geometry = anchor.geometry
        let vertexSource = geometry.vertices
        let vertexCount = vertexSource.count
        let faceCount = geometry.faces.count
        
        // Build per-vertex classification from face classifications
        var vertexClassifications = Array(repeating: MeshClassification.none, count: vertexCount)
        
        if let classificationSource = geometry.classification {
            let classBuffer = classificationSource.buffer.contents()
            let classStride = classificationSource.stride
            let classOffset = classificationSource.offset
            let facesElement = geometry.faces
            let facesBuffer = facesElement.buffer.contents()
            let bytesPerIndex = facesElement.bytesPerIndex
            let indicesPerPrimitive = facesElement.indexCountPerPrimitive
            
            for faceIndex in 0..<faceCount {
                // Read classification for this face
                let classPtr = classBuffer.advanced(by: classOffset + classStride * faceIndex)
                let classValue = classPtr.assumingMemoryBound(to: UInt8.self).pointee
                let classification = MeshClassification.from(arClassification: Int(classValue))
                
                // Assign classification to each vertex of this face
                for j in 0..<indicesPerPrimitive {
                    let indexPtr = facesBuffer.advanced(by: (faceIndex * indicesPerPrimitive + j) * bytesPerIndex)
                    let vertexIndex: Int
                    if bytesPerIndex == 4 {
                        vertexIndex = Int(indexPtr.assumingMemoryBound(to: Int32.self).pointee)
                    } else {
                        vertexIndex = Int(indexPtr.assumingMemoryBound(to: Int16.self).pointee)
                    }
                    if vertexIndex < vertexCount {
                        // Prefer more specific classification
                        if classification != .none {
                            vertexClassifications[vertexIndex] = classification
                        }
                    }
                }
            }
        }
        
        // Extract sampled vertices and their classifications
        var vertices: [simd_float3] = []
        var classifications: [MeshClassification] = []
        vertices.reserveCapacity(vertexCount / vertexSamplingRate)
        classifications.reserveCapacity(vertexCount / vertexSamplingRate)
        
        let buffer = vertexSource.buffer.contents()
        let vertexStride = vertexSource.stride
        let vertexOffset = vertexSource.offset
        
        for i in stride(from: 0, to: vertexCount, by: vertexSamplingRate) {
            let ptr = buffer.advanced(by: vertexOffset + vertexStride * i)
            let vertex = ptr.assumingMemoryBound(to: simd_float3.self).pointee
            vertices.append(vertex)
            classifications.append(vertexClassifications[i])
        }
        
        return ExtractedMeshData(vertices: vertices, classifications: classifications, transform: anchor.transform, generation: currentGeneration(), updateId: updateId, anchorId: anchor.identifier)
    }
    
    // MARK: - Processing (can be called on background thread)
    
    /// Process extracted mesh data and update the occupancy grid
    func processExtractedMesh(_ meshData: ExtractedMeshData) {
        // Check if this mesh data is stale (from before a reset)
        if meshData.generation != currentGeneration() {
            return  // Discard stale mesh data
        }
        
        guard let grid = occupancyGrid else { return }
        
        let transform = meshData.transform
        
        // Collect points to mark as obstacles (with height and classification)
        var obstaclePoints: [(x: Float, y: Float, height: Float, classification: MeshClassification)] = []
        var floorPoints: [(x: Float, y: Float, classification: MeshClassification)] = []
        obstaclePoints.reserveCapacity(meshData.vertices.count)
        floorPoints.reserveCapacity(meshData.vertices.count)
        
        // Process vertices
        for (index, vertex) in meshData.vertices.enumerated() {
            let classification = meshData.classifications[index]
            
            // Skip ceiling-classified vertices entirely
            if classification == .ceiling {
                continue
            }
            
            // Transform vertex to world coordinates
            let localPos = simd_float4(vertex.x, vertex.y, vertex.z, 1.0)
            let worldPos = transform * localPos
            
            let worldX = worldPos.x
            let worldY = worldPos.y  // Height in ARKit
            let worldZ = worldPos.z
            
            // Update floor height estimate (look for lowest points)
            updateFloorEstimate(height: worldY)
            
            // Skip points that are absolutely too high (ceiling)
            if worldY > absoluteMaxHeight {
                continue
            }
            
            // Check if this is an obstacle (above floor, below ceiling)
            let heightAboveFloor = worldY - floorHeight
            
            if heightAboveFloor > minObstacleHeight && heightAboveFloor < maxObstacleHeight {
                // This is an obstacle - add to grid with height and classification info
                // Use X and Z as the 2D coordinates (Y is up in ARKit)
                obstaclePoints.append((worldX, worldZ, heightAboveFloor, classification))
            } else if heightAboveFloor >= -0.05 && heightAboveFloor <= minObstacleHeight {
                // This is floor level - mark as free/floor
                floorPoints.append((worldX, worldZ, classification == .none ? .floor : classification))
            }
        }
        
        // Atomically clear old cells for this anchor and write new data
        grid.replaceAnchorData(
            anchorId: meshData.anchorId,
            obstaclePoints: obstaclePoints,
            floorPoints: floorPoints,
            updateId: meshData.updateId
        )
    }
    
    /// Process a mesh anchor and update the occupancy grid (legacy - retains anchor)
    @available(*, deprecated, message: "Use extractMeshData + processExtractedMesh instead")
    func processMeshAnchor(_ anchor: ARMeshAnchor) {
        let meshData = extractMeshData(from: anchor, updateId: 0)
        processExtractedMesh(meshData)
    }
    
    // MARK: - Floor Detection
    
    private func updateFloorEstimate(height: Float) {
        lock.lock()
        defer { lock.unlock() }
        
        // Keep track of low points to estimate floor
        if floorHeightSamples.count < 1000 {
            floorHeightSamples.append(height)
        } else {
            // Replace random sample
            let index = Int.random(in: 0..<floorHeightSamples.count)
            floorHeightSamples[index] = height
        }
        
        // Initialize floor estimate early with just 20 samples
        if !floorInitialized && floorHeightSamples.count >= 20 {
            let sorted = floorHeightSamples.sorted()
            let percentileIndex = max(0, sorted.count / 20)  // 5%
            floorHeight = sorted[percentileIndex]
            floorInitialized = true
        }
        // Update floor estimate periodically after initialization
        else if floorInitialized && floorHeightSamples.count % 100 == 0 {
            // Use 5th percentile as floor estimate
            let sorted = floorHeightSamples.sorted()
            let percentileIndex = sorted.count / 20  // 5%
            floorHeight = sorted[percentileIndex]
        }
    }
    
    // MARK: - Telemetry Snapshot Extraction

    /// Extract a telemetry-ready snapshot from an ARMeshAnchor.
    /// Call on the main thread while the anchor is still valid.
    /// Returns nil if the geometry buffers have been invalidated.
    static func extractAnchorSnapshot(
        from anchor: ARMeshAnchor,
        generation: Int,
        frame: ARFrame? = nil,
        projectionOrientation: UIInterfaceOrientation = .landscapeRight
    ) -> MeshAnchorSnapshot? {
        let geometry = anchor.geometry
        let vertexSource = geometry.vertices
        let vertexCount = vertexSource.count
        let faceCount = geometry.faces.count

        // Guard against empty or invalidated geometry
        guard vertexCount > 0, faceCount > 0 else { return nil }

        // Validate vertex buffer bounds before accessing raw memory.
        // Use 3 * Float (12 bytes) not MemoryLayout<simd_float3>.size (16 — SIMD3 is padded).
        let vStride = vertexSource.stride
        let vOffset = vertexSource.offset
        let vBufLength = vertexSource.buffer.length
        let floatSize = 3 * MemoryLayout<Float>.size  // 12 bytes per vertex
        let requiredVertexBytes = vOffset + vStride * max(vertexCount - 1, 0) + floatSize
        guard vBufLength >= requiredVertexBytes else { return nil }

        // Validate face/index buffer bounds
        let facesElement = geometry.faces
        let indicesPerPrimitive = facesElement.indexCountPerPrimitive
        let bytesPerIndex = facesElement.bytesPerIndex
        let fBufLength = facesElement.buffer.length
        let totalIndices = faceCount * indicesPerPrimitive
        let requiredFaceBytes = totalIndices * bytesPerIndex
        guard fBufLength >= requiredFaceBytes else { return nil }

        // Vertices: flatten [x,y,z, x,y,z, ...] as Float array
        var floats: [Float] = []
        floats.reserveCapacity(vertexCount * 3)
        let vBuf = vertexSource.buffer.contents()
        for i in 0..<vertexCount {
            let ptr = vBuf.advanced(by: vOffset + vStride * i)
            let v = ptr.assumingMemoryBound(to: simd_float3.self).pointee
            floats.append(v.x)
            floats.append(v.y)
            floats.append(v.z)
        }
        let verticesB64 = floatArrayToBase64(floats)
        pruneVertexColorCache(keepingGeneration: generation)
        let vertexColorsB64 = sampleVertexColors(
            cacheKey: "\(generation):\(anchor.identifier.uuidString)",
            localVertices: floats,
            anchorTransform: anchor.transform,
            frame: frame,
            orientation: projectionOrientation
        )

        // Indices: UInt32 triangle indices
        let fBuf = facesElement.buffer.contents()
        var indices: [UInt32] = []
        indices.reserveCapacity(totalIndices)
        for i in 0..<totalIndices {
            let ptr = fBuf.advanced(by: i * bytesPerIndex)
            if bytesPerIndex == 4 {
                indices.append(UInt32(bitPattern: ptr.assumingMemoryBound(to: Int32.self).pointee))
            } else {
                indices.append(UInt32(ptr.assumingMemoryBound(to: UInt16.self).pointee))
            }
        }
        let indicesB64 = uint32ArrayToBase64(indices)

        // Per-face classifications
        var classValues: [UInt8] = []
        classValues.reserveCapacity(faceCount)
        if let classSource = geometry.classification {
            let cBuf = classSource.buffer.contents()
            let cStride = classSource.stride
            let cOffset = classSource.offset
            let cBufLength = classSource.buffer.length
            let requiredClassBytes = cOffset + cStride * (faceCount - 1) + 1
            if cBufLength >= requiredClassBytes {
                for i in 0..<faceCount {
                    classValues.append(cBuf.advanced(by: cOffset + cStride * i).assumingMemoryBound(to: UInt8.self).pointee)
                }
            } else {
                classValues = Array(repeating: 0, count: faceCount)
            }
        } else {
            classValues = Array(repeating: 0, count: faceCount)
        }
        let classificationsB64 = uint8ArrayToBase64(classValues)

        // Transform: column-major 16 floats
        let m = anchor.transform
        let transform: [Float] = [
            m.columns.0.x, m.columns.0.y, m.columns.0.z, m.columns.0.w,
            m.columns.1.x, m.columns.1.y, m.columns.1.z, m.columns.1.w,
            m.columns.2.x, m.columns.2.y, m.columns.2.z, m.columns.2.w,
            m.columns.3.x, m.columns.3.y, m.columns.3.z, m.columns.3.w,
        ]

        return MeshAnchorSnapshot(
            anchorId: anchor.identifier.uuidString,
            transform: transform,
            verticesB64: verticesB64,
            vertexCount: vertexCount,
            indicesB64: indicesB64,
            triangleCount: faceCount,
            classificationsB64: classificationsB64,
            vertexColorsB64: vertexColorsB64,
            generation: generation
        )
    }

    private static func sampleVertexColors(
        cacheKey: String,
        localVertices: [Float],
        anchorTransform: simd_float4x4,
        frame: ARFrame?,
        orientation: UIInterfaceOrientation
    ) -> String? {
        let vertexCount = localVertices.count / 3
        let requiredColorBytes = vertexCount * 4
        let cachedColors = vertexColorCache[cacheKey]
        let hasCachedColors = cachedColors != nil && !cachedColors!.isEmpty

        guard let frame,
              vertexCount > 0,
              CVPixelBufferGetPlaneCount(frame.capturedImage) >= 2 else {
            return hasCachedColors ? uint8ArrayToBase64(Array(cachedColors!.prefix(requiredColorBytes))) : nil
        }

        let pixelBuffer = frame.capturedImage
        guard let depthMap = frame.smoothedSceneDepth?.depthMap ?? frame.sceneDepth?.depthMap else {
            return hasCachedColors ? uint8ArrayToBase64(Array(cachedColors!.prefix(requiredColorBytes))) : nil
        }

        let viewportSize = frame.camera.imageResolution
        guard viewportSize.width > 0, viewportSize.height > 0 else { return nil }

        CVPixelBufferLockBaseAddress(pixelBuffer, .readOnly)
        CVPixelBufferLockBaseAddress(depthMap, .readOnly)
        defer {
            CVPixelBufferUnlockBaseAddress(depthMap, .readOnly)
            CVPixelBufferUnlockBaseAddress(pixelBuffer, .readOnly)
        }

        guard let yBase = CVPixelBufferGetBaseAddressOfPlane(pixelBuffer, 0),
              let cbcrBase = CVPixelBufferGetBaseAddressOfPlane(pixelBuffer, 1) else { return nil }

        let imageWidth = CVPixelBufferGetWidthOfPlane(pixelBuffer, 0)
        let imageHeight = CVPixelBufferGetHeightOfPlane(pixelBuffer, 0)
        let yBytesPerRow = CVPixelBufferGetBytesPerRowOfPlane(pixelBuffer, 0)
        let cbcrWidth = CVPixelBufferGetWidthOfPlane(pixelBuffer, 1)
        let cbcrHeight = CVPixelBufferGetHeightOfPlane(pixelBuffer, 1)
        let cbcrBytesPerRow = CVPixelBufferGetBytesPerRowOfPlane(pixelBuffer, 1)
        let depthWidth = CVPixelBufferGetWidth(depthMap)
        let depthHeight = CVPixelBufferGetHeight(depthMap)
        let depthBytesPerRow = CVPixelBufferGetBytesPerRow(depthMap)
        guard let depthBase = CVPixelBufferGetBaseAddress(depthMap),
              depthWidth > 0,
              depthHeight > 0 else { return nil }

        var colors: [UInt8]
        if let cached = cachedColors, cached.count == requiredColorBytes {
            // Same vertex count — reuse accumulated colors directly
            colors = cached
        } else if let cached = cachedColors, !cached.isEmpty {
            // Vertex count changed (mesh refined); preserve old colors, pad/trim as needed
            colors = [UInt8](repeating: 180, count: requiredColorBytes)
            for alphaIndex in stride(from: 3, to: colors.count, by: 4) { colors[alphaIndex] = 255 }
            let copyCount = min(cached.count, requiredColorBytes)
            colors.replaceSubrange(0..<copyCount, with: cached.prefix(copyCount))
        } else {
            colors = [UInt8](repeating: 180, count: requiredColorBytes)
            for alphaIndex in stride(from: 3, to: colors.count, by: 4) { colors[alphaIndex] = 255 }
        }

        let worldToCamera = frame.camera.transform.inverse
        var sampledCount = 0

        for vertexIndex in 0..<vertexCount {
            let local = simd_float4(
                localVertices[vertexIndex * 3],
                localVertices[vertexIndex * 3 + 1],
                localVertices[vertexIndex * 3 + 2],
                1
            )
            let world4 = anchorTransform * local
            let world = simd_float3(world4.x, world4.y, world4.z)
            let cameraSpace = worldToCamera * world4
            let cameraDepth = -cameraSpace.z
            guard cameraDepth > 0.05 else { continue }

            let point = frame.camera.projectPoint(world, orientation: orientation, viewportSize: viewportSize)
            let px = Int(point.x.rounded())
            let py = Int(point.y.rounded())
            let out = vertexIndex * 4

            guard px >= 0, py >= 0, px < imageWidth, py < imageHeight else { continue }
            guard depthMatchesVertex(
                pixelX: px,
                pixelY: py,
                imageWidth: imageWidth,
                imageHeight: imageHeight,
                cameraDepth: cameraDepth,
                depthBase: depthBase,
                depthWidth: depthWidth,
                depthHeight: depthHeight,
                depthBytesPerRow: depthBytesPerRow
            ) else { continue }

            let y = yBase.advanced(by: py * yBytesPerRow + px).assumingMemoryBound(to: UInt8.self).pointee
            let cbcrX = min(max(px / 2, 0), cbcrWidth - 1)
            let cbcrY = min(max(py / 2, 0), cbcrHeight - 1)
            let cbcrOffset = cbcrY * cbcrBytesPerRow + cbcrX * 2
            let cb = cbcrBase.advanced(by: cbcrOffset).assumingMemoryBound(to: UInt8.self).pointee
            let cr = cbcrBase.advanced(by: cbcrOffset + 1).assumingMemoryBound(to: UInt8.self).pointee
            let rgb = ycbcrToRGB(y: y, cb: cb, cr: cr)

            colors[out] = rgb.r
            colors[out + 1] = rgb.g
            colors[out + 2] = rgb.b
            colors[out + 3] = 255
            sampledCount += 1
        }

        guard sampledCount > 0 || hasCachedColors else { return nil }
        vertexColorCache[cacheKey] = colors
        return uint8ArrayToBase64(colors)
    }

    private static func pruneVertexColorCache(keepingGeneration generation: Int) {
        let prefix = "\(generation):"
        vertexColorCache = vertexColorCache.filter { $0.key.hasPrefix(prefix) }
    }

    private static func depthMatchesVertex(
        pixelX: Int,
        pixelY: Int,
        imageWidth: Int,
        imageHeight: Int,
        cameraDepth: Float,
        depthBase: UnsafeMutableRawPointer,
        depthWidth: Int,
        depthHeight: Int,
        depthBytesPerRow: Int
    ) -> Bool {
        let depthX = min(max(Int((Float(pixelX) / Float(imageWidth)) * Float(depthWidth)), 0), depthWidth - 1)
        let depthY = min(max(Int((Float(pixelY) / Float(imageHeight)) * Float(depthHeight)), 0), depthHeight - 1)
        let depth = depthBase.advanced(by: depthY * depthBytesPerRow + depthX * MemoryLayout<Float32>.size)
            .assumingMemoryBound(to: Float32.self)
            .pointee

        guard depth.isFinite, depth > 0 else { return false }
        let tolerance = max(0.08, cameraDepth * 0.06)
        return abs(Float(depth) - cameraDepth) <= tolerance
    }

    private static func ycbcrToRGB(y: UInt8, cb: UInt8, cr: UInt8) -> (r: UInt8, g: UInt8, b: UInt8) {
        let yf = Float(y)
        let cbf = Float(cb) - 128
        let crf = Float(cr) - 128
        let r = yf + 1.4020 * crf
        let g = yf - 0.3441 * cbf - 0.7141 * crf
        let b = yf + 1.7720 * cbf
        return (clampToByte(r), clampToByte(g), clampToByte(b))
    }

    private static func clampToByte(_ value: Float) -> UInt8 {
        UInt8(max(0, min(255, Int(value.rounded()))))
    }

    // MARK: - Face Processing (for more accurate obstacles)
    
    /// Process mesh faces to get surface normals and better obstacle detection
    func processMeshFaces(_ anchor: ARMeshAnchor) {
        guard let grid = occupancyGrid else { return }
        
        let geometry = anchor.geometry
        let transform = anchor.transform
        
        let vertices = geometry.vertices
        let faces = geometry.faces
        let faceCount = faces.count
        
        var obstaclePoints: [(x: Float, y: Float)] = []
        
        // Process each face
        for i in stride(from: 0, to: faceCount, by: 3) {  // Sample every 3rd face
            let faceIndices = faces[i]
            
            // Get the three vertices of the triangle
            let v0 = vertices[Int(faceIndices[0])]
            let v1 = vertices[Int(faceIndices[1])]
            let v2 = vertices[Int(faceIndices[2])]
            
            // Calculate face center
            let localCenter = simd_float4(
                (v0.x + v1.x + v2.x) / 3,
                (v0.y + v1.y + v2.y) / 3,
                (v0.z + v1.z + v2.z) / 3,
                1.0
            )
            let worldCenter = transform * localCenter
            
            // Calculate face normal
            let edge1 = simd_float3(v1.x - v0.x, v1.y - v0.y, v1.z - v0.z)
            let edge2 = simd_float3(v2.x - v0.x, v2.y - v0.y, v2.z - v0.z)
            var normal = simd_cross(edge1, edge2)
            normal = simd_normalize(normal)
            
            // Transform normal to world space
            let worldNormal = simd_float3(
                transform.columns.0.x * normal.x + transform.columns.1.x * normal.y + transform.columns.2.x * normal.z,
                transform.columns.0.y * normal.x + transform.columns.1.y * normal.y + transform.columns.2.y * normal.z,
                transform.columns.0.z * normal.x + transform.columns.1.z * normal.y + transform.columns.2.z * normal.z
            )
            
            let heightAboveFloor = worldCenter.y - floorHeight
            
            // Check if this face is a vertical surface (obstacle) or horizontal (floor/ceiling)
            let verticalness = abs(worldNormal.y)  // 0 = vertical wall, 1 = horizontal floor
            
            // Mark vertical surfaces as obstacles
            if verticalness < 0.7 && heightAboveFloor > minObstacleHeight && heightAboveFloor < maxObstacleHeight {
                obstaclePoints.append((worldCenter.x, worldCenter.z))
            }
        }
        
        if !obstaclePoints.isEmpty {
            grid.markOccupiedBatch(obstaclePoints)
        }
    }
}

// MARK: - ARGeometrySource Extension

extension ARGeometrySource {
    subscript(index: Int) -> SIMD3<Float> {
        let pointer = buffer.contents().advanced(by: offset + stride * index)
        return pointer.assumingMemoryBound(to: SIMD3<Float>.self).pointee
    }
}

// MARK: - ARGeometryElement Extension

extension ARGeometryElement {
    subscript(index: Int) -> [Int32] {
        let pointer = buffer.contents().advanced(by: index * bytesPerIndex * indexCountPerPrimitive)
        var indices: [Int32] = []
        
        for i in 0..<indexCountPerPrimitive {
            if bytesPerIndex == 4 {
                let value = pointer.advanced(by: i * 4).assumingMemoryBound(to: Int32.self).pointee
                indices.append(value)
            } else {
                let value = pointer.advanced(by: i * 2).assumingMemoryBound(to: Int16.self).pointee
                indices.append(Int32(value))
            }
        }
        
        return indices
    }
}
