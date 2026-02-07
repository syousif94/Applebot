//
//  MeshProcessor.swift
//  RoboCar
//
//  Created by Sammy Yousif on 1/3/26.
//

import ARKit
import simd

/// Extracted mesh data that can be processed without retaining ARMeshAnchor
struct ExtractedMeshData {
    let vertices: [simd_float3]
    let classifications: [MeshClassification]
    let transform: simd_float4x4
    let generation: Int  // Generation counter to detect stale data after reset
}

/// Processes ARMeshAnchor data and updates the occupancy grid
class MeshProcessor {
    
    // MARK: - Configuration
    
    /// Height threshold for obstacles (meters above floor)
    var minObstacleHeight: Float = 0.1  // 10cm minimum
    var maxObstacleHeight: Float = 1.8  // 1.8m maximum (ignore ceiling - typical door height)
    
    /// Absolute ceiling height threshold (fallback if floor estimate is wrong)
    var absoluteMaxHeight: Float = 2.5  // Ignore anything above 2.5m absolute
    
    /// Estimated floor height (updated dynamically)
    var floorHeight: Float = 0.0
    
    /// Whether floor height has been initialized from device position
    private var floorInitialized: Bool = false
    
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
    func extractMeshData(from anchor: ARMeshAnchor) -> ExtractedMeshData {
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
        
        return ExtractedMeshData(vertices: vertices, classifications: classifications, transform: anchor.transform, generation: currentGeneration())
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
        
        // Batch update the grid - floor first, then obstacles on top
        if !floorPoints.isEmpty {
            grid.markFreeBatchWithClassification(floorPoints)
        }
        if !obstaclePoints.isEmpty {
            grid.markOccupiedBatchWithClassification(obstaclePoints)
        }
    }
    
    /// Process a mesh anchor and update the occupancy grid (legacy - retains anchor)
    @available(*, deprecated, message: "Use extractMeshData + processExtractedMesh instead")
    func processMeshAnchor(_ anchor: ARMeshAnchor) {
        let meshData = extractMeshData(from: anchor)
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
