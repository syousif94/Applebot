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
    let transform: simd_float4x4
}

/// Processes ARMeshAnchor data and updates the occupancy grid
class MeshProcessor {
    
    // MARK: - Configuration
    
    /// Height threshold for obstacles (meters above floor)
    var minObstacleHeight: Float = 0.1  // 10cm minimum
    var maxObstacleHeight: Float = 2.0  // 2m maximum (ignore ceiling)
    
    /// Estimated floor height (updated dynamically)
    var floorHeight: Float = 0.0
    
    /// Sampling rate for mesh vertices (skip every N vertices for performance)
    var vertexSamplingRate: Int = 4  // Process every 4th vertex for better performance
    
    // MARK: - State
    
    private weak var occupancyGrid: OccupancyGrid?
    
    /// Track floor height estimates
    private var floorHeightSamples: [Float] = []
    
    /// Lock for thread-safe access to floor samples
    private let lock = NSLock()
    
    // MARK: - Initialization
    
    init(occupancyGrid: OccupancyGrid) {
        self.occupancyGrid = occupancyGrid
    }
    
    // MARK: - Data Extraction (call on main thread to avoid retaining ARFrame)
    
    /// Extract vertex data from mesh anchor synchronously (call before async processing)
    func extractMeshData(from anchor: ARMeshAnchor) -> ExtractedMeshData {
        let geometry = anchor.geometry
        let vertexSource = geometry.vertices
        let vertexCount = vertexSource.count
        
        // Extract sampled vertices
        var vertices: [simd_float3] = []
        vertices.reserveCapacity(vertexCount / vertexSamplingRate)
        
        let buffer = vertexSource.buffer.contents()
        let vertexStride = vertexSource.stride
        let vertexOffset = vertexSource.offset
        
        for i in stride(from: 0, to: vertexCount, by: vertexSamplingRate) {
            let ptr = buffer.advanced(by: vertexOffset + vertexStride * i)
            let vertex = ptr.assumingMemoryBound(to: simd_float3.self).pointee
            vertices.append(vertex)
        }
        
        return ExtractedMeshData(vertices: vertices, transform: anchor.transform)
    }
    
    // MARK: - Processing (can be called on background thread)
    
    /// Process extracted mesh data and update the occupancy grid
    func processExtractedMesh(_ meshData: ExtractedMeshData) {
        guard let grid = occupancyGrid else { return }
        
        let transform = meshData.transform
        
        // Collect points to mark as obstacles (with height)
        var obstaclePoints: [(x: Float, y: Float, height: Float)] = []
        var floorPoints: [(x: Float, y: Float)] = []
        obstaclePoints.reserveCapacity(meshData.vertices.count)
        floorPoints.reserveCapacity(meshData.vertices.count)
        
        // Process vertices
        for vertex in meshData.vertices {
            // Transform vertex to world coordinates
            let localPos = simd_float4(vertex.x, vertex.y, vertex.z, 1.0)
            let worldPos = transform * localPos
            
            let worldX = worldPos.x
            let worldY = worldPos.y  // Height in ARKit
            let worldZ = worldPos.z
            
            // Update floor height estimate (look for lowest points)
            updateFloorEstimate(height: worldY)
            
            // Check if this is an obstacle (above floor, below ceiling)
            let heightAboveFloor = worldY - floorHeight
            
            if heightAboveFloor > minObstacleHeight && heightAboveFloor < maxObstacleHeight {
                // This is an obstacle - add to grid with height info
                // Use X and Z as the 2D coordinates (Y is up in ARKit)
                obstaclePoints.append((worldX, worldZ, heightAboveFloor))
            } else if heightAboveFloor >= -0.05 && heightAboveFloor <= minObstacleHeight {
                // This is floor level - mark as free/floor
                floorPoints.append((worldX, worldZ))
            }
        }
        
        // Batch update the grid - floor first, then obstacles on top
        if !floorPoints.isEmpty {
            grid.markFreeBatch(floorPoints)
        }
        if !obstaclePoints.isEmpty {
            grid.markOccupiedBatchWithHeights(obstaclePoints)
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
        
        // Update floor estimate periodically
        if floorHeightSamples.count >= 100 && floorHeightSamples.count % 100 == 0 {
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
