//
//  MeshSimplifier.swift
//  RoboCar
//
//  Created by Sammy Yousif on 1/3/26.
//

import ARKit
import Accelerate
import simd

/// High-performance mesh simplifier using SIMD and Accelerate
class MeshSimplifier {
    
    // MARK: - Configuration
    
    /// Grid cell size for vertex clustering (meters)
    let clusterSize: Float
    
    /// Minimum edge length to keep a triangle (meters)
    let minEdgeLength: Float
    
    // MARK: - Initialization
    
    init(clusterSize: Float = 0.03, minEdgeLength: Float = 0.02) {
        self.clusterSize = clusterSize
        self.minEdgeLength = minEdgeLength
    }
    
    // MARK: - Simplification
    
    /// Simplify mesh geometry using spatial vertex clustering
    /// Returns simplified vertices and triangle indices
    func simplify(_ geometry: ARMeshGeometry, transform: simd_float4x4) -> (vertices: [simd_float3], indices: [UInt32])? {
        let vertexCount = geometry.vertices.count
        let faceCount = geometry.faces.count
        
        guard vertexCount > 0 && faceCount > 0 else { return nil }
        
        // Step 1: Extract and transform vertices using SIMD
        let worldVertices = extractAndTransformVertices(geometry.vertices, transform: transform)
        
        // Step 2: Cluster vertices using spatial hashing
        let (clusterMap, clusterCenters) = clusterVertices(worldVertices)
        
        // Step 3: Rebuild triangles with clustered vertices, filtering degenerate ones
        let indices = rebuildTriangles(geometry.faces, clusterMap: clusterMap, clusterCount: clusterCenters.count)
        
        return (clusterCenters, indices)
    }
    
    /// Create SCNGeometry from simplified mesh
    func createGeometry(vertices: [simd_float3], indices: [UInt32]) -> SCNGeometry {
        // Create vertex data
        let vertexData = vertices.withUnsafeBytes { Data($0) }
        let vertexSource = SCNGeometrySource(
            data: vertexData,
            semantic: .vertex,
            vectorCount: vertices.count,
            usesFloatComponents: true,
            componentsPerVector: 3,
            bytesPerComponent: MemoryLayout<Float>.size,
            dataOffset: 0,
            dataStride: MemoryLayout<simd_float3>.stride
        )
        
        // Create index data
        let indexData = indices.withUnsafeBytes { Data($0) }
        let element = SCNGeometryElement(
            data: indexData,
            primitiveType: .triangles,
            primitiveCount: indices.count / 3,
            bytesPerIndex: MemoryLayout<UInt32>.size
        )
        
        return SCNGeometry(sources: [vertexSource], elements: [element])
    }
    
    // MARK: - SIMD Vertex Extraction & Transform
    
    private func extractAndTransformVertices(_ source: ARGeometrySource, transform: simd_float4x4) -> [simd_float3] {
        let count = source.count
        var result = [simd_float3](repeating: .zero, count: count)
        
        let buffer = source.buffer.contents()
        let stride = source.stride
        let offset = source.offset
        
        // Process vertices in batches using SIMD
        result.withUnsafeMutableBufferPointer { resultPtr in
            for i in 0..<count {
                let vertexPtr = buffer.advanced(by: offset + stride * i)
                let vertex = vertexPtr.assumingMemoryBound(to: simd_float3.self).pointee
                
                // Transform to world coordinates using SIMD matrix multiply
                let local = simd_float4(vertex.x, vertex.y, vertex.z, 1.0)
                let world = transform * local
                resultPtr[i] = simd_float3(world.x, world.y, world.z)
            }
        }
        
        return result
    }
    
    // MARK: - Spatial Vertex Clustering
    
    /// Cluster vertices into grid cells and return mapping + cluster centers
    private func clusterVertices(_ vertices: [simd_float3]) -> (map: [Int], centers: [simd_float3]) {
        let count = vertices.count
        guard count > 0 else { return ([], []) }
        
        // Find bounding box using Accelerate
        var minBound = simd_float3(repeating: .greatestFiniteMagnitude)
        var maxBound = simd_float3(repeating: -.greatestFiniteMagnitude)
        
        for v in vertices {
            minBound = simd_min(minBound, v)
            maxBound = simd_max(maxBound, v)
        }
        
        let invClusterSize = 1.0 / clusterSize
        
        // Spatial hash map: hash -> (cluster index, sum of positions, count)
        var hashToCluster: [Int: Int] = [:]
        hashToCluster.reserveCapacity(count / 4)
        
        var clusterSums: [simd_float3] = []
        var clusterCounts: [Int] = []
        clusterSums.reserveCapacity(count / 4)
        clusterCounts.reserveCapacity(count / 4)
        
        var vertexToCluster = [Int](repeating: 0, count: count)
        
        for (i, v) in vertices.enumerated() {
            // Compute grid cell coordinates
            let cell = simd_int3(
                Int32((v.x - minBound.x) * invClusterSize),
                Int32((v.y - minBound.y) * invClusterSize),
                Int32((v.z - minBound.z) * invClusterSize)
            )
            
            // Spatial hash
            let hash = spatialHash(cell)
            
            if let clusterIdx = hashToCluster[hash] {
                // Add to existing cluster
                vertexToCluster[i] = clusterIdx
                clusterSums[clusterIdx] += v
                clusterCounts[clusterIdx] += 1
            } else {
                // Create new cluster
                let newIdx = clusterSums.count
                hashToCluster[hash] = newIdx
                vertexToCluster[i] = newIdx
                clusterSums.append(v)
                clusterCounts.append(1)
            }
        }
        
        // Compute cluster centers (average of vertices in each cluster)
        var centers = [simd_float3](repeating: .zero, count: clusterSums.count)
        for i in 0..<clusterSums.count {
            centers[i] = clusterSums[i] / Float(clusterCounts[i])
        }
        
        return (vertexToCluster, centers)
    }
    
    /// Fast spatial hash function
    @inline(__always)
    private func spatialHash(_ cell: simd_int3) -> Int {
        // Large primes for better distribution
        let p1: Int = 73856093
        let p2: Int = 19349663
        let p3: Int = 83492791
        return Int(cell.x) &* p1 ^ Int(cell.y) &* p2 ^ Int(cell.z) &* p3
    }
    
    // MARK: - Triangle Rebuilding
    
    private func rebuildTriangles(_ faces: ARGeometryElement, clusterMap: [Int], clusterCount: Int) -> [UInt32] {
        let faceCount = faces.count
        let bytesPerIndex = faces.bytesPerIndex
        let buffer = faces.buffer.contents()
        
        var indices: [UInt32] = []
        indices.reserveCapacity(faceCount * 3)
        
        // Track which triangles we've already added (avoid duplicates)
        var addedTriangles = Set<Int>()
        addedTriangles.reserveCapacity(faceCount)
        
        for i in 0..<faceCount {
            let baseOffset = i * 3 * bytesPerIndex
            
            // Read original vertex indices
            let i0: Int
            let i1: Int
            let i2: Int
            
            if bytesPerIndex == 4 {
                i0 = Int(buffer.advanced(by: baseOffset).assumingMemoryBound(to: UInt32.self).pointee)
                i1 = Int(buffer.advanced(by: baseOffset + 4).assumingMemoryBound(to: UInt32.self).pointee)
                i2 = Int(buffer.advanced(by: baseOffset + 8).assumingMemoryBound(to: UInt32.self).pointee)
            } else {
                i0 = Int(buffer.advanced(by: baseOffset).assumingMemoryBound(to: UInt16.self).pointee)
                i1 = Int(buffer.advanced(by: baseOffset + 2).assumingMemoryBound(to: UInt16.self).pointee)
                i2 = Int(buffer.advanced(by: baseOffset + 4).assumingMemoryBound(to: UInt16.self).pointee)
            }
            
            // Map to cluster indices
            guard i0 < clusterMap.count && i1 < clusterMap.count && i2 < clusterMap.count else { continue }
            
            let c0 = clusterMap[i0]
            let c1 = clusterMap[i1]
            let c2 = clusterMap[i2]
            
            // Skip degenerate triangles (where vertices collapsed to same cluster)
            if c0 == c1 || c1 == c2 || c2 == c0 {
                continue
            }
            
            // Create canonical triangle hash to avoid duplicates
            let sorted = [c0, c1, c2].sorted()
            let triHash = sorted[0] &* 1000003 ^ sorted[1] &* 1009 ^ sorted[2]
            
            if addedTriangles.contains(triHash) {
                continue
            }
            addedTriangles.insert(triHash)
            
            indices.append(UInt32(c0))
            indices.append(UInt32(c1))
            indices.append(UInt32(c2))
        }
        
        return indices
    }
}

// MARK: - Accelerate-based Batch Transform

extension MeshSimplifier {
    
    /// Transform vertices in batch using vDSP (for very large meshes)
    func batchTransformVertices(_ vertices: UnsafePointer<Float>, count: Int, transform: simd_float4x4) -> [Float] {
        // Input: array of [x, y, z, x, y, z, ...] 
        // We need to apply 4x4 matrix transform to each (x,y,z,1) vector
        
        var result = [Float](repeating: 0, count: count * 3)
        
        // Extract transform columns for SIMD operations
        let col0 = transform.columns.0
        let col1 = transform.columns.1
        let col2 = transform.columns.2
        let col3 = transform.columns.3  // Translation
        
        // Process each vertex
        for i in stride(from: 0, to: count * 3, by: 3) {
            let x = vertices[i]
            let y = vertices[i + 1]
            let z = vertices[i + 2]
            
            // Matrix multiply: result = M * [x, y, z, 1]
            result[i]     = col0.x * x + col1.x * y + col2.x * z + col3.x
            result[i + 1] = col0.y * x + col1.y * y + col2.y * z + col3.y
            result[i + 2] = col0.z * x + col1.z * y + col2.z * z + col3.z
        }
        
        return result
    }
}
