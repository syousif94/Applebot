//
//  ObstacleDetector.swift
//  RoboCar
//
//  Created by Sammy Yousif on 2/17/26.
//

import Foundation
import simd
import ARKit

/// Info about a single nearby obstacle for display purposes
struct NearbyObstacle {
    let distance: Float               // meters
    let classification: MeshClassification
    let localDirection: simd_float2   // relative to device (XZ plane)
    let elevationDegrees: Int         // vertical angle: 0 = level, negative = below, positive = above
    let isDepthBased: Bool            // true if from LiDAR depth map, false if from mesh grid
    
    /// Horizontal angle in degrees from straight ahead (0°). Negative = left, positive = right.
    var angleDegrees: Int {
        Int(atan2f(localDirection.x, localDirection.y) * 180 / .pi)
    }
    
    /// Human-readable direction label
    var directionLabel: String {
        let angle = angleDegrees
        if angle > 60 { return "Right" }
        if angle < -60 { return "Left" }
        if angle > 20 { return "Front-Right" }
        if angle < -20 { return "Front-Left" }
        if localDirection.y < 0 { return "Behind" }
        return "Ahead"
    }
}

/// Checks the occupancy grid for obstacles within a protective radius around
/// the iPhone and prevents the car from driving toward them. Movement away
/// from obstacles is always allowed.
class ObstacleDetector {
    
    // MARK: - Singleton
    
    static let shared = ObstacleDetector()
    
    // MARK: - Configuration
    
    /// Protective radius in meters — any obstacle within this distance triggers avoidance
    var stopRadius: Float = 0.18  // 180mm
    
    /// Whether obstacle avoidance is enabled
    var isEnabled: Bool = true
    
    /// Floor height in world coordinates (updated from MeshProcessor each frame)
    var floorHeight: Float = 0.0
    
    /// Whether the floor height has been estimated (skip depth filtering until ready)
    var floorHeightEstimated: Bool = false
    
    /// Minimum height above floor for a valid depth obstacle (meters)
    var floorFilterMargin: Float = 0.08  // 8cm — ignore depth hits on/near the floor
    
    /// Maximum height above floor for a valid depth obstacle (meters)
    var ceilingFilterHeight: Float = 1.8  // ignore ceiling and above
    
    // MARK: - State
    
    /// Set by the LiDARViewController each frame
    weak var occupancyGrid: OccupancyGrid?
    
    /// Camera transform set each frame for computing device-relative elevation
    var cameraTransform: simd_float4x4 = matrix_identity_float4x4
    
    /// True when any obstacle is inside the protective radius
    private(set) var obstacleDetected: Bool = false
    
    /// Distance to the nearest obstacle (meters), or nil if none in range
    private(set) var nearestObstacleDistance: Float? = nil
    
    /// Unit vector pointing from the device TOWARD the centroid of nearby obstacles,
    /// expressed in **world coordinates**. Nil when no obstacles in range.
    /// Movement in this direction is blocked; movement opposite is allowed.
    private(set) var blockedWorldDirection: simd_float2? = nil
    
    /// Same direction expressed relative to the device heading:
    ///   x > 0 → obstacle is to the right,  x < 0 → left
    ///   y > 0 → obstacle is ahead,          y < 0 → behind
    private(set) var blockedLocalDirection: simd_float2? = nil
    
    /// Classification of the nearest obstacle (e.g. wall, table, seat)
    private(set) var nearestClassification: MeshClassification = .none
    
    /// All nearby obstacles within the stop radius, sorted by distance
    private(set) var nearbyObstacles: [NearbyObstacle] = []
    
    /// World-space (x, y) positions of all obstacle cells within the stop radius.
    /// Used by GridMapView to highlight obstacle cells in red.
    private(set) var obstacleWorldPositions: [(x: Float, y: Float)] = []
    
    /// Callback when obstacle state changes
    var onObstacleStateChanged: ((Bool, Float?) -> Void)?
    
    private init() {}
    
    // MARK: - Depth-Based Detection
    
    /// Nearest depth reading from the LiDAR depth map (meters), nil if none close
    private(set) var depthObstacleDistance: Float? = nil
    
    /// World position of the nearest depth obstacle (x=ARKit X, y=ARKit Z, z=ARKit Y height)
    private(set) var depthObstacleWorldPosition: (x: Float, y: Float, z: Float)? = nil
    
    /// Update proximity from the raw LiDAR depth map.
    /// This catches dynamic objects (hands, people, pets) that scene reconstruction ignores.
    /// Depth points near the floor or ceiling planes are discarded using world-space projection.
    /// Call every frame from the display link.
    func updateDepth(frame: ARFrame) {
        guard isEnabled else { return }
        guard let sceneDepth = frame.sceneDepth ?? frame.smoothedSceneDepth else {
            depthObstacleDistance = nil
            depthObstacleWorldPosition = nil
            return
        }
        
        let depthMap = sceneDepth.depthMap
        let width = CVPixelBufferGetWidth(depthMap)
        let height = CVPixelBufferGetHeight(depthMap)
        
        CVPixelBufferLockBaseAddress(depthMap, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(depthMap, .readOnly) }
        
        guard let baseAddress = CVPixelBufferGetBaseAddress(depthMap) else { return }
        let floatBuffer = baseAddress.assumingMemoryBound(to: Float32.self)
        let bytesPerRow = CVPixelBufferGetBytesPerRow(depthMap)
        let floatsPerRow = bytesPerRow / MemoryLayout<Float32>.size
        
        // Camera intrinsics scaled to depth map resolution for world-space projection
        let intrinsics = frame.camera.intrinsics
        let imageRes = frame.camera.imageResolution
        let scaleX = Float(width) / Float(imageRes.width)
        let scaleY = Float(height) / Float(imageRes.height)
        let fx = intrinsics[0][0] * scaleX
        let fy = intrinsics[1][1] * scaleY
        let cx = intrinsics[2][0] * scaleX
        let cy = intrinsics[2][1] * scaleY
        let camTransform = frame.camera.transform
        
        // Sample a grid of points across the depth map to find close obstacles
        let sampleStepX = max(1, width / 16)
        let sampleStepY = max(1, height / 16)
        
        var minDepth: Float = .greatestFiniteMagnitude
        var minPixelX = 0
        var minPixelY = 0
        
        for py in stride(from: 0, to: height, by: sampleStepY) {
            for px in stride(from: 0, to: width, by: sampleStepX) {
                let depth = floatBuffer[py * floatsPerRow + px]
                // Ignore invalid readings and very far objects
                guard depth > 0.01 && depth < stopRadius else { continue }
                
                // Project depth pixel to world space to check its height.
                // Intrinsics use (x-right, y-down, z-forward); camera.transform
                // uses (x-right, y-up, z-backward), so negate y and z.
                if floorHeightEstimated {
                    let intrinsicX = (Float(px) - cx) / fx * depth
                    let intrinsicY = (Float(py) - cy) / fy * depth
                    let camPoint = simd_float4(intrinsicX, -intrinsicY, -depth, 1.0)
                    let worldPoint = camTransform * camPoint
                    let heightAboveFloor = worldPoint.y - floorHeight
                    
                    // Discard points on or near the floor plane
                    if heightAboveFloor < floorFilterMargin {
                        continue
                    }
                    // Discard points at or above ceiling height
                    if heightAboveFloor > ceilingFilterHeight {
                        continue
                    }
                }
                
                if depth < minDepth {
                    minDepth = depth
                    minPixelX = px
                    minPixelY = py
                }
            }
        }
        
        guard minDepth < stopRadius else {
            if depthObstacleDistance != nil {
                depthObstacleDistance = nil
                depthObstacleWorldPosition = nil
            }
            return
        }
        
        depthObstacleDistance = minDepth
        
        // Project closest depth point to world space for angle calculation
        let intrinsicX_closest = (Float(minPixelX) - cx) / fx * minDepth
        let intrinsicY_closest = (Float(minPixelY) - cy) / fy * minDepth
        let camPoint_closest = simd_float4(intrinsicX_closest, -intrinsicY_closest, -minDepth, 1.0)
        let worldPoint = camTransform * camPoint_closest
        // x = ARKit X, y = ARKit Z (grid plane), z = ARKit Y (height)
        depthObstacleWorldPosition = (x: worldPoint.x, y: worldPoint.z, z: worldPoint.y)
    }
    
    // MARK: - 3D Point Collection for Overlay
    
    /// Threshold for considering a face "horizontal" — |worldNormal.y| above this is flat
    private let horizontalNormalThreshold: Float = 0.7
    
    /// Collect all 3D world-space obstacle points within the stop radius.
    /// Uses mesh faces to skip horizontal planes (floors, ceilings, table tops).
    /// Depth map points that overlap identified horizontal mesh surfaces are also filtered out.
    /// Used to render white dots over obstacles in the AR camera view.
    func collectObstaclePoints3D(frame: ARFrame) -> [simd_float3] {
        guard isEnabled else { return [] }
        
        let camPos = simd_float3(frame.camera.transform.columns.3.x,
                                  frame.camera.transform.columns.3.y,
                                  frame.camera.transform.columns.3.z)
        let radiusSq = stopRadius * stopRadius
        var points: [simd_float3] = []
        
        // Spatial grid to record horizontal surface heights for depth filtering.
        // Key = quantised (x, z) position, Value = array of Y heights at that column.
        let gridCell: Float = 0.05  // 5cm buckets
        var horizontalHeights: [UInt64: [Float]] = [:]
        
        func gridKey(_ x: Float, _ z: Float) -> UInt64 {
            let ix = Int32(floorf(x / gridCell))
            let iz = Int32(floorf(z / gridCell))
            let ux = UInt64(bitPattern: Int64(ix))
            let uz = UInt64(bitPattern: Int64(iz))
            return (ux << 32) | (uz & 0xFFFFFFFF)
        }
        
        // Rotation part of each anchor transform for normal transformation
        func rotationMatrix(_ m: simd_float4x4) -> simd_float3x3 {
            simd_float3x3(
                simd_float3(m.columns.0.x, m.columns.0.y, m.columns.0.z),
                simd_float3(m.columns.1.x, m.columns.1.y, m.columns.1.z),
                simd_float3(m.columns.2.x, m.columns.2.y, m.columns.2.z)
            )
        }
        
        // 1. Process mesh faces — keep non-horizontal vertices, record horizontal surface heights
        for anchor in frame.anchors {
            guard let meshAnchor = anchor as? ARMeshAnchor else { continue }
            let transform = meshAnchor.transform
            
            let anchorPos = simd_float3(transform.columns.3.x, transform.columns.3.y, transform.columns.3.z)
            if simd_distance(anchorPos, camPos) > stopRadius + 1.5 { continue }
            
            let geometry = meshAnchor.geometry
            let vertexSource = geometry.vertices
            let vertexCount = vertexSource.count
            let vBuffer = vertexSource.buffer.contents()
            let vStride = vertexSource.stride
            let vOffset = vertexSource.offset
            
            let faces = geometry.faces
            let faceCount = faces.count
            let fBuffer = faces.buffer.contents()
            let bytesPerIndex = faces.bytesPerIndex
            let indicesPerFace = faces.indexCountPerPrimitive  // 3 for triangles
            
            let rot = rotationMatrix(transform)
            
            // Per-vertex: track if ANY adjacent face is non-horizontal
            // (vertex is drawable if at least one of its faces is vertical)
            var vertexIsVertical = [Bool](repeating: false, count: vertexCount)
            
            for fi in 0..<faceCount {
                // Read vertex indices for this face
                var idx = [Int](repeating: 0, count: indicesPerFace)
                for j in 0..<indicesPerFace {
                    let ptr = fBuffer.advanced(by: (fi * indicesPerFace + j) * bytesPerIndex)
                    if bytesPerIndex == 4 {
                        idx[j] = Int(ptr.assumingMemoryBound(to: Int32.self).pointee)
                    } else {
                        idx[j] = Int(ptr.assumingMemoryBound(to: Int16.self).pointee)
                    }
                }
                
                // Read local vertices
                func localVertex(_ i: Int) -> simd_float3 {
                    let p = vBuffer.advanced(by: vOffset + vStride * i)
                    return p.assumingMemoryBound(to: simd_float3.self).pointee
                }
                let v0 = localVertex(idx[0])
                let v1 = localVertex(idx[1])
                let v2 = localVertex(idx[2])
                
                // Face normal in local space → world space
                let edge1 = v1 - v0
                let edge2 = v2 - v0
                let localNormal = simd_normalize(simd_cross(edge1, edge2))
                let worldNormal = simd_normalize(rot * localNormal)
                
                let isHorizontal = abs(worldNormal.y) > horizontalNormalThreshold
                
                if isHorizontal {
                    // Record face centroid height for depth filtering
                    let localCenter = (v0 + v1 + v2) / 3.0
                    let wc = transform * simd_float4(localCenter.x, localCenter.y, localCenter.z, 1.0)
                    let key = gridKey(wc.x, wc.z)
                    horizontalHeights[key, default: []].append(wc.y)
                } else {
                    // Mark vertices of this vertical face as drawable
                    for j in 0..<indicesPerFace {
                        if idx[j] < vertexCount { vertexIsVertical[idx[j]] = true }
                    }
                }
            }
            
            // Emit only vertices belonging to at least one non-horizontal face
            for i in Swift.stride(from: 0, to: vertexCount, by: 2) {
                guard vertexIsVertical[i] else { continue }
                
                let ptr = vBuffer.advanced(by: vOffset + vStride * i)
                let vertex = ptr.assumingMemoryBound(to: simd_float3.self).pointee
                let world4 = transform * simd_float4(vertex.x, vertex.y, vertex.z, 1.0)
                let worldPos = simd_float3(world4.x, world4.y, world4.z)
                
                guard simd_distance_squared(worldPos, camPos) <= radiusSq else { continue }
                
                if floorHeightEstimated {
                    let h = worldPos.y - floorHeight
                    if h < floorFilterMargin || h > ceilingFilterHeight { continue }
                }
                
                points.append(worldPos)
            }
        }
        
        // 2. Depth map points — skip any that overlap a horizontal mesh surface
        guard let sceneDepth = frame.sceneDepth ?? frame.smoothedSceneDepth else { return points }
        
        let depthMap = sceneDepth.depthMap
        let width = CVPixelBufferGetWidth(depthMap)
        let height = CVPixelBufferGetHeight(depthMap)
        
        CVPixelBufferLockBaseAddress(depthMap, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(depthMap, .readOnly) }
        
        guard let baseAddress = CVPixelBufferGetBaseAddress(depthMap) else { return points }
        let floatBuffer = baseAddress.assumingMemoryBound(to: Float32.self)
        let bytesPerRow = CVPixelBufferGetBytesPerRow(depthMap)
        let floatsPerRow = bytesPerRow / MemoryLayout<Float32>.size
        
        let intrinsics = frame.camera.intrinsics
        let imageRes = frame.camera.imageResolution
        let scaleX = Float(width) / Float(imageRes.width)
        let scaleY = Float(height) / Float(imageRes.height)
        let fx = intrinsics[0][0] * scaleX
        let fy = intrinsics[1][1] * scaleY
        let cx = intrinsics[2][0] * scaleX
        let cy = intrinsics[2][1] * scaleY
        let camTransform = frame.camera.transform
        
        let heightTolerance: Float = 0.005  // 0.5cm tolerance for matching horizontal surface
        let sampleStep = max(1, min(width, height) / 32)
        
        for py in Swift.stride(from: 0, to: height, by: sampleStep) {
            for px in Swift.stride(from: 0, to: width, by: sampleStep) {
                let depth = floatBuffer[py * floatsPerRow + px]
                guard depth > 0.01 && depth < stopRadius else { continue }
                
                let ix = (Float(px) - cx) / fx * depth
                let iy = (Float(py) - cy) / fy * depth
                let camPoint = simd_float4(ix, -iy, -depth, 1.0)
                let worldPoint = camTransform * camPoint
                
                if floorHeightEstimated {
                    let h = worldPoint.y - floorHeight
                    if h < floorFilterMargin || h > ceilingFilterHeight { continue }
                }
                
                // Check if this depth point overlaps a known horizontal mesh surface
                let key = gridKey(worldPoint.x, worldPoint.z)
                if let heights = horizontalHeights[key] {
                    let onHorizontal = heights.contains { abs(worldPoint.y - $0) < heightTolerance }
                    if onHorizontal { continue }
                }
                
                points.append(simd_float3(worldPoint.x, worldPoint.y, worldPoint.z))
            }
        }
        
        return points
    }
    
    // MARK: - Detection
    
    /// Called every frame from the display link to update obstacle state.
    func update() {
        guard isEnabled, let grid = occupancyGrid else {
            if obstacleDetected {
                clearState()
            }
            return
        }
        
        let pos = grid.devicePosition
        let heading = pos.heading
        
        // Query occupied cells within the protective radius, with classification
        let occupied = grid.getOccupiedCellsDetailed(aroundX: pos.x, aroundY: pos.y, radius: stopRadius)
        
        // Filter out floor and ceiling — they are surfaces, not obstacles
        let realObstacles = occupied.filter { cell in
            cell.classification != .floor && cell.classification != .ceiling
        }
        
        // Store world positions for grid map highlighting
        obstacleWorldPositions = realObstacles.map { ($0.worldX, $0.worldY) }
        
        // Also check depth-based detection for dynamic obstacles
        let hasGridObstacles = !realObstacles.isEmpty
        let hasDepthObstacle = depthObstacleDistance != nil
        
        guard hasGridObstacles || hasDepthObstacle else {
            if obstacleDetected {
                clearState()
                print("[Obstacle] ✅ All clear — no obstacles within \(Int(stopRadius * 1000))mm")
            }
            return
        }
        
        // Find nearest distance and compute weighted direction toward obstacles
        // (closer obstacles have more influence)
        var nearest: Float = .greatestFiniteMagnitude
        var nearestClassif: MeshClassification = .none
        var weightedDirX: Float = 0
        var weightedDirY: Float = 0
        
        let sinH = sinf(heading)
        let cosH = cosf(heading)
        var obstacles: [NearbyObstacle] = []
        
        for cell in realObstacles {
            let dx = cell.worldX - pos.x
            let dy = cell.worldY - pos.y
            let distance = cell.distance
            
            if distance < 0.001 { continue }  // skip if right on top
            
            if distance < nearest {
                nearest = distance
                nearestClassif = cell.classification
            }
            
            // Weight by inverse distance — closer obstacles dominate
            let weight = 1.0 / distance
            weightedDirX += dx * weight
            weightedDirY += dy * weight
            
            // Build local direction for this cell
            let normDX = dx / distance
            let normDY = dy / distance
            let localY =  normDX * sinH + normDY * cosH
            let localX = -normDX * cosH + normDY * sinH
            
            // Elevation relative to device orientation:
            // Build 3D direction in ARKit world space, transform to camera-local, compute pitch
            let worldDir = simd_float3(dx, cell.height - pos.z, dy)  // ARKit: X, Y(up), Z
            let invRot = simd_float3x3(
                simd_float3(cameraTransform.columns.0.x, cameraTransform.columns.1.x, cameraTransform.columns.2.x),
                simd_float3(cameraTransform.columns.0.y, cameraTransform.columns.1.y, cameraTransform.columns.2.y),
                simd_float3(cameraTransform.columns.0.z, cameraTransform.columns.1.z, cameraTransform.columns.2.z)
            )
            let localDir3D = invRot * worldDir
            // Camera local: X=right, Y=up, -Z=forward
            let elevation = Int(atan2f(localDir3D.y, -localDir3D.z) * 180 / .pi)
            
            obstacles.append(NearbyObstacle(
                distance: distance,
                classification: cell.classification,
                localDirection: simd_float2(localX, localY),
                elevationDegrees: elevation,
                isDepthBased: false
            ))
        }
        
        // Sort by distance and deduplicate classifications for display
        obstacles.sort { $0.distance < $1.distance }
        
        // Group by classification to reduce noise (keep closest per type)
        var seenTypes = Set<UInt8>()
        var uniqueObstacles: [NearbyObstacle] = []
        for obs in obstacles {
            let key = obs.classification.rawValue
            if !seenTypes.contains(key) {
                seenTypes.insert(key)
                uniqueObstacles.append(obs)
            }
        }
        nearbyObstacles = uniqueObstacles
        
        // Merge depth-based obstacle if present — compute angle from grid world position
        if let depthDist = depthObstacleDistance, let depthWorldPos = depthObstacleWorldPosition {
            if depthDist < nearest {
                nearest = depthDist
                nearestClassif = .none
            }
            
            // Direction from device to depth obstacle in grid world
            let dx = depthWorldPos.x - pos.x
            let dy = depthWorldPos.y - pos.y
            let dist = max(depthDist, 0.001)
            
            // Include in blocked direction (world space)
            let depthWeight = 2.0 / max(depthDist, 0.01)  // stronger weight for depth (more immediate)
            weightedDirX += (dx / dist) * depthWeight
            weightedDirY += (dy / dist) * depthWeight
            
            // Compute local direction for angle display (same formula as grid obstacles)
            let normDX = dx / dist
            let normDY = dy / dist
            let depthLocalY =  normDX * sinH + normDY * cosH
            let depthLocalX = -normDX * cosH + normDY * sinH
            
            // Elevation relative to device orientation
            let depthWorldDir = simd_float3(dx, depthWorldPos.z - pos.z, dy)  // ARKit: X, Y(up), Z
            let depthInvRot = simd_float3x3(
                simd_float3(cameraTransform.columns.0.x, cameraTransform.columns.1.x, cameraTransform.columns.2.x),
                simd_float3(cameraTransform.columns.0.y, cameraTransform.columns.1.y, cameraTransform.columns.2.y),
                simd_float3(cameraTransform.columns.0.z, cameraTransform.columns.1.z, cameraTransform.columns.2.z)
            )
            let depthLocalDir3D = depthInvRot * depthWorldDir
            let depthElevation = Int(atan2f(depthLocalDir3D.y, -depthLocalDir3D.z) * 180 / .pi)
            
            let depthObstacle = NearbyObstacle(
                distance: depthDist,
                classification: .none,
                localDirection: simd_float2(depthLocalX, depthLocalY),
                elevationDegrees: depthElevation,
                isDepthBased: true
            )
            nearbyObstacles.insert(depthObstacle, at: depthDist < (nearbyObstacles.first?.distance ?? .greatestFiniteMagnitude) ? 0 : nearbyObstacles.count)
        }
        
        let wasBlocked = obstacleDetected
        obstacleDetected = true
        nearestObstacleDistance = nearest
        nearestClassification = nearestClassif
        
        // Normalize the blocked direction
        let dirMag = sqrtf(weightedDirX * weightedDirX + weightedDirY * weightedDirY)
        if dirMag > 0.001 {
            blockedWorldDirection = simd_float2(weightedDirX / dirMag, weightedDirY / dirMag)
            
            // Convert to device-local coordinates
            let localY =  weightedDirX / dirMag * sinH + weightedDirY / dirMag * cosH   // forward component
            let localX = -weightedDirX / dirMag * cosH + weightedDirY / dirMag * sinH    // right component
            blockedLocalDirection = simd_float2(localX, localY)
        }
        
        if !wasBlocked {
            print("[Obstacle] 🛑 Obstacle within \(Int(stopRadius * 1000))mm — nearest at \(String(format: "%.0f", nearest * 1000))mm")
            onObstacleStateChanged?(true, nearest)
            // Stop motors if they're driving toward the obstacle
            stopMotionIfNeeded()
        }
    }
    
    private func clearState() {
        obstacleDetected = false
        nearestObstacleDistance = nil
        blockedWorldDirection = nil
        blockedLocalDirection = nil
        nearestClassification = .none
        nearbyObstacles = []
        obstacleWorldPositions = []
        onObstacleStateChanged?(false, nil)
    }
    
    // MARK: - Motor Filtering
    
    /// Filter a joystick drive command.
    /// x = turn (+ right), y = throttle (+ forward relative to device heading).
    /// Blocks the component of motion that points toward obstacles; allows escape.
    func filterDrive(x: Float, y: Float) -> (x: Float, y: Float) {
        guard isEnabled, obstacleDetected, let blocked = blockedLocalDirection else {
            return (x, y)
        }
        
        // Project joystick vector onto blocked direction
        let dot = x * blocked.x + y * blocked.y
        
        // If the joystick is pointing away from the obstacle (dot < 0), allow it fully
        if dot <= 0 {
            return (x, y)
        }
        
        // Remove the component pointing toward the obstacle
        let filteredX = x - dot * blocked.x
        let filteredY = y - dot * blocked.y
        
        return (filteredX, filteredY)
    }
    
    /// Filter raw motor powers.
    /// Determines the net movement direction from motor values and blocks
    /// motion toward obstacles.
    func filterMotors(a: Int8, b: Int8, c: Int8, d: Int8) -> (a: Int8, b: Int8, c: Int8, d: Int8) {
        guard isEnabled, obstacleDetected, let blocked = blockedLocalDirection else {
            return (a, b, c, d)
        }
        
        // All zero — always pass through
        if a == 0 && b == 0 && c == 0 && d == 0 {
            return (0, 0, 0, 0)
        }
        
        // Derive approximate local movement direction from motor powers:
        // Left side = (a + c) / 2, Right side = (b + d) / 2
        let leftPower  = Float(Int(a) + Int(c)) / 2.0
        let rightPower = Float(Int(b) + Int(d)) / 2.0
        
        // Forward component: average of both sides
        let forward = (leftPower + rightPower) / 2.0
        // Turn component: difference (positive = turning right in place)
        let turn = (leftPower - rightPower) / 2.0
        
        // Movement direction in device-local space: (turn, forward)
        // turn > 0 with no forward = spinning right, which sweeps the front right
        // For simplicity, map: localX ~ turn, localY ~ forward
        let moveX = turn / 100.0   // normalize to -1…1 range
        let moveY = forward / 100.0
        
        // Dot product with blocked direction
        let dot = moveX * blocked.x + moveY * blocked.y
        
        // Moving away from obstacle — allow
        if dot <= 0 {
            return (a, b, c, d)
        }
        
        // Moving toward obstacle — block all motion
        // (We can't easily decompose motor powers into components,
        //  so we fully block rather than partially)
        return (0, 0, 0, 0)
    }
    
    /// If motors are currently moving toward the obstacle, stop immediately
    private func stopMotionIfNeeded() {
        let ble = ESP32BLEManager.shared
        guard let lastData = ble.lastMotorDataPublic, lastData.count == 4 else { return }
        guard let blocked = blockedLocalDirection else { return }
        
        let a = Int8(bitPattern: lastData[0])
        let b = Int8(bitPattern: lastData[1])
        let c = Int8(bitPattern: lastData[2])
        let d = Int8(bitPattern: lastData[3])
        
        if a == 0 && b == 0 && c == 0 && d == 0 { return }
        
        let leftPower  = Float(Int(a) + Int(c)) / 2.0
        let rightPower = Float(Int(b) + Int(d)) / 2.0
        let forward = (leftPower + rightPower) / 200.0
        let turn    = (leftPower - rightPower) / 200.0
        
        let dot = turn * blocked.x + forward * blocked.y
        if dot > 0 {
            ble.stopAll()
        }
    }
}

