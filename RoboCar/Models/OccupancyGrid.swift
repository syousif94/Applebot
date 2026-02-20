//
//  OccupancyGrid.swift
//  RoboCar
//
//  Created by Sammy Yousif on 1/3/26.
//

import Foundation
import simd

/// Represents the state of a cell in the occupancy grid
enum CellState: UInt8 {
    case unknown = 0
    case free = 1
    case occupied = 2
}

/// Classification of a mesh surface from ARKit scene reconstruction
enum MeshClassification: UInt8 {
    case none = 0
    case wall = 1
    case floor = 2
    case ceiling = 3
    case table = 4
    case seat = 5
    case window = 6
    case door = 7
    
    /// Human-readable label for display
    var label: String {
        switch self {
        case .none: return ""
        case .wall: return "Wall"
        case .floor: return "Floor"
        case .ceiling: return "Ceiling"
        case .table: return "Table"
        case .seat: return "Seat"
        case .window: return "Window"
        case .door: return "Door"
        }
    }
    
    /// Color for rendering on the 2D grid
    var color: (r: CGFloat, g: CGFloat, b: CGFloat) {
        switch self {
        case .none: return (1.0, 1.0, 1.0)       // white
        case .wall: return (0.8, 0.4, 0.2)       // orange
        case .floor: return (0.15, 0.05, 0.25)   // dark purple
        case .ceiling: return (0.5, 0.5, 0.5)    // gray (shouldn't render)
        case .table: return (0.2, 0.6, 1.0)      // blue
        case .seat: return (0.2, 0.8, 0.4)       // green
        case .window: return (0.4, 0.9, 0.9)     // cyan
        case .door: return (1.0, 0.8, 0.2)       // yellow
        }
    }
    
    /// Convert from ARMeshClassification
    static func from(arClassification: Int) -> MeshClassification {
        switch arClassification {
        case 0: return .none
        case 1: return .wall
        case 2: return .floor
        case 3: return .ceiling
        case 4: return .table
        case 5: return .seat
        case 6: return .window
        case 7: return .door
        default: return .none
        }
    }
}

/// Device position and orientation in the grid
struct DevicePosition {
    var x: Float  // meters from origin
    var y: Float  // meters from origin
    var z: Float  // height in meters
    var heading: Float  // radians, 0 = positive X axis
    
    static let zero = DevicePosition(x: 0, y: 0, z: 0, heading: 0)
}

/// A large 2D occupancy grid that stores obstacle information
/// Grid uses a sparse representation for memory efficiency with large areas
class OccupancyGrid {
    
    // MARK: - Configuration
    
    /// Size of each cell in meters
    let cellSize: Float
    
    /// Grid dimensions (number of cells in each direction from origin)
    /// Total grid spans from -gridRadius to +gridRadius in both X and Y
    let gridRadius: Int
    
    /// Total grid size in cells (diameter)
    var gridSize: Int { gridRadius * 2 }
    
    /// Total grid size in meters
    var sizeInMeters: Float { Float(gridSize) * cellSize }
    
    // MARK: - State
    
    /// The occupancy grid data - 2D array [x][y]
    /// Using UInt8 for memory efficiency
    private var cells: [[UInt8]]
    
    /// Height data per cell - stores min height
    private var minHeights: [[Float]]
    
    /// Height data per cell - stores max height
    private var maxHeights: [[Float]]
    
    /// Classification data per cell
    private var classifications: [[UInt8]]
    
    /// Global min and max recorded heights
    private(set) var globalMinHeight: Float = .greatestFiniteMagnitude
    private(set) var globalMaxHeight: Float = -.greatestFiniteMagnitude
    
    /// Current device position in world coordinates
    var devicePosition: DevicePosition = .zero
    
    /// Origin offset in world coordinates (where grid center maps to)
    var originOffset: simd_float3 = .zero
    
    /// Lock for thread-safe access
    private let lock = NSLock()
    
    // MARK: - Statistics
    
    /// Number of occupied cells
    private(set) var occupiedCount: Int = 0
    
    /// Number of free cells
    private(set) var freeCount: Int = 0
    
    // MARK: - Initialization
    
    /// Initialize a new occupancy grid
    /// - Parameters:
    ///   - cellSize: Size of each cell in meters (default 0.05m = 5cm)
    ///   - gridRadius: Number of cells from center to edge (default 500 = 25m radius at 5cm cells)
    init(cellSize: Float = 0.05, gridRadius: Int = 500) {
        self.cellSize = cellSize
        self.gridRadius = gridRadius
        
        // Initialize grid with unknown state
        let size = gridRadius * 2
        self.cells = Array(repeating: Array(repeating: CellState.unknown.rawValue, count: size), count: size)
        self.minHeights = Array(repeating: Array(repeating: Float.greatestFiniteMagnitude, count: size), count: size)
        self.maxHeights = Array(repeating: Array(repeating: -Float.greatestFiniteMagnitude, count: size), count: size)
        self.classifications = Array(repeating: Array(repeating: MeshClassification.none.rawValue, count: size), count: size)
        
        print("OccupancyGrid initialized: \(size)x\(size) cells, \(sizeInMeters)m x \(sizeInMeters)m, cell size: \(cellSize)m")
    }
    
    // MARK: - Coordinate Conversion
    
    /// Convert world coordinates to grid indices
    func worldToGrid(_ worldX: Float, _ worldY: Float) -> (x: Int, y: Int)? {
        let localX = worldX - originOffset.x
        let localY = worldY - originOffset.z  // Note: ARKit Y is up, we use Z for forward
        
        let gridX = Int((localX / cellSize) + Float(gridRadius))
        let gridY = Int((localY / cellSize) + Float(gridRadius))
        
        guard gridX >= 0 && gridX < gridSize && gridY >= 0 && gridY < gridSize else {
            return nil
        }
        
        return (gridX, gridY)
    }
    
    /// Convert grid indices to world coordinates (center of cell)
    func gridToWorld(_ gridX: Int, _ gridY: Int) -> (x: Float, y: Float) {
        let worldX = (Float(gridX - gridRadius) + 0.5) * cellSize + originOffset.x
        let worldY = (Float(gridY - gridRadius) + 0.5) * cellSize + originOffset.z
        return (worldX, worldY)
    }
    
    // MARK: - Grid Access
    
    /// Get the state of a cell at world coordinates
    func getState(worldX: Float, worldY: Float) -> CellState {
        lock.lock()
        defer { lock.unlock() }
        
        guard let (gx, gy) = worldToGrid(worldX, worldY) else {
            return .unknown
        }
        return CellState(rawValue: cells[gx][gy]) ?? .unknown
    }
    
    /// Get the state of a cell at grid coordinates
    func getState(gridX: Int, gridY: Int) -> CellState {
        lock.lock()
        defer { lock.unlock() }
        
        guard gridX >= 0 && gridX < gridSize && gridY >= 0 && gridY < gridSize else {
            return .unknown
        }
        return CellState(rawValue: cells[gridX][gridY]) ?? .unknown
    }
    
    /// Set the state of a cell at world coordinates
    func setState(worldX: Float, worldY: Float, state: CellState) {
        lock.lock()
        defer { lock.unlock() }
        
        guard let (gx, gy) = worldToGrid(worldX, worldY) else { return }
        setStateUnsafe(gridX: gx, gridY: gy, state: state)
    }
    
    /// Set the state of a cell at grid coordinates (not thread-safe, use within lock)
    private func setStateUnsafe(gridX: Int, gridY: Int, state: CellState) {
        guard gridX >= 0 && gridX < gridSize && gridY >= 0 && gridY < gridSize else { return }
        
        let oldState = CellState(rawValue: cells[gridX][gridY]) ?? .unknown
        
        // Update counts
        if oldState == .occupied { occupiedCount -= 1 }
        if oldState == .free { freeCount -= 1 }
        if state == .occupied { occupiedCount += 1 }
        if state == .free { freeCount += 1 }
        
        cells[gridX][gridY] = state.rawValue
    }
    
    // MARK: - Bulk Operations
    
    /// Mark a line of cells as free (ray from device to obstacle)
    func markRayAsFree(fromX: Float, fromY: Float, toX: Float, toY: Float) {
        lock.lock()
        defer { lock.unlock() }
        
        guard let start = worldToGrid(fromX, fromY),
              let end = worldToGrid(toX, toY) else { return }
        
        // Bresenham's line algorithm
        var x = start.x
        var y = start.y
        let dx = abs(end.x - start.x)
        let dy = abs(end.y - start.y)
        let sx = start.x < end.x ? 1 : -1
        let sy = start.y < end.y ? 1 : -1
        var err = dx - dy
        
        while true {
            // Don't mark the endpoint (that's the obstacle)
            if x == end.x && y == end.y { break }
            
            if getState(gridX: x, gridY: y) != .occupied {
                setStateUnsafe(gridX: x, gridY: y, state: .free)
            }
            
            let e2 = 2 * err
            if e2 > -dy {
                err -= dy
                x += sx
            }
            if e2 < dx {
                err += dx
                y += sy
            }
        }
    }
    
    /// Mark a point as occupied
    func markOccupied(worldX: Float, worldY: Float) {
        setState(worldX: worldX, worldY: worldY, state: .occupied)
    }
    
    /// Mark a point as free (floor)
    func markFree(worldX: Float, worldY: Float) {
        setState(worldX: worldX, worldY: worldY, state: .free)
    }
    
    /// Mark multiple points as free/floor (batch operation) - only if not already occupied
    func markFreeBatch(_ points: [(x: Float, y: Float)]) {
        lock.lock()
        defer { lock.unlock() }
        
        for point in points {
            guard let (gx, gy) = worldToGrid(point.x, point.y) else { continue }
            // Only mark as free if not already occupied (obstacles take priority)
            if cells[gx][gy] != CellState.occupied.rawValue {
                setStateUnsafe(gridX: gx, gridY: gy, state: .free)
            }
        }
    }
    
    /// Mark a point as occupied with height info
    func markOccupied(worldX: Float, worldY: Float, height: Float) {
        lock.lock()
        defer { lock.unlock() }
        
        guard let (gx, gy) = worldToGrid(worldX, worldY) else { return }
        setStateUnsafe(gridX: gx, gridY: gy, state: .occupied)
        
        // Update height data
        if height < minHeights[gx][gy] {
            minHeights[gx][gy] = height
        }
        if height > maxHeights[gx][gy] {
            maxHeights[gx][gy] = height
        }
        
        // Update global min/max
        if height < globalMinHeight {
            globalMinHeight = height
        }
        if height > globalMaxHeight {
            globalMaxHeight = height
        }
    }
    
    /// Mark multiple points as occupied (batch operation)
    func markOccupiedBatch(_ points: [(x: Float, y: Float)]) {
        lock.lock()
        defer { lock.unlock() }
        
        for point in points {
            guard let (gx, gy) = worldToGrid(point.x, point.y) else { continue }
            setStateUnsafe(gridX: gx, gridY: gy, state: .occupied)
        }
    }
    
    /// Mark multiple points as occupied with heights (batch operation)
    func markOccupiedBatchWithHeights(_ points: [(x: Float, y: Float, height: Float)]) {
        lock.lock()
        defer { lock.unlock() }
        
        for point in points {
            guard let (gx, gy) = worldToGrid(point.x, point.y) else { continue }
            setStateUnsafe(gridX: gx, gridY: gy, state: .occupied)
            
            // Update height data
            if point.height < minHeights[gx][gy] {
                minHeights[gx][gy] = point.height
            }
            if point.height > maxHeights[gx][gy] {
                maxHeights[gx][gy] = point.height
            }
            
            // Update global min/max
            if point.height < globalMinHeight {
                globalMinHeight = point.height
            }
            if point.height > globalMaxHeight {
                globalMaxHeight = point.height
            }
        }
    }
    
    /// Mark multiple points as occupied with heights and classification (batch operation)
    func markOccupiedBatchWithClassification(_ points: [(x: Float, y: Float, height: Float, classification: MeshClassification)]) {
        lock.lock()
        defer { lock.unlock() }
        
        for point in points {
            guard let (gx, gy) = worldToGrid(point.x, point.y) else { continue }
            setStateUnsafe(gridX: gx, gridY: gy, state: .occupied)
            
            // Update classification (prefer more specific classifications over .none)
            if point.classification != .none || classifications[gx][gy] == MeshClassification.none.rawValue {
                classifications[gx][gy] = point.classification.rawValue
            }
            
            // Update height data
            if point.height < minHeights[gx][gy] {
                minHeights[gx][gy] = point.height
            }
            if point.height > maxHeights[gx][gy] {
                maxHeights[gx][gy] = point.height
            }
            
            // Update global min/max
            if point.height < globalMinHeight {
                globalMinHeight = point.height
            }
            if point.height > globalMaxHeight {
                globalMaxHeight = point.height
            }
        }
    }
    
    /// Mark multiple points as free with classification (batch operation)
    func markFreeBatchWithClassification(_ points: [(x: Float, y: Float, classification: MeshClassification)]) {
        lock.lock()
        defer { lock.unlock() }
        
        for point in points {
            guard let (gx, gy) = worldToGrid(point.x, point.y) else { continue }
            if cells[gx][gy] != CellState.occupied.rawValue {
                setStateUnsafe(gridX: gx, gridY: gy, state: .free)
                if point.classification != .none {
                    classifications[gx][gy] = point.classification.rawValue
                }
            }
        }
    }
    
    /// Clear all cells to unknown state and reset origin
    func clear() {
        lock.lock()
        defer { lock.unlock() }
        
        for x in 0..<gridSize {
            for y in 0..<gridSize {
                cells[x][y] = CellState.unknown.rawValue
                minHeights[x][y] = Float.greatestFiniteMagnitude
                maxHeights[x][y] = -Float.greatestFiniteMagnitude
                classifications[x][y] = MeshClassification.none.rawValue
            }
        }
        occupiedCount = 0
        freeCount = 0
        
        // Reset origin offset so new AR session coordinates map correctly
        originOffset = .zero
        
        // Reset global height tracking
        globalMinHeight = .greatestFiniteMagnitude
        globalMaxHeight = -.greatestFiniteMagnitude
        
        // Reset device position
        devicePosition = .zero
    }
    
    // MARK: - Region Queries
    
    /// Get all occupied cells within a radius of a point (in world coordinates)
    func getOccupiedCells(aroundX: Float, aroundY: Float, radius: Float) -> [(worldX: Float, worldY: Float)] {
        lock.lock()
        defer { lock.unlock() }
        
        var result: [(worldX: Float, worldY: Float)] = []
        
        guard let center = worldToGrid(aroundX, aroundY) else { return result }
        
        let cellRadius = Int(radius / cellSize) + 1
        
        for dx in -cellRadius...cellRadius {
            for dy in -cellRadius...cellRadius {
                let gx = center.x + dx
                let gy = center.y + dy
                
                guard gx >= 0 && gx < gridSize && gy >= 0 && gy < gridSize else { continue }
                
                if cells[gx][gy] == CellState.occupied.rawValue {
                    let world = gridToWorld(gx, gy)
                    let dist = sqrt(pow(world.x - aroundX, 2) + pow(world.y - aroundY, 2))
                    if dist <= radius {
                        result.append((world.x, world.y))
                    }
                }
            }
        }
        
        return result
    }
    
    /// Get all occupied cells within a radius, including classification, distance, and height
    func getOccupiedCellsDetailed(aroundX: Float, aroundY: Float, radius: Float) -> [(worldX: Float, worldY: Float, distance: Float, classification: MeshClassification, height: Float)] {
        lock.lock()
        defer { lock.unlock() }
        
        var result: [(worldX: Float, worldY: Float, distance: Float, classification: MeshClassification, height: Float)] = []
        
        guard let center = worldToGrid(aroundX, aroundY) else { return result }
        
        let cellRadius = Int(radius / cellSize) + 1
        
        for dx in -cellRadius...cellRadius {
            for dy in -cellRadius...cellRadius {
                let gx = center.x + dx
                let gy = center.y + dy
                
                guard gx >= 0 && gx < gridSize && gy >= 0 && gy < gridSize else { continue }
                
                if cells[gx][gy] == CellState.occupied.rawValue {
                    let world = gridToWorld(gx, gy)
                    let dist = sqrt(pow(world.x - aroundX, 2) + pow(world.y - aroundY, 2))
                    if dist <= radius {
                        let classif = MeshClassification(rawValue: classifications[gx][gy]) ?? .none
                        let h = maxHeights[gx][gy] > -Float.greatestFiniteMagnitude ? maxHeights[gx][gy] : 0
                        result.append((world.x, world.y, dist, classif, h))
                    }
                }
            }
        }
        
        return result
    }
    
    // MARK: - Frontier Detection
    
    /// A frontier cell is an unknown cell adjacent to at least one free cell.
    /// Returns frontier cells clustered into groups, sorted by distance from the device.
    /// Each returned point is the centroid of a frontier cluster in world coordinates.
    func findFrontierClusters(maxClusters: Int = 20, minClusterSize: Int = 3) -> [(x: Float, y: Float, size: Int)] {
        lock.lock()
        defer { lock.unlock() }
        
        // Find the bounding box of known cells to limit search
        var minGX = gridSize, maxGX = 0, minGY = gridSize, maxGY = 0
        var hasKnown = false
        
        // Scan a reasonable area around the device position
        let pos = devicePosition
        guard let deviceGrid = worldToGrid(pos.x, pos.y) else { return [] }
        
        // Search within 10m of device (200 cells at 5cm)
        let searchRadius = min(200, gridRadius)
        let startX = max(0, deviceGrid.x - searchRadius)
        let endX = min(gridSize - 1, deviceGrid.x + searchRadius)
        let startY = max(0, deviceGrid.y - searchRadius)
        let endY = min(gridSize - 1, deviceGrid.y + searchRadius)
        
        // Find all frontier cells
        var frontierCells: [(gx: Int, gy: Int)] = []
        
        let neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        
        for gx in startX...endX {
            for gy in startY...endY {
                guard cells[gx][gy] == CellState.unknown.rawValue else { continue }
                
                // Check if adjacent to a free cell
                var adjacentToFree = false
                for (dx, dy) in neighbors {
                    let nx = gx + dx
                    let ny = gy + dy
                    if nx >= 0 && nx < gridSize && ny >= 0 && ny < gridSize {
                        if cells[nx][ny] == CellState.free.rawValue {
                            adjacentToFree = true
                            break
                        }
                    }
                }
                
                if adjacentToFree {
                    frontierCells.append((gx, gy))
                }
            }
        }
        
        guard !frontierCells.isEmpty else { return [] }
        
        // Cluster frontier cells using simple flood-fill grouping
        var visited = Set<Int>()  // gx * gridSize + gy
        var clusters: [[(gx: Int, gy: Int)]] = []
        
        for cell in frontierCells {
            let key = cell.gx * gridSize + cell.gy
            if visited.contains(key) { continue }
            
            // BFS to find connected frontier cells
            var cluster: [(gx: Int, gy: Int)] = []
            var queue: [(gx: Int, gy: Int)] = [cell]
            visited.insert(key)
            
            while !queue.isEmpty {
                let current = queue.removeFirst()
                cluster.append(current)
                
                for (dx, dy) in neighbors {
                    let nx = current.gx + dx
                    let ny = current.gy + dy
                    let nkey = nx * gridSize + ny
                    
                    if visited.contains(nkey) { continue }
                    if nx < startX || nx > endX || ny < startY || ny > endY { continue }
                    if cells[nx][ny] != CellState.unknown.rawValue { continue }
                    
                    // Check if this neighbor is also a frontier
                    var isFrontier = false
                    for (ddx, ddy) in neighbors {
                        let nnx = nx + ddx
                        let nny = ny + ddy
                        if nnx >= 0 && nnx < gridSize && nny >= 0 && nny < gridSize {
                            if cells[nnx][nny] == CellState.free.rawValue {
                                isFrontier = true
                                break
                            }
                        }
                    }
                    
                    if isFrontier {
                        visited.insert(nkey)
                        queue.append((nx, ny))
                    }
                }
            }
            
            if cluster.count >= minClusterSize {
                clusters.append(cluster)
            }
        }
        
        // Convert clusters to world-coordinate centroids
        var result: [(x: Float, y: Float, size: Int)] = clusters.map { cluster in
            var sumX: Float = 0
            var sumY: Float = 0
            for cell in cluster {
                let world = gridToWorld(cell.gx, cell.gy)
                sumX += world.x
                sumY += world.y
            }
            let cx = sumX / Float(cluster.count)
            let cy = sumY / Float(cluster.count)
            return (cx, cy, cluster.count)
        }
        
        // Sort by distance from device
        result.sort { a, b in
            let distA = (a.x - pos.x) * (a.x - pos.x) + (a.y - pos.y) * (a.y - pos.y)
            let distB = (b.x - pos.x) * (b.x - pos.x) + (b.y - pos.y) * (b.y - pos.y)
            return distA < distB
        }
        
        return Array(result.prefix(maxClusters))
    }
    
    /// Check whether the mapped area is fully enclosed (no frontier cells reachable).
    /// Returns true if all free space is bounded by occupied cells with no unknown neighbors.
    func isFullyMapped() -> Bool {
        let frontiers = findFrontierClusters(maxClusters: 1, minClusterSize: 2)
        return frontiers.isEmpty
    }
    
    /// Check if a straight-line path between two world points is free of occupied cells
    func isPathClear(fromX: Float, fromY: Float, toX: Float, toY: Float) -> Bool {
        lock.lock()
        defer { lock.unlock() }
        
        guard let start = worldToGrid(fromX, fromY),
              let end = worldToGrid(toX, toY) else { return false }
        
        var x = start.x
        var y = start.y
        let dx = abs(end.x - start.x)
        let dy = abs(end.y - start.y)
        let sx = start.x < end.x ? 1 : -1
        let sy = start.y < end.y ? 1 : -1
        var err = dx - dy
        
        while true {
            if x == end.x && y == end.y { break }
            
            if x >= 0 && x < gridSize && y >= 0 && y < gridSize {
                if cells[x][y] == CellState.occupied.rawValue {
                    return false
                }
            }
            
            let e2 = 2 * err
            if e2 > -dy {
                err -= dy
                x += sx
            }
            if e2 < dx {
                err += dx
                y += sy
            }
        }
        
        return true
    }
    
    /// Get a subsection of the grid for rendering
    /// Returns cell states, heights, and classifications in 2D arrays for the specified region
    func getRegion(centerX: Float, centerY: Float, radiusMeters: Float) -> (cells: [[CellState]], heights: [[Float]], classifications: [[MeshClassification]], originX: Float, originY: Float, cellSize: Float, minHeight: Float, maxHeight: Float) {
        lock.lock()
        defer { lock.unlock() }
        
        let cellRadius = Int(radiusMeters / cellSize)
        let regionSize = cellRadius * 2 + 1
        
        var region = Array(repeating: Array(repeating: CellState.unknown, count: regionSize), count: regionSize)
        var heights = Array(repeating: Array(repeating: Float(0), count: regionSize), count: regionSize)
        var classifs = Array(repeating: Array(repeating: MeshClassification.none, count: regionSize), count: regionSize)
        
        guard let center = worldToGrid(centerX, centerY) else {
            let origin = gridToWorld(gridRadius - cellRadius, gridRadius - cellRadius)
            return (region, heights, classifs, origin.x, origin.y, cellSize, globalMinHeight, globalMaxHeight)
        }
        
        for dx in -cellRadius...cellRadius {
            for dy in -cellRadius...cellRadius {
                let gx = center.x + dx
                let gy = center.y + dy
                let rx = dx + cellRadius
                let ry = dy + cellRadius
                
                if gx >= 0 && gx < gridSize && gy >= 0 && gy < gridSize {
                    region[rx][ry] = CellState(rawValue: cells[gx][gy]) ?? .unknown
                    classifs[rx][ry] = MeshClassification(rawValue: classifications[gx][gy]) ?? .none
                    // Use max height for this cell
                    if maxHeights[gx][gy] > -Float.greatestFiniteMagnitude {
                        heights[rx][ry] = maxHeights[gx][gy]
                    }
                }
            }
        }
        
        let origin = gridToWorld(center.x - cellRadius, center.y - cellRadius)
        return (region, heights, classifs, origin.x, origin.y, cellSize, globalMinHeight, globalMaxHeight)
    }
    
    // MARK: - Update Device Position
    
    func updateDevicePosition(transform: simd_float4x4) {
        let position = transform.columns.3
        
        // Extract heading from rotation matrix (yaw around Y axis)
        let forward = simd_float3(-transform.columns.2.x, -transform.columns.2.y, -transform.columns.2.z)
        let heading = atan2(forward.x, forward.z)
        
        devicePosition = DevicePosition(
            x: position.x,
            y: position.z,  // Use Z as Y in 2D (ARKit Y is up)
            z: position.y,  // Store actual height
            heading: heading
        )
    }
}
