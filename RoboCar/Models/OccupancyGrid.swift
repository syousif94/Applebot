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
    
    /// Clear all cells to unknown state
    func clear() {
        lock.lock()
        defer { lock.unlock() }
        
        for x in 0..<gridSize {
            for y in 0..<gridSize {
                cells[x][y] = CellState.unknown.rawValue
            }
        }
        occupiedCount = 0
        freeCount = 0
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
    
    /// Get a subsection of the grid for rendering
    /// Returns cell states in a 2D array for the specified region
    func getRegion(centerX: Float, centerY: Float, radiusMeters: Float) -> (cells: [[CellState]], heights: [[Float]], originX: Float, originY: Float, cellSize: Float, minHeight: Float, maxHeight: Float) {
        lock.lock()
        defer { lock.unlock() }
        
        let cellRadius = Int(radiusMeters / cellSize)
        let regionSize = cellRadius * 2 + 1
        
        var region = Array(repeating: Array(repeating: CellState.unknown, count: regionSize), count: regionSize)
        var heights = Array(repeating: Array(repeating: Float(0), count: regionSize), count: regionSize)
        
        guard let center = worldToGrid(centerX, centerY) else {
            let origin = gridToWorld(gridRadius - cellRadius, gridRadius - cellRadius)
            return (region, heights, origin.x, origin.y, cellSize, globalMinHeight, globalMaxHeight)
        }
        
        for dx in -cellRadius...cellRadius {
            for dy in -cellRadius...cellRadius {
                let gx = center.x + dx
                let gy = center.y + dy
                let rx = dx + cellRadius
                let ry = dy + cellRadius
                
                if gx >= 0 && gx < gridSize && gy >= 0 && gy < gridSize {
                    region[rx][ry] = CellState(rawValue: cells[gx][gy]) ?? .unknown
                    // Use max height for this cell
                    if maxHeights[gx][gy] > -Float.greatestFiniteMagnitude {
                        heights[rx][ry] = maxHeights[gx][gy]
                    }
                }
            }
        }
        
        let origin = gridToWorld(center.x - cellRadius, center.y - cellRadius)
        return (region, heights, origin.x, origin.y, cellSize, globalMinHeight, globalMaxHeight)
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
