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
    
    /// Update ID per cell to track which mesh generated it
    private var cellUpdates: [[UInt64]]
    
    /// Tracks which grid cells each mesh anchor currently owns.
    /// Encoded as  `gridX * gridSize + gridY`.
    private var anchorCells: [UUID: Set<Int>] = [:]
    
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
    
    // MARK: - Stuck Zone Memory
    
    /// Records a location+heading where the robot got stuck.
    struct StuckRecord {
        let gridX: Int
        let gridY: Int
        /// Heading the robot was trying to travel when it got stuck (radians)
        let heading: Float
        /// How many times the robot got stuck at (approximately) this spot+heading
        var count: Int = 1
        /// Timestamp of the most recent stuck event
        var lastTime: Date = Date()
    }
    
    /// All recorded stuck events. Keyed by a spatial hash (grid cell) so
    /// lookups during A* are fast.
    private var stuckRecords: [Int: [StuckRecord]] = [:]
    
    /// Radius (in grid cells) around a stuck point that receives a penalty.
    private let stuckPenaltyRadius: Int = 4  // ~20cm at 5cm/cell
    
    /// Base cost added per stuck event in the penalty zone.
    private let stuckPenaltyBase: Float = 40.0
    
    /// Two headings are considered "same direction" if within this angle (radians).
    private let stuckHeadingSimilarity: Float = .pi / 3.0  // 60°
    
    /// Time-to-live for a stuck record (seconds). Records older than this are
    /// ignored during pathfinding and pruned lazily.
    private let stuckRecordTTL: TimeInterval = 20.0
    
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
        self.cellUpdates = Array(repeating: Array(repeating: 0, count: size), count: size)
        
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
    
    /// Find the nearest free world-coordinate point to the given position.
    /// Returns the input point itself if it's already free, otherwise searches
    /// outward for the closest free cell and returns its world position.
    /// Returns nil only if no free cell exists within the search radius.
    func nearestFreeWorldPoint(x: Float, y: Float) -> (x: Float, y: Float)? {
        lock.lock()
        defer { lock.unlock() }
        
        guard let gridCoord = worldToGrid(x, y) else { return nil }
        
        // Already free — return as-is
        if cells[gridCoord.x][gridCoord.y] == CellState.free.rawValue {
            return (x, y)
        }
        
        // Search outward for nearest free cell
        if let nearest = findNearestFreeCell(gridX: gridCoord.x, gridY: gridCoord.y) {
            let world = gridToWorld(nearest.x, nearest.y)
            return (world.x, world.y)
        }
        return nil
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
    
    /// Get the state of a cell at world coordinates
    func getStateAtWorld(x: Float, y: Float) -> CellState {
        lock.lock()
        defer { lock.unlock() }
        
        guard let coord = worldToGrid(x, y) else { return .unknown }
        return CellState(rawValue: cells[coord.x][coord.y]) ?? .unknown
    }
    
    /// Get the max height of a cell at world coordinates, or nil if no height data
    func getMaxHeight(worldX: Float, worldY: Float) -> Float? {
        lock.lock()
        defer { lock.unlock() }
        
        guard let (gx, gy) = worldToGrid(worldX, worldY) else { return nil }
        let h = maxHeights[gx][gy]
        return h > -Float.greatestFiniteMagnitude ? h : nil
    }
    
    /// Thread-safe cell state by grid indices (convenience alias for ObstacleDetector)
    func getCellState(gridX: Int, gridY: Int) -> CellState {
        return getState(gridX: gridX, gridY: gridY)
    }
    
    /// Thread-safe classification lookup by grid indices
    func getCellClassification(gridX: Int, gridY: Int) -> MeshClassification {
        lock.lock()
        defer { lock.unlock() }
        guard gridX >= 0 && gridX < gridSize && gridY >= 0 && gridY < gridSize else {
            return .none
        }
        return MeshClassification(rawValue: classifications[gridX][gridY]) ?? .none
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
    func markFreeBatch(_ points: [(x: Float, y: Float)], updateId: UInt64? = nil) {
        lock.lock()
        defer { lock.unlock() }
        
        for point in points {
            guard let (gx, gy) = worldToGrid(point.x, point.y) else { continue }
            
            if let uid = updateId {
                if uid > cellUpdates[gx][gy] {
                    setStateUnsafe(gridX: gx, gridY: gy, state: .free)
                    cellUpdates[gx][gy] = uid
                } else if uid == cellUpdates[gx][gy] {
                    setStateUnsafe(gridX: gx, gridY: gy, state: .free)
                }
            } else {
                // Only mark as free if not already occupied (obstacles take priority)
                if cells[gx][gy] != CellState.occupied.rawValue {
                    setStateUnsafe(gridX: gx, gridY: gy, state: .free)
                }
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
    func markOccupiedBatch(_ points: [(x: Float, y: Float)], updateId: UInt64? = nil) {
        lock.lock()
        defer { lock.unlock() }
        
        for point in points {
            guard let (gx, gy) = worldToGrid(point.x, point.y) else { continue }
            
            if let uid = updateId {
                if uid > cellUpdates[gx][gy] {
                    setStateUnsafe(gridX: gx, gridY: gy, state: .occupied)
                    cellUpdates[gx][gy] = uid
                } else if uid == cellUpdates[gx][gy] {
                    setStateUnsafe(gridX: gx, gridY: gy, state: .occupied)
                }
            } else {
                setStateUnsafe(gridX: gx, gridY: gy, state: .occupied)
            }
        }
    }
    
    /// Mark multiple points as occupied with heights (batch operation)
    func markOccupiedBatchWithHeights(_ points: [(x: Float, y: Float, height: Float)], updateId: UInt64? = nil) {
        lock.lock()
        defer { lock.unlock() }
        
        for point in points {
            guard let (gx, gy) = worldToGrid(point.x, point.y) else { continue }
            
            if let uid = updateId {
                if uid > cellUpdates[gx][gy] {
                    setStateUnsafe(gridX: gx, gridY: gy, state: .occupied)
                    minHeights[gx][gy] = point.height
                    maxHeights[gx][gy] = point.height
                    if point.height < globalMinHeight { globalMinHeight = point.height }
                    if point.height > globalMaxHeight { globalMaxHeight = point.height }
                    cellUpdates[gx][gy] = uid
                } else if uid == cellUpdates[gx][gy] {
                    setStateUnsafe(gridX: gx, gridY: gy, state: .occupied)
                    if point.height < minHeights[gx][gy] { minHeights[gx][gy] = point.height }
                    if point.height > maxHeights[gx][gy] { maxHeights[gx][gy] = point.height }
                    if point.height < globalMinHeight { globalMinHeight = point.height }
                    if point.height > globalMaxHeight { globalMaxHeight = point.height }
                }
            } else {
                setStateUnsafe(gridX: gx, gridY: gy, state: .occupied)
                if point.height < minHeights[gx][gy] { minHeights[gx][gy] = point.height }
                if point.height > maxHeights[gx][gy] { maxHeights[gx][gy] = point.height }
                if point.height < globalMinHeight { globalMinHeight = point.height }
                if point.height > globalMaxHeight { globalMaxHeight = point.height }
            }
        }
    }
    
    /// Mark multiple points as occupied with heights and classification (batch operation)
    func markOccupiedBatchWithClassification(_ points: [(x: Float, y: Float, height: Float, classification: MeshClassification)], updateId: UInt64? = nil) {
        lock.lock()
        defer { lock.unlock() }
        
        for point in points {
            guard let (gx, gy) = worldToGrid(point.x, point.y) else { continue }
            
            if let uid = updateId {
                if uid > cellUpdates[gx][gy] {
                    setStateUnsafe(gridX: gx, gridY: gy, state: .occupied)
                    classifications[gx][gy] = point.classification.rawValue
                    minHeights[gx][gy] = point.height
                    maxHeights[gx][gy] = point.height
                    if point.height < globalMinHeight { globalMinHeight = point.height }
                    if point.height > globalMaxHeight { globalMaxHeight = point.height }
                    cellUpdates[gx][gy] = uid
                } else if uid == cellUpdates[gx][gy] {
                    setStateUnsafe(gridX: gx, gridY: gy, state: .occupied)
                    if point.classification != .none || classifications[gx][gy] == MeshClassification.none.rawValue {
                        classifications[gx][gy] = point.classification.rawValue
                    }
                    if point.height < minHeights[gx][gy] { minHeights[gx][gy] = point.height }
                    if point.height > maxHeights[gx][gy] { maxHeights[gx][gy] = point.height }
                    if point.height < globalMinHeight { globalMinHeight = point.height }
                    if point.height > globalMaxHeight { globalMaxHeight = point.height }
                }
            } else {
                setStateUnsafe(gridX: gx, gridY: gy, state: .occupied)
                if point.classification != .none || classifications[gx][gy] == MeshClassification.none.rawValue {
                    classifications[gx][gy] = point.classification.rawValue
                }
                if point.height < minHeights[gx][gy] { minHeights[gx][gy] = point.height }
                if point.height > maxHeights[gx][gy] { maxHeights[gx][gy] = point.height }
                if point.height < globalMinHeight { globalMinHeight = point.height }
                if point.height > globalMaxHeight { globalMaxHeight = point.height }
            }
        }
    }
    
    /// Atomically replace all grid data for a mesh anchor.
    /// Clears any cells the anchor previously owned, writes new floor and obstacle
    /// data, and records the new cell set for future replacement.
    func replaceAnchorData(
        anchorId: UUID,
        obstaclePoints: [(x: Float, y: Float, height: Float, classification: MeshClassification)],
        floorPoints: [(x: Float, y: Float, classification: MeshClassification)],
        updateId: UInt64
    ) {
        lock.lock()
        defer { lock.unlock() }
        
        // 1. Clear cells from the previous version of this anchor
        if let oldCells = anchorCells.removeValue(forKey: anchorId) {
            for encoded in oldCells {
                let gx = encoded / gridSize
                let gy = encoded % gridSize
                guard gx >= 0 && gx < gridSize && gy >= 0 && gy < gridSize else { continue }
                setStateUnsafe(gridX: gx, gridY: gy, state: .unknown)
                classifications[gx][gy] = MeshClassification.none.rawValue
                minHeights[gx][gy] = Float.greatestFiniteMagnitude
                maxHeights[gx][gy] = -Float.greatestFiniteMagnitude
                cellUpdates[gx][gy] = 0
            }
        }
        
        var newCells = Set<Int>()
        
        // 2. Write floor / free cells first
        for point in floorPoints {
            guard let (gx, gy) = worldToGrid(point.x, point.y) else { continue }
            let encoded = gx * gridSize + gy
            if updateId > cellUpdates[gx][gy] {
                setStateUnsafe(gridX: gx, gridY: gy, state: .free)
                classifications[gx][gy] = point.classification.rawValue
                minHeights[gx][gy] = Float.greatestFiniteMagnitude
                maxHeights[gx][gy] = -Float.greatestFiniteMagnitude
                cellUpdates[gx][gy] = updateId
                newCells.insert(encoded)
            } else if updateId == cellUpdates[gx][gy] {
                setStateUnsafe(gridX: gx, gridY: gy, state: .free)
                if point.classification != .none || classifications[gx][gy] == MeshClassification.none.rawValue {
                    classifications[gx][gy] = point.classification.rawValue
                }
                newCells.insert(encoded)
            }
        }
        
        // 3. Write obstacle cells (overrides floor where they overlap)
        for point in obstaclePoints {
            guard let (gx, gy) = worldToGrid(point.x, point.y) else { continue }
            let encoded = gx * gridSize + gy
            if updateId > cellUpdates[gx][gy] {
                setStateUnsafe(gridX: gx, gridY: gy, state: .occupied)
                classifications[gx][gy] = point.classification.rawValue
                minHeights[gx][gy] = point.height
                maxHeights[gx][gy] = point.height
                if point.height < globalMinHeight { globalMinHeight = point.height }
                if point.height > globalMaxHeight { globalMaxHeight = point.height }
                cellUpdates[gx][gy] = updateId
                newCells.insert(encoded)
            } else if updateId == cellUpdates[gx][gy] {
                setStateUnsafe(gridX: gx, gridY: gy, state: .occupied)
                if point.classification != .none || classifications[gx][gy] == MeshClassification.none.rawValue {
                    classifications[gx][gy] = point.classification.rawValue
                }
                if point.height < minHeights[gx][gy] { minHeights[gx][gy] = point.height }
                if point.height > maxHeights[gx][gy] { maxHeights[gx][gy] = point.height }
                if point.height < globalMinHeight { globalMinHeight = point.height }
                if point.height > globalMaxHeight { globalMaxHeight = point.height }
                newCells.insert(encoded)
            }
        }
        
        anchorCells[anchorId] = newCells
    }
    
    /// Remove all grid data owned by a mesh anchor (e.g. when ARKit removes it).
    func removeAnchorData(_ anchorId: UUID) {
        lock.lock()
        defer { lock.unlock() }
        
        guard let oldCells = anchorCells.removeValue(forKey: anchorId) else { return }
        for encoded in oldCells {
            let gx = encoded / gridSize
            let gy = encoded % gridSize
            guard gx >= 0 && gx < gridSize && gy >= 0 && gy < gridSize else { continue }
            setStateUnsafe(gridX: gx, gridY: gy, state: .unknown)
            classifications[gx][gy] = MeshClassification.none.rawValue
            minHeights[gx][gy] = Float.greatestFiniteMagnitude
            maxHeights[gx][gy] = -Float.greatestFiniteMagnitude
            cellUpdates[gx][gy] = 0
        }
    }
    
    /// Mark multiple points as free with classification (batch operation)
    func markFreeBatchWithClassification(_ points: [(x: Float, y: Float, classification: MeshClassification)], updateId: UInt64? = nil) {
        lock.lock()
        defer { lock.unlock() }
        
        for point in points {
            guard let (gx, gy) = worldToGrid(point.x, point.y) else { continue }
            
            if let uid = updateId {
                if uid > cellUpdates[gx][gy] {
                    setStateUnsafe(gridX: gx, gridY: gy, state: .free)
                    classifications[gx][gy] = point.classification.rawValue
                    // Reset heights for free space
                    minHeights[gx][gy] = Float.greatestFiniteMagnitude
                    maxHeights[gx][gy] = -Float.greatestFiniteMagnitude
                    cellUpdates[gx][gy] = uid
                } else if uid == cellUpdates[gx][gy] {
                    setStateUnsafe(gridX: gx, gridY: gy, state: .free)
                    if point.classification != .none || classifications[gx][gy] == MeshClassification.none.rawValue {
                        classifications[gx][gy] = point.classification.rawValue
                    }
                }
            } else {
                if cells[gx][gy] != CellState.occupied.rawValue {
                    setStateUnsafe(gridX: gx, gridY: gy, state: .free)
                    if point.classification != .none {
                        classifications[gx][gy] = point.classification.rawValue
                    }
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
                cellUpdates[x][y] = 0
            }
        }
        occupiedCount = 0
        freeCount = 0
        
        // Reset anchor cell tracking
        anchorCells.removeAll()
        
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
    
    // MARK: - Pathfinding (A*)
    
    /// Find the nearest free cell to a grid position using BFS.
    /// Must be called while lock is held.
    private func findNearestFreeCell(gridX: Int, gridY: Int, maxRadius: Int = 50) -> (x: Int, y: Int)? {
        for r in 1...maxRadius {
            for dx in -r...r {
                for dy in -r...r {
                    guard abs(dx) == r || abs(dy) == r else { continue } // perimeter only
                    let nx = gridX + dx
                    let ny = gridY + dy
                    guard nx >= 0 && nx < gridSize && ny >= 0 && ny < gridSize else { continue }
                    if cells[nx][ny] == CellState.free.rawValue {
                        return (nx, ny)
                    }
                }
            }
        }
        return nil
    }
    
    /// Find the nearest cell that is not occupied (free or unknown).
    /// Useful for greedy pathfinding where unknown cells are traversable.
    private func findNearestNonOccupiedCell(gridX: Int, gridY: Int, maxRadius: Int = 50) -> (x: Int, y: Int)? {
        for r in 1...maxRadius {
            for dx in -r...r {
                for dy in -r...r {
                    guard abs(dx) == r || abs(dy) == r else { continue }
                    let nx = gridX + dx
                    let ny = gridY + dy
                    guard nx >= 0 && nx < gridSize && ny >= 0 && ny < gridSize else { continue }
                    if cells[nx][ny] != CellState.occupied.rawValue {
                        return (nx, ny)
                    }
                }
            }
        }
        return nil
    }
    
    /// Check line-of-sight between two grid cells, respecting an obstacle buffer.
    /// Returns false if any cell along the line is occupied or within `bufferCells`
    /// of an occupied cell. Must be called while lock is held.
    private func lineOfSight(fromX: Int, fromY: Int, toX: Int, toY: Int) -> Bool {
        let bufferCells = 5  // 25cm buffer — preserves waypoints around corners
        var x = fromX
        var y = fromY
        let dx = abs(toX - fromX)
        let dy = abs(toY - fromY)
        let sx = fromX < toX ? 1 : -1
        let sy = fromY < toY ? 1 : -1
        var err = dx - dy
        
        while true {
            if x == toX && y == toY { return true }
            
            if x >= 0 && x < gridSize && y >= 0 && y < gridSize {
                if cells[x][y] == CellState.occupied.rawValue {
                    return false
                }
                // Check buffer zone around the current cell
                for bx in -bufferCells...bufferCells {
                    for by in -bufferCells...bufferCells {
                        if bx == 0 && by == 0 { continue }
                        let cx = x + bx, cy = y + by
                        if cx >= 0 && cx < gridSize && cy >= 0 && cy < gridSize {
                            if cells[cx][cy] == CellState.occupied.rawValue {
                                return false
                            }
                        }
                    }
                }
            } else {
                return false
            }
            
            let e2 = 2 * err
            if e2 > -dy { err -= dy; x += sx }
            if e2 < dx { err += dx; y += sy }
        }
    }
    
    /// Public thread-safe wrapper for line-of-sight check.
    /// Returns true if a straight line between two grid cells is clear of obstacles
    /// with adequate buffer for the car body.
    func hasLineOfSight(fromX: Int, fromY: Int, toX: Int, toY: Int) -> Bool {
        lock.lock()
        defer { lock.unlock() }
        return lineOfSight(fromX: fromX, fromY: fromY, toX: toX, toY: toY)
    }
    
    // MARK: - Stuck Zone API
    
    /// Record that the robot got stuck at the given world position while
    /// heading in `heading` direction.  Nearby duplicate headings are merged
    /// (count incremented) rather than creating new records.
    func recordStuckZone(worldX: Float, worldY: Float, heading: Float) {
        lock.lock()
        defer { lock.unlock() }
        
        guard let cell = worldToGrid(worldX, worldY) else { return }
        let k = cell.x * gridSize + cell.y
        
        // Check for existing record with similar heading in this or adjacent cells
        let searchRadius = 2  // merge within ~10cm
        for dx in -searchRadius...searchRadius {
            for dy in -searchRadius...searchRadius {
                let sx = cell.x + dx, sy = cell.y + dy
                guard sx >= 0 && sx < gridSize && sy >= 0 && sy < gridSize else { continue }
                let sk = sx * gridSize + sy
                if var records = stuckRecords[sk] {
                    for i in records.indices {
                        var angleDiff = records[i].heading - heading
                        while angleDiff >  .pi { angleDiff -= 2 * .pi }
                        while angleDiff < -.pi { angleDiff += 2 * .pi }
                        if fabsf(angleDiff) < stuckHeadingSimilarity {
                            records[i].count += 1
                            records[i].lastTime = Date()
                            stuckRecords[sk] = records
                            print("[OccGrid] Stuck zone reinforced at (\(sx),\(sy)) heading \(String(format: "%.1f°", heading * 180 / .pi)) count=\(records[i].count)")
                            return
                        }
                    }
                }
            }
        }
        
        // New record
        var records = stuckRecords[k] ?? []
        records.append(StuckRecord(gridX: cell.x, gridY: cell.y, heading: heading))
        stuckRecords[k] = records
        print("[OccGrid] New stuck zone at (\(cell.x),\(cell.y)) heading \(String(format: "%.1f°", heading * 180 / .pi))")
    }
    
    /// Compute the stuck penalty for a grid cell being approached from `approachHeading`.
    /// Must be called while `lock` is held.
    private func stuckZonePenalty(_ x: Int, _ y: Int) -> Float {
        var totalPenalty: Float = 0
        let now = Date()
        let r = stuckPenaltyRadius
        for dx in -r...r {
            for dy in -r...r {
                let sx = x + dx, sy = y + dy
                guard sx >= 0 && sx < gridSize && sy >= 0 && sy < gridSize else { continue }
                let sk = sx * gridSize + sy
                guard let records = stuckRecords[sk] else { continue }
                let cellDist = sqrtf(Float(dx * dx + dy * dy))
                guard cellDist <= Float(r) else { continue }
                // Distance falloff: full penalty at center, linear decay to edge
                let distFactor = 1.0 - (cellDist / Float(r + 1))
                for record in records {
                    let age = now.timeIntervalSince(record.lastTime)
                    guard age < stuckRecordTTL else { continue }
                    // Fade out penalty as the record ages
                    let ageFactor = Float(1.0 - age / stuckRecordTTL)
                    // Penalty scales with how many times stuck here
                    let countFactor = Float(min(record.count, 5))  // cap at 5x
                    totalPenalty += stuckPenaltyBase * countFactor * distFactor * ageFactor
                }
            }
        }
        return totalPenalty
    }
    
    /// Remove all stuck-zone records (e.g. when a new high-level goal is set).
    func clearStuckZones() {
        lock.lock()
        defer { lock.unlock() }
        stuckRecords.removeAll()
        print("[OccGrid] Stuck zones cleared")
    }
    
    /// Number of stuck records currently stored.
    var stuckZoneCount: Int {
        lock.lock()
        defer { lock.unlock() }
        return stuckRecords.values.reduce(0) { $0 + $1.count }
    }
    
    /// Find a path from start to goal using A* on the occupancy grid.
    /// Only traverses free cells, avoiding occupied and unknown cells.
    /// Returns an array of world-coordinate waypoints (smoothed), or empty if no path found.
    func findPath(fromX: Float, fromY: Float, toX: Float, toY: Float) -> [(x: Float, y: Float)] {
        lock.lock()
        defer { lock.unlock() }
        
        guard var start = worldToGrid(fromX, fromY),
              var goal = worldToGrid(toX, toY) else { return [] }
        
        // If start isn't free, find nearest free cell
        if cells[start.x][start.y] != CellState.free.rawValue {
            if let nearest = findNearestFreeCell(gridX: start.x, gridY: start.y) {
                start = nearest
            } else { return [] }
        }
        
        // If goal isn't free, find nearest free cell
        if cells[goal.x][goal.y] != CellState.free.rawValue {
            if let nearest = findNearestFreeCell(gridX: goal.x, gridY: goal.y) {
                goal = nearest
            } else { return [] }
        }
        
        if start.x == goal.x && start.y == goal.y { return [] }
        
        // Clearance: light proximity penalty near walls.
        // Real-time ObstacleDetector handles close-range avoidance at 15cm,
        // so the planner only needs a soft preference to stay away from walls.
        let clearanceCells = 3  // 15cm proximity zone at 5cm/cell
        let hardBlockRadius = 2 // 10cm — match car half-width for corridor clearance
        
        /// Returns a proximity penalty for (x,y): high within car half-width
        /// of an obstacle, lighter further out, 0 if far away.
        func obstaclePenalty(_ x: Int, _ y: Int) -> Float {
            var minR = Int.max
            for r in 1...clearanceCells {
                for ddx in -r...r {
                    for ddy in -r...r {
                        guard abs(ddx) == r || abs(ddy) == r else { continue } // perimeter only
                        let cx = x + ddx, cy = y + ddy
                        if cx >= 0 && cx < gridSize && cy >= 0 && cy < gridSize {
                            if cells[cx][cy] == CellState.occupied.rawValue {
                                if r <= hardBlockRadius {
                                    return 50.0  // discouraged — directly adjacent to wall
                                }
                                minR = min(minR, r)
                            }
                        }
                    }
                }
                if minR <= r { break } // found closest obstacle ring
            }
            if minR <= clearanceCells {
                return 10.0 * Float(clearanceCells + 1 - minR) / Float(max(clearanceCells - hardBlockRadius, 1))
            }
            return 0
        }
        
        // A* with 8-directional movement
        let sqrt2: Float = 1.41421356
        let directions: [(dx: Int, dy: Int, cost: Float)] = [
            (-1, 0, 1), (1, 0, 1), (0, -1, 1), (0, 1, 1),
            (-1, -1, sqrt2), (-1, 1, sqrt2), (1, -1, sqrt2), (1, 1, sqrt2)
        ]
        
        // Octile distance heuristic
        func heuristic(_ x: Int, _ y: Int) -> Float {
            let dx = Float(abs(x - goal.x))
            let dy = Float(abs(y - goal.y))
            return max(dx, dy) + (sqrt2 - 1) * min(dx, dy)
        }
        
        func key(_ x: Int, _ y: Int) -> Int { x * gridSize + y }
        
        // Open set as a simple sorted array (adequate for typical indoor paths)
        struct AStarNode: Comparable {
            let x: Int
            let y: Int
            let f: Float
            let g: Float
            static func < (lhs: AStarNode, rhs: AStarNode) -> Bool { lhs.f < rhs.f }
        }
        
        var openList: [AStarNode] = []
        var gScore: [Int: Float] = [:]
        var cameFrom: [Int: Int] = [:]
        var closedSet = Set<Int>()
        
        let startKey = key(start.x, start.y)
        let goalKey = key(goal.x, goal.y)
        let startH = heuristic(start.x, start.y)
        gScore[startKey] = 0
        openList.append(AStarNode(x: start.x, y: start.y, f: startH, g: 0))
        
        let maxIterations = 100_000
        var iterations = 0
        
        while !openList.isEmpty && iterations < maxIterations {
            iterations += 1
            
            // Pop node with lowest f
            var minIdx = 0
            for i in 1..<openList.count {
                if openList[i].f < openList[minIdx].f { minIdx = i }
            }
            let current = openList.remove(at: minIdx)
            let currentKey = key(current.x, current.y)
            
            if currentKey == goalKey {
                // Reconstruct path
                var gridPath: [(x: Int, y: Int)] = [(goal.x, goal.y)]
                var ck = goalKey
                while let parentKey = cameFrom[ck] {
                    let px = parentKey / gridSize
                    let py = parentKey % gridSize
                    gridPath.append((px, py))
                    ck = parentKey
                }
                gridPath.reverse()
                
                // Smooth path using line-of-sight optimization, then round corners
                let smoothed = roundCorners(smoothPath(gridPath))
                
                // Convert to world coordinates
                return smoothed.map { cell in
                    let world = gridToWorld(cell.x, cell.y)
                    return (x: world.x, y: world.y)
                }
            }
            
            if closedSet.contains(currentKey) { continue }
            closedSet.insert(currentKey)
            
            for dir in directions {
                let nx = current.x + dir.dx
                let ny = current.y + dir.dy
                
                guard nx >= 0 && nx < gridSize && ny >= 0 && ny < gridSize else { continue }
                
                let nKey = key(nx, ny)
                if closedSet.contains(nKey) { continue }
                
                // Only traverse free cells (or the goal itself)
                if cells[nx][ny] != CellState.free.rawValue && nKey != goalKey { continue }
                
                // For diagonal moves, check that both adjacent cardinal cells are free
                if dir.dx != 0 && dir.dy != 0 {
                    let cx1 = current.x + dir.dx
                    let cy1 = current.y
                    let cx2 = current.x
                    let cy2 = current.y + dir.dy
                    if cx1 >= 0 && cx1 < gridSize && cy1 >= 0 && cy1 < gridSize &&
                       cx2 >= 0 && cx2 < gridSize && cy2 >= 0 && cy2 < gridSize {
                        if cells[cx1][cy1] == CellState.occupied.rawValue ||
                           cells[cx2][cy2] == CellState.occupied.rawValue {
                            continue  // Can't cut corners around obstacles
                        }
                    }
                }
                
                let tentativeG = current.g + dir.cost + obstaclePenalty(nx, ny) + stuckZonePenalty(nx, ny)
                if tentativeG < (gScore[nKey] ?? .greatestFiniteMagnitude) {
                    gScore[nKey] = tentativeG
                    cameFrom[nKey] = currentKey
                    let f = tentativeG + heuristic(nx, ny)
                    openList.append(AStarNode(x: nx, y: ny, f: f, g: tentativeG))
                }
            }
        }
        
        return []  // No path found
    }
    
    /// Find a path allowing traversal through unknown cells (greedy fallback).
    /// Uses A* with unknown cells treated as high-cost passable terrain.
    /// This lets the robot navigate toward unexplored areas.
    func findPathGreedy(fromX: Float, fromY: Float, toX: Float, toY: Float) -> [(x: Float, y: Float)] {
        lock.lock()
        defer { lock.unlock() }
        
        guard var start = worldToGrid(fromX, fromY),
              var goal = worldToGrid(toX, toY) else { return [] }
        
        // Start can be on free or unknown cells (both are traversable in greedy mode).
        // Only snap when the cell is occupied (e.g. device drifted onto a wall cell).
        if cells[start.x][start.y] == CellState.occupied.rawValue {
            if let nearest = findNearestNonOccupiedCell(gridX: start.x, gridY: start.y) {
                start = nearest
            } else { return [] }
        }
        
        // Goal can be in unknown territory — only snap if occupied
        if cells[goal.x][goal.y] == CellState.occupied.rawValue {
            if let nearest = findNearestNonOccupiedCell(gridX: goal.x, gridY: goal.y) {
                goal = nearest
            } else { return [] }
        }
        
        if start.x == goal.x && start.y == goal.y { return [] }
        
        let clearanceCells = 3  // 15cm proximity zone at 5cm/cell
        let hardBlockRadius = 2 // 10cm — match car half-width for corridor clearance
        
        func obstaclePenalty(_ x: Int, _ y: Int) -> Float {
            var minR = Int.max
            for r in 1...clearanceCells {
                for ddx in -r...r {
                    for ddy in -r...r {
                        guard abs(ddx) == r || abs(ddy) == r else { continue }
                        let cx = x + ddx, cy = y + ddy
                        if cx >= 0 && cx < gridSize && cy >= 0 && cy < gridSize {
                            if cells[cx][cy] == CellState.occupied.rawValue {
                                if r <= hardBlockRadius {
                                    return 50.0  // discouraged — directly adjacent to wall
                                }
                                minR = min(minR, r)
                            }
                        }
                    }
                }
                if minR <= r { break }
            }
            if minR <= clearanceCells {
                return 10.0 * Float(clearanceCells + 1 - minR) / Float(max(clearanceCells - hardBlockRadius, 1))
            }
            return 0
        }
        
        let sqrt2: Float = 1.41421356
        let directions: [(dx: Int, dy: Int, cost: Float)] = [
            (-1, 0, 1), (1, 0, 1), (0, -1, 1), (0, 1, 1),
            (-1, -1, sqrt2), (-1, 1, sqrt2), (1, -1, sqrt2), (1, 1, sqrt2)
        ]
        
        // Cost penalty for traversing unknown cells — high enough to strongly
        // prefer known-free paths, but still allows exploration when no
        // free path exists.  Each unknown cell adds 5x its base cost.
        let unknownPenalty: Float = 5.0
        
        func heuristic(_ x: Int, _ y: Int) -> Float {
            let dx = Float(abs(x - goal.x))
            let dy = Float(abs(y - goal.y))
            return max(dx, dy) + (sqrt2 - 1) * min(dx, dy)
        }
        
        func key(_ x: Int, _ y: Int) -> Int { x * gridSize + y }
        
        struct AStarNode: Comparable {
            let x: Int, y: Int, f: Float, g: Float
            static func < (lhs: AStarNode, rhs: AStarNode) -> Bool { lhs.f < rhs.f }
        }
        
        var openList: [AStarNode] = []
        var gScore: [Int: Float] = [:]
        var cameFrom: [Int: Int] = [:]
        var closedSet = Set<Int>()
        
        let startKey = key(start.x, start.y)
        let goalKey = key(goal.x, goal.y)
        gScore[startKey] = 0
        openList.append(AStarNode(x: start.x, y: start.y, f: heuristic(start.x, start.y), g: 0))
        
        let maxIterations = 150_000
        var iterations = 0
        
        while !openList.isEmpty && iterations < maxIterations {
            iterations += 1
            
            var minIdx = 0
            for i in 1..<openList.count {
                if openList[i].f < openList[minIdx].f { minIdx = i }
            }
            let current = openList.remove(at: minIdx)
            let currentKey = key(current.x, current.y)
            
            if currentKey == goalKey {
                var gridPath: [(x: Int, y: Int)] = [(goal.x, goal.y)]
                var ck = goalKey
                while let parentKey = cameFrom[ck] {
                    let px = parentKey / gridSize
                    let py = parentKey % gridSize
                    gridPath.append((px, py))
                    ck = parentKey
                }
                gridPath.reverse()
                let smoothed = roundCorners(smoothPath(gridPath))
                return smoothed.map { cell in
                    let world = gridToWorld(cell.x, cell.y)
                    return (x: world.x, y: world.y)
                }
            }
            
            if closedSet.contains(currentKey) { continue }
            closedSet.insert(currentKey)
            
            for dir in directions {
                let nx = current.x + dir.dx
                let ny = current.y + dir.dy
                guard nx >= 0 && nx < gridSize && ny >= 0 && ny < gridSize else { continue }
                let nKey = key(nx, ny)
                if closedSet.contains(nKey) { continue }
                
                let cellState = CellState(rawValue: cells[nx][ny]) ?? .unknown
                
                // Block occupied cells (never traverse walls)
                if cellState == .occupied { continue }
                
                // For diagonal moves, check corners
                if dir.dx != 0 && dir.dy != 0 {
                    let cx1 = current.x + dir.dx, cy1 = current.y
                    let cx2 = current.x, cy2 = current.y + dir.dy
                    if cx1 >= 0 && cx1 < gridSize && cy1 >= 0 && cy1 < gridSize &&
                       cx2 >= 0 && cx2 < gridSize && cy2 >= 0 && cy2 < gridSize {
                        if cells[cx1][cy1] == CellState.occupied.rawValue ||
                           cells[cx2][cy2] == CellState.occupied.rawValue {
                            continue
                        }
                    }
                }
                
                // Unknown cells get a penalty but are traversable
                let extraCost: Float = (cellState == .unknown) ? unknownPenalty : 0
                let tentativeG = current.g + dir.cost + obstaclePenalty(nx, ny) + extraCost + stuckZonePenalty(nx, ny)
                
                if tentativeG < (gScore[nKey] ?? .greatestFiniteMagnitude) {
                    gScore[nKey] = tentativeG
                    cameFrom[nKey] = currentKey
                    let f = tentativeG + heuristic(nx, ny)
                    openList.append(AStarNode(x: nx, y: ny, f: f, g: tentativeG))
                }
            }
        }
        
        return []
    }
    
    /// Insert waypoints at sharp corners to route through corridor centers.
    /// Prevents the path from cutting corners at corridor entrances.
    /// Must be called while lock is held.
    private func roundCorners(_ path: [(x: Int, y: Int)]) -> [(x: Int, y: Int)] {
        guard path.count >= 3 else { return path }
        
        var result = [path[0]]
        
        for i in 1..<(path.count - 1) {
            let a = path[i - 1]
            let b = path[i]
            let c = path[i + 1]
            
            // Vectors for the two segments meeting at B
            let abx = Float(b.x - a.x), aby = Float(b.y - a.y)
            let bcx = Float(c.x - b.x), bcy = Float(c.y - b.y)
            let lenAB = sqrtf(abx * abx + aby * aby)
            let lenBC = sqrtf(bcx * bcx + bcy * bcy)
            
            guard lenAB > 2 && lenBC > 2 else {
                result.append(b)
                continue
            }
            
            // Turn angle at B
            let cosAngle = (abx * bcx + aby * bcy) / (lenAB * lenBC)
            let turnAngle = acosf(min(max(cosAngle, -1.0), 1.0))
            
            // Sharp turn > ~30° — check if we're near a corridor
            if turnAngle > 0.52 {
                // Perpendicular to outgoing direction (B→C)
                let ndx = bcx / lenBC, ndy = bcy / lenBC
                let perpX = -ndy, perpY = ndx
                
                // Scan perpendicular from B to find corridor walls
                let maxScan = 20  // 1m at 5cm/cell
                var leftDist = maxScan, rightDist = maxScan
                
                for d in 1...maxScan {
                    let sx = b.x + Int(roundf(Float(d) * perpX))
                    let sy = b.y + Int(roundf(Float(d) * perpY))
                    if sx < 0 || sx >= gridSize || sy < 0 || sy >= gridSize ||
                       cells[sx][sy] == CellState.occupied.rawValue {
                        leftDist = d
                        break
                    }
                }
                for d in 1...maxScan {
                    let sx = b.x - Int(roundf(Float(d) * perpX))
                    let sy = b.y - Int(roundf(Float(d) * perpY))
                    if sx < 0 || sx >= gridSize || sy < 0 || sy >= gridSize ||
                       cells[sx][sy] == CellState.occupied.rawValue {
                        rightDist = d
                        break
                    }
                }
                
                // If in a constrained area (wall within ~60cm on either side)
                if leftDist <= 12 || rightDist <= 12 {
                    // Shift B to the center of the corridor
                    let offset = Float(leftDist - rightDist) / 2.0
                    let cbx = b.x + Int(roundf(offset * perpX))
                    let cby = b.y + Int(roundf(offset * perpY))
                    
                    // Insert an approach midpoint halfway between A and centered-B
                    // so the robot arcs smoothly into the corridor center
                    let mx = (a.x + cbx) / 2
                    let my = (a.y + cby) / 2
                    
                    if mx >= 0 && mx < gridSize && my >= 0 && my < gridSize &&
                       cells[mx][my] != CellState.occupied.rawValue {
                        result.append((x: mx, y: my))
                    }
                    
                    // Use corridor-centered position for B
                    if cbx >= 0 && cbx < gridSize && cby >= 0 && cby < gridSize &&
                       cells[cbx][cby] != CellState.occupied.rawValue {
                        result.append((x: cbx, y: cby))
                        continue
                    }
                }
            }
            
            result.append(b)
        }
        
        result.append(path.last!)
        return result
    }
    
    /// Smooth a grid path by removing intermediate waypoints when line-of-sight exists.
    /// Must be called while lock is held.
    private func smoothPath(_ path: [(x: Int, y: Int)]) -> [(x: Int, y: Int)] {
        guard path.count > 2 else { return path }
        
        var smoothed = [path[0]]
        var current = 0
        
        while current < path.count - 1 {
            var farthest = current + 1
            // Try to skip as far ahead as possible with clear line of sight
            for i in stride(from: path.count - 1, to: current + 1, by: -1) {
                if lineOfSight(fromX: path[current].x, fromY: path[current].y,
                              toX: path[i].x, toY: path[i].y) {
                    farthest = i
                    break
                }
            }
            smoothed.append(path[farthest])
            current = farthest
        }
        
        return smoothed
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
