//
//  SceneLandmarkStore.swift
//  RoboCar

import Foundation

/// A named world-coordinate position remembered from the scene.
struct SceneLandmark {
    let name: String
    var worldX: Float
    var worldY: Float
}

/// Registry of named scene objects and their world positions.
/// Populated from two sources:
///   1. ARKit mesh classifications (door, table, seat/couch, window)
///   2. LLM camera-identified objects stored via StoreLandmarkTool
class SceneLandmarkStore {
    static let shared = SceneLandmarkStore()

    private var landmarks: [String: SceneLandmark] = [:]
    private let lock = NSLock()

    private init() {}

    // MARK: - Write

    func add(name: String, x: Float, y: Float) {
        let key = name.lowercased().trimmingCharacters(in: .whitespaces)
        lock.lock()
        landmarks[key] = SceneLandmark(name: name, worldX: x, worldY: y)
        lock.unlock()
    }

    // MARK: - Read

    func findBestMatch(for query: String) -> SceneLandmark? {
        let q = query.lowercased().trimmingCharacters(in: .whitespaces)
        lock.lock()
        defer { lock.unlock() }
        if let direct = landmarks[q] { return direct }
        for (key, landmark) in landmarks {
            if key.contains(q) || q.contains(key) { return landmark }
        }
        return nil
    }

    func summary() -> String {
        lock.lock()
        defer { lock.unlock() }
        guard !landmarks.isEmpty else { return "none" }
        return landmarks.values
            .map { String(format: "%@: (%.1f, %.1f)", $0.name, $0.worldX, $0.worldY) }
            .sorted()
            .joined(separator: "; ")
    }

    // MARK: - Mesh Classification Update

    /// Scan the occupancy grid for ARKit-classified surfaces and update landmarks.
    /// Call from a background thread — scans the full grid row by row.
    func updateMeshClassifications(from grid: OccupancyGrid) {
        let mappings: [(MeshClassification, String)] = [
            (.door,   "door"),
            (.seat,   "couch"),
            (.table,  "table"),
            (.window, "window"),
        ]
        for (classification, name) in mappings {
            if let centroid = grid.centroid(ofClassification: classification) {
                add(name: name, x: centroid.x, y: centroid.y)
            }
        }
    }
}
