//
//  NamedPersonStore.swift
//  RoboCar
//
//  Persists named people as OSNet ReID embeddings on disk so the robot can
//  recognise and follow a person by name across app launches. A person is
//  named by tapping their bounding box and entering a name; their appearance
//  is stored as a gallery of up to maxGallerySize embeddings. Matching uses
//  the maximum cosine similarity across the gallery so pose/lighting variation
//  is covered by multiple samples rather than a single averaged embedding.
//

import Foundation

/// A single saved person: a stable identity name plus a gallery of ReID
/// embeddings captured over time. Matching takes the max similarity across
/// all gallery entries.
struct NamedPerson: Codable {
    var name: String
    /// Gallery of L2-normalised 512-dim OSNet embeddings (newest at the end).
    var embeddings: [[Float]]
    var createdAt: Date
    var updatedAt: Date

    static let maxGallerySize = 8

    mutating func addEmbedding(_ embedding: [Float]) {
        embeddings.append(embedding)
        if embeddings.count > Self.maxGallerySize {
            embeddings.removeFirst()
        }
    }

    func bestSimilarity(to query: [Float]) -> Float {
        embeddings.map { NamedPersonStore.cosineSimilarity($0, query) }.max() ?? 0
    }

    // Backward-compat decoder: old format stored a single `embedding` key.
    enum CodingKeys: String, CodingKey {
        case name, embeddings, embedding, createdAt, updatedAt
    }

    init(name: String, embeddings: [[Float]], createdAt: Date, updatedAt: Date) {
        self.name = name
        self.embeddings = embeddings
        self.createdAt = createdAt
        self.updatedAt = updatedAt
    }

    init(from decoder: Decoder) throws {
        let c = try decoder.container(keyedBy: CodingKeys.self)
        name = try c.decode(String.self, forKey: .name)
        createdAt = try c.decode(Date.self, forKey: .createdAt)
        updatedAt = try c.decode(Date.self, forKey: .updatedAt)
        if let embs = try? c.decode([[Float]].self, forKey: .embeddings), !embs.isEmpty {
            embeddings = embs
        } else if let emb = try? c.decode([Float].self, forKey: .embedding), !emb.isEmpty {
            embeddings = [emb]
        } else {
            embeddings = []
        }
    }

    func encode(to encoder: Encoder) throws {
        var c = encoder.container(keyedBy: CodingKeys.self)
        try c.encode(name, forKey: .name)
        try c.encode(embeddings, forKey: .embeddings)
        try c.encode(createdAt, forKey: .createdAt)
        try c.encode(updatedAt, forKey: .updatedAt)
    }
}

/// Disk-backed store of named people. Thread-safe via a serial queue.
final class NamedPersonStore {

    static let shared = NamedPersonStore()

    /// Minimum cosine similarity required to consider an embedding a match.
    var matchThreshold: Float = 0.60

    private let queue = DispatchQueue(label: "com.robocar.namedpersonstore")
    private var people: [NamedPerson] = []

    private let fileURL: URL = {
        let dir = FileManager.default.urls(for: .applicationSupportDirectory, in: .userDomainMask).first
            ?? FileManager.default.temporaryDirectory
        try? FileManager.default.createDirectory(at: dir, withIntermediateDirectories: true)
        return dir.appendingPathComponent("named_people.json")
    }()

    private init() {
        load()
    }

    // MARK: - Public API

    /// All saved people (snapshot copy).
    var all: [NamedPerson] {
        queue.sync { people }
    }

    /// Returns the saved name whose gallery best matches `embedding`,
    /// or `nil` if no entry exceeds `matchThreshold`.
    func name(matching embedding: [Float]) -> String? {
        bestMatch(for: embedding)?.name
    }

    /// Returns the best-matching saved person (and similarity) for an embedding.
    func bestMatch(for embedding: [Float]) -> (name: String, similarity: Float)? {
        queue.sync {
            var best: (name: String, similarity: Float)?
            for person in people {
                let sim = person.bestSimilarity(to: embedding)
                if sim >= matchThreshold && (best == nil || sim > best!.similarity) {
                    best = (person.name, sim)
                }
            }
            return best
        }
    }

    /// Saves or updates a person. If a person with `name` already exists, the new
    /// embedding is added to their gallery (up to maxGallerySize); otherwise a new
    /// entry is created.
    func save(name: String, embedding: [Float]) {
        let trimmed = name.trimmingCharacters(in: .whitespacesAndNewlines)
        guard !trimmed.isEmpty, !embedding.isEmpty else { return }
        queue.sync {
            let now = Date()
            if let idx = people.firstIndex(where: { $0.name.caseInsensitiveCompare(trimmed) == .orderedSame }) {
                people[idx].addEmbedding(embedding)
                people[idx].name = trimmed
                people[idx].updatedAt = now
            } else {
                people.append(NamedPerson(name: trimmed, embeddings: [embedding], createdAt: now, updatedAt: now))
            }
            persist()
        }
    }

    /// Deletes the saved person with the given name (case-insensitive).
    func delete(name: String) {
        queue.sync {
            people.removeAll { $0.name.caseInsensitiveCompare(name) == .orderedSame }
            persist()
        }
    }

    /// Returns a representative (centroid) embedding for a named person, or `nil`
    /// if no entry exists. Callers that need a single vector for further matching
    /// should prefer `bestMatch(for:)` directly.
    func embedding(forName name: String) -> [Float]? {
        queue.sync {
            guard let person = people.first(where: { $0.name.caseInsensitiveCompare(name) == .orderedSame }),
                  !person.embeddings.isEmpty else { return nil }
            let dim = person.embeddings[0].count
            guard dim > 0 else { return nil }
            var centroid = [Float](repeating: 0, count: dim)
            for emb in person.embeddings {
                for i in 0..<dim { centroid[i] += emb[i] }
            }
            let n = Float(person.embeddings.count)
            centroid = centroid.map { $0 / n }
            let norm = sqrtf(centroid.reduce(0) { $0 + $1 * $1 })
            if norm > 1e-6 { centroid = centroid.map { $0 / norm } }
            return centroid
        }
    }

    // MARK: - Persistence

    private func load() {
        guard let data = try? Data(contentsOf: fileURL) else { return }
        if let decoded = try? JSONDecoder().decode([NamedPerson].self, from: data) {
            people = decoded
        }
    }

    /// Must be called on `queue`.
    private func persist() {
        guard let data = try? JSONEncoder().encode(people) else { return }
        try? data.write(to: fileURL, options: .atomic)
    }

    // MARK: - Similarity

    /// Cosine similarity for L2-normalised vectors (dot product). Falls back to
    /// full cosine if magnitudes differ from unit length.
    static func cosineSimilarity(_ a: [Float], _ b: [Float]) -> Float {
        guard a.count == b.count, !a.isEmpty else { return 0 }
        var dot: Float = 0
        var na: Float = 0
        var nb: Float = 0
        for i in 0..<a.count {
            dot += a[i] * b[i]
            na += a[i] * a[i]
            nb += b[i] * b[i]
        }
        let denom = (na.squareRoot() * nb.squareRoot())
        guard denom > 1e-6 else { return 0 }
        return dot / denom
    }
}
