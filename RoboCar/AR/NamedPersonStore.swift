//
//  NamedPersonStore.swift
//  RoboCar
//
//  Persists named people as OSNet ReID embeddings on disk so the robot can
//  recognise and follow a person by name across app launches. A person is
//  named by tapping their bounding box and entering a name; their current
//  embedding is stored. Matching uses cosine similarity (embeddings are
//  L2-normalised, so a dot product suffices).
//

import Foundation

/// A single saved person: a stable identity name plus one or more ReID
/// embeddings captured over time (averaged for robustness).
struct NamedPerson: Codable {
    var name: String
    /// L2-normalised 512-dim OSNet embedding representing this person.
    var embedding: [Float]
    /// When this entry was created.
    var createdAt: Date
    /// When the embedding was last refreshed.
    var updatedAt: Date
}

/// Disk-backed store of named people. Thread-safe via a serial queue.
final class NamedPersonStore {

    static let shared = NamedPersonStore()

    /// Minimum cosine similarity required to consider an embedding a match.
    var matchThreshold: Float = 0.55

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

    /// Returns the saved name whose stored embedding best matches `embedding`,
    /// or `nil` if no entry exceeds `matchThreshold`.
    func name(matching embedding: [Float]) -> String? {
        bestMatch(for: embedding)?.name
    }

    /// Returns the best-matching saved person (and similarity) for an embedding.
    func bestMatch(for embedding: [Float]) -> (name: String, similarity: Float)? {
        queue.sync {
            var best: (name: String, similarity: Float)?
            for person in people {
                let sim = Self.cosineSimilarity(person.embedding, embedding)
                if sim >= matchThreshold && (best == nil || sim > best!.similarity) {
                    best = (person.name, sim)
                }
            }
            return best
        }
    }

    /// Saves or renames a person. If a person with `name` already exists, their
    /// embedding is refreshed; otherwise a new entry is created.
    func save(name: String, embedding: [Float]) {
        let trimmed = name.trimmingCharacters(in: .whitespacesAndNewlines)
        guard !trimmed.isEmpty, !embedding.isEmpty else { return }
        queue.sync {
            let now = Date()
            if let idx = people.firstIndex(where: { $0.name.caseInsensitiveCompare(trimmed) == .orderedSame }) {
                people[idx].embedding = embedding
                people[idx].name = trimmed
                people[idx].updatedAt = now
            } else {
                people.append(NamedPerson(name: trimmed, embedding: embedding, createdAt: now, updatedAt: now))
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

    /// Returns the stored embedding for a named person, if any.
    func embedding(forName name: String) -> [Float]? {
        queue.sync {
            people.first { $0.name.caseInsensitiveCompare(name) == .orderedSame }?.embedding
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
