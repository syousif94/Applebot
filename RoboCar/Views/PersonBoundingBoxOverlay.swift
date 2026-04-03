//
//  PersonBoundingBoxOverlay.swift
//  RoboCar
//
//  Draws colored bounding boxes around detected people, overlaid on the AR
//  camera view.  Each person gets a deterministic color derived from their
//  stable UUID so colors stay consistent across frames.
//

import UIKit
import simd

/// Lightweight struct passed from the view controller so the overlay doesn't
/// need to know about PersonTracker directly.
struct PersonBoxInfo {
    let id: UUID
    /// Bounding box in **Vision normalised coordinates** (origin at bottom-left,
    /// values 0-1, relative to landscape-right image orientation).
    let boundingBox: CGRect
    /// `true` when this person is the actively tracked target.
    let isActive: Bool
    /// `true` when this person is showing the activation gesture.
    let isGesturing: Bool
    /// Short display label, e.g. first 4 chars of the UUID.
    let label: String
    /// World position in occupancy-grid convention (ARKit X, ARKit Z). Nil if depth unavailable.
    let worldPosition: simd_float2?
}

/// A transparent `UIView` that draws coloured bounding boxes around detected
/// people.  Must be added as a subview of `ARView` so that its coordinate
/// space matches the camera image.
class PersonBoundingBoxOverlay: UIView {

    // MARK: - Public data

    /// Set this every time detected people change, then call `setNeedsDisplay()`.
    var people: [PersonBoxInfo] = []

    // MARK: - Colour palette

    /// Fixed palette of 12 perceptually-distinct colours (enough for a handful
    /// of people; the hash just picks an index).
    private static let palette: [UIColor] = [
        UIColor(red: 0.12, green: 0.72, blue: 0.96, alpha: 1),   // cyan
        UIColor(red: 1.00, green: 0.42, blue: 0.42, alpha: 1),   // red
        UIColor(red: 0.30, green: 0.90, blue: 0.40, alpha: 1),   // green
        UIColor(red: 1.00, green: 0.78, blue: 0.20, alpha: 1),   // yellow
        UIColor(red: 0.70, green: 0.40, blue: 1.00, alpha: 1),   // purple
        UIColor(red: 1.00, green: 0.55, blue: 0.15, alpha: 1),   // orange
        UIColor(red: 0.95, green: 0.45, blue: 0.75, alpha: 1),   // pink
        UIColor(red: 0.40, green: 0.85, blue: 0.75, alpha: 1),   // teal
        UIColor(red: 0.55, green: 0.72, blue: 1.00, alpha: 1),   // light blue
        UIColor(red: 0.85, green: 0.70, blue: 0.45, alpha: 1),   // tan
        UIColor(red: 0.60, green: 0.90, blue: 0.20, alpha: 1),   // lime
        UIColor(red: 0.80, green: 0.60, blue: 0.90, alpha: 1),   // lavender
    ]

    /// Deterministic colour for a given UUID (consistent across frames).
    private func color(for id: UUID) -> UIColor {
        let hash = abs(id.hashValue)
        return Self.palette[hash % Self.palette.count]
    }

    // MARK: - Init

    override init(frame: CGRect) {
        super.init(frame: frame)
        isOpaque = false
        isUserInteractionEnabled = false
        backgroundColor = .clear
    }

    required init?(coder: NSCoder) { fatalError() }

    // MARK: - Drawing

    override func draw(_ rect: CGRect) {
        guard let ctx = UIGraphicsGetCurrentContext(), !people.isEmpty else { return }

        let viewWidth  = bounds.width
        let viewHeight = bounds.height

        for person in people {
            let box = person.boundingBox

            // Convert from Vision normalised coordinates to UIKit screen
            // coordinates.  Vision uses origin at **bottom-left** with values
            // 0-1.  Because we pass `orientation: .right` when creating the
            // VNImageRequestHandler, Vision already accounts for the camera
            // rotation and returns boxes in portrait orientation.  We only
            // need to flip Y (bottom-left → top-left).
            let screenRect = CGRect(
                x: box.origin.x * viewWidth,
                y: (1 - box.origin.y - box.size.height) * viewHeight,
                width: box.size.width * viewWidth,
                height: box.size.height * viewHeight
            )

            let baseColor = person.isActive ? UIColor.white : color(for: person.id)

            // Box line
            let lineWidth: CGFloat = person.isActive ? 3.0 : 2.0
            ctx.setStrokeColor(baseColor.cgColor)
            ctx.setLineWidth(lineWidth)
            ctx.stroke(screenRect)

            // Semi-transparent fill when gesturing
            if person.isGesturing {
                ctx.setFillColor(baseColor.withAlphaComponent(0.15).cgColor)
                ctx.fill(screenRect)
            }

            // Label background + text
            let labelText = person.label as NSString
            let fontSize: CGFloat = 12
            let font = UIFont.systemFont(ofSize: fontSize, weight: .semibold)
            let attrs: [NSAttributedString.Key: Any] = [
                .font: font,
                .foregroundColor: UIColor.white,
            ]
            let textSize = labelText.size(withAttributes: attrs)
            let padding: CGFloat = 4
            let labelBG = CGRect(
                x: screenRect.minX,
                y: screenRect.minY - textSize.height - padding * 2,
                width: textSize.width + padding * 2,
                height: textSize.height + padding * 2
            )
            ctx.setFillColor(baseColor.withAlphaComponent(0.8).cgColor)
            ctx.fill(labelBG)

            // Draw text
            let textOrigin = CGPoint(x: labelBG.minX + padding, y: labelBG.minY + padding)
            labelText.draw(at: textOrigin, withAttributes: attrs)

            // Active indicator — draw a small "FOLLOWING" badge below the label
            if person.isActive {
                let badge = "FOLLOWING" as NSString
                let badgeFont = UIFont.systemFont(ofSize: 10, weight: .bold)
                let badgeAttrs: [NSAttributedString.Key: Any] = [
                    .font: badgeFont,
                    .foregroundColor: UIColor.black,
                ]
                let badgeSize = badge.size(withAttributes: badgeAttrs)
                let badgeBG = CGRect(
                    x: screenRect.minX,
                    y: screenRect.minY,
                    width: badgeSize.width + padding * 2,
                    height: badgeSize.height + padding
                )
                ctx.setFillColor(baseColor.cgColor)
                ctx.fill(badgeBG)
                badge.draw(at: CGPoint(x: badgeBG.minX + padding, y: badgeBG.minY + padding / 2),
                           withAttributes: badgeAttrs)
            }
        }
    }
}
