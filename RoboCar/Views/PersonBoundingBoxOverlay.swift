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
    /// Short display label, e.g. first 4 chars of the UUID.
    let label: String
    /// World position in occupancy-grid convention (ARKit X, ARKit Z). Nil if depth unavailable.
    let worldPosition: simd_float2?
    /// Saved name for this person if their embedding matches a stored entry.
    var name: String? = nil
}

/// A transparent `UIView` that draws coloured bounding boxes around detected
/// people.  Must be added as a subview of `ARView` so that its coordinate
/// space matches the camera image.
class PersonBoundingBoxOverlay: UIView {

    // MARK: - Public data

    /// Set this every time detected people change, then call `setNeedsDisplay()`.
    var people: [PersonBoxInfo] = []

    /// Called when the body of a person's box is tapped — start following them.
    var onTapBody: ((UUID) -> Void)?
    /// Called when a person's name/ID label is tapped — prompt to name/rename.
    var onTapName: ((UUID) -> Void)?
    /// Called when the "✕" next to a saved name is tapped — delete the saved person.
    var onTapDelete: ((UUID) -> Void)?

    /// Cached hit targets computed during the last `draw(_:)` pass, in screen
    /// coordinates, used for tap routing.
    private struct HitTarget {
        let id: UUID
        let bodyRect: CGRect
        let labelRect: CGRect
        let deleteRect: CGRect?
    }
    private var hitTargets: [HitTarget] = []

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
        isUserInteractionEnabled = true
        backgroundColor = .clear
    }

    required init?(coder: NSCoder) { fatalError() }

    // MARK: - Hit testing

    /// Only intercept touches that land on a known box/label/delete target so
    /// that taps elsewhere fall through to the AR view and other controls.
    override func hitTest(_ point: CGPoint, with event: UIEvent?) -> UIView? {
        for target in hitTargets {
            if target.deleteRect?.contains(point) == true { return self }
            if target.labelRect.contains(point) { return self }
            if target.bodyRect.contains(point) { return self }
        }
        return nil
    }

    override func touchesEnded(_ touches: Set<UITouch>, with event: UIEvent?) {
        guard let point = touches.first?.location(in: self) else { return }
        // Delete badge takes priority, then label (rename), then body (follow).
        for target in hitTargets {
            if let del = target.deleteRect, del.contains(point) {
                onTapDelete?(target.id)
                return
            }
        }
        for target in hitTargets {
            if target.labelRect.contains(point) {
                onTapName?(target.id)
                return
            }
        }
        for target in hitTargets {
            if target.bodyRect.contains(point) {
                onTapBody?(target.id)
                return
            }
        }
    }

    // MARK: - Drawing

    override func draw(_ rect: CGRect) {
        hitTargets.removeAll(keepingCapacity: true)
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

            // Label background + text — show the saved name if present, else the
            // short UUID label.
            let isNamed = (person.name != nil)
            let labelText = (person.name ?? person.label) as NSString
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

            // Delete badge ("✕") next to the name, only for saved people.
            var deleteRect: CGRect?
            if isNamed {
                let xText = "✕" as NSString
                let xFont = UIFont.systemFont(ofSize: fontSize, weight: .bold)
                let xAttrs: [NSAttributedString.Key: Any] = [
                    .font: xFont,
                    .foregroundColor: UIColor.white,
                ]
                let xSize = xText.size(withAttributes: xAttrs)
                let xBG = CGRect(
                    x: labelBG.maxX + 4,
                    y: labelBG.minY,
                    width: xSize.width + padding * 2,
                    height: labelBG.height
                )
                ctx.setFillColor(UIColor.systemRed.withAlphaComponent(0.9).cgColor)
                ctx.fill(xBG)
                xText.draw(at: CGPoint(x: xBG.minX + padding, y: xBG.minY + padding),
                           withAttributes: xAttrs)
                deleteRect = xBG.insetBy(dx: -6, dy: -6)
            }

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

            // Record hit targets (slightly expanded for easier tapping).
            hitTargets.append(HitTarget(
                id: person.id,
                bodyRect: screenRect,
                labelRect: labelBG.insetBy(dx: -6, dy: -6),
                deleteRect: deleteRect
            ))
        }
    }
}

