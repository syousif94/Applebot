//
//  HandJointOverlayView.swift
//  RoboCar
//
//  Draws coloured dots for each detected hand joint, overlaid on the AR camera
//  view.  Joints are connected with lines to form a skeleton.  Each hand gets
//  a distinct colour so multiple hands are easy to distinguish.
//

import UIKit
import Vision

/// Lightweight container for one hand's joint positions in Vision normalised
/// coordinates.  Built on the background detection thread, consumed on main.
struct HandJointData {
    /// All recognised points keyed by joint name. Values are in Vision
    /// normalised coordinates (origin bottom-left, 0-1).
    let joints: [(name: VNHumanHandPoseObservation.JointName, point: CGPoint, confidence: Float)]
}

/// Transparent overlay that draws hand joint dots + skeleton lines.
class HandJointOverlayView: UIView {

    // MARK: - Public data

    /// Set every detection cycle, then call `setNeedsDisplay()`.
    var hands: [HandJointData] = []

    /// Minimum confidence to draw a joint.
    var minConfidence: Float = 0.3

    /// Dot radius in points.
    var dotRadius: CGFloat = 4

    // MARK: - Colour palette (one per hand)
    private static let handColors: [UIColor] = [
        UIColor(red: 0.12, green: 0.96, blue: 0.55, alpha: 1),   // green
        UIColor(red: 1.00, green: 0.42, blue: 0.42, alpha: 1),   // red
        UIColor(red: 0.40, green: 0.70, blue: 1.00, alpha: 1),   // blue
        UIColor(red: 1.00, green: 0.78, blue: 0.20, alpha: 1),   // yellow
        UIColor(red: 0.85, green: 0.45, blue: 0.95, alpha: 1),   // purple
        UIColor(red: 1.00, green: 0.60, blue: 0.20, alpha: 1),   // orange
    ]

    // MARK: - Skeleton connections

    /// Pairs of joints to connect with lines.
    private static let connections: [(VNHumanHandPoseObservation.JointName, VNHumanHandPoseObservation.JointName)] = [
        // Thumb
        (.wrist, .thumbCMC), (.thumbCMC, .thumbMP), (.thumbMP, .thumbIP), (.thumbIP, .thumbTip),
        // Index
        (.wrist, .indexMCP), (.indexMCP, .indexPIP), (.indexPIP, .indexDIP), (.indexDIP, .indexTip),
        // Middle
        (.wrist, .middleMCP), (.middleMCP, .middlePIP), (.middlePIP, .middleDIP), (.middleDIP, .middleTip),
        // Ring
        (.wrist, .ringMCP), (.ringMCP, .ringPIP), (.ringPIP, .ringDIP), (.ringDIP, .ringTip),
        // Little
        (.wrist, .littleMCP), (.littleMCP, .littlePIP), (.littlePIP, .littleDIP), (.littleDIP, .littleTip),
        // Palm cross-connections
        (.indexMCP, .middleMCP), (.middleMCP, .ringMCP), (.ringMCP, .littleMCP),
    ]

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
        guard let ctx = UIGraphicsGetCurrentContext(), !hands.isEmpty else { return }

        let viewWidth  = bounds.width
        let viewHeight = bounds.height

        for (handIndex, hand) in hands.enumerated() {
            let color = Self.handColors[handIndex % Self.handColors.count]

            // Build a lookup from joint name → screen point for this hand
            var screenPoints: [VNHumanHandPoseObservation.JointName: CGPoint] = [:]
            for joint in hand.joints where joint.confidence >= minConfidence {
                // Vision normalised coords → UIKit screen coords (Y-flip)
                let sx = joint.point.x * viewWidth
                let sy = (1 - joint.point.y) * viewHeight
                screenPoints[joint.name] = CGPoint(x: sx, y: sy)
            }

            // Draw skeleton lines
            ctx.setStrokeColor(color.withAlphaComponent(0.6).cgColor)
            ctx.setLineWidth(2.0)
            for (from, to) in Self.connections {
                guard let p1 = screenPoints[from], let p2 = screenPoints[to] else { continue }
                ctx.move(to: p1)
                ctx.addLine(to: p2)
                ctx.strokePath()
            }

            // Draw joint dots
            let dotColor = color.cgColor
            let borderColor = UIColor.black.withAlphaComponent(0.5).cgColor
            for (_, screenPt) in screenPoints {
                let dotRect = CGRect(
                    x: screenPt.x - dotRadius,
                    y: screenPt.y - dotRadius,
                    width: dotRadius * 2,
                    height: dotRadius * 2
                )
                // Border
                ctx.setFillColor(borderColor)
                ctx.fillEllipse(in: dotRect.insetBy(dx: -1, dy: -1))
                // Dot
                ctx.setFillColor(dotColor)
                ctx.fillEllipse(in: dotRect)
            }

            // Draw wrist label with gesture hint
            if let wristPt = screenPoints[.wrist] {
                let label = "H\(handIndex + 1)" as NSString
                let font = UIFont.systemFont(ofSize: 10, weight: .bold)
                let attrs: [NSAttributedString.Key: Any] = [
                    .font: font,
                    .foregroundColor: UIColor.white,
                ]
                let textSize = label.size(withAttributes: attrs)
                let bgRect = CGRect(
                    x: wristPt.x - textSize.width / 2 - 3,
                    y: wristPt.y + dotRadius + 2,
                    width: textSize.width + 6,
                    height: textSize.height + 4
                )
                ctx.setFillColor(color.withAlphaComponent(0.7).cgColor)
                ctx.fill(bgRect)
                label.draw(
                    at: CGPoint(x: bgRect.minX + 3, y: bgRect.minY + 2),
                    withAttributes: attrs
                )
            }
        }
    }

    // MARK: - Helpers

    /// Extract all joint positions from a `VNHumanHandPoseObservation` into
    /// a lightweight `HandJointData` struct. Can be called on any thread.
    static func extractJoints(from observation: VNHumanHandPoseObservation) -> HandJointData? {
        let allJointNames: [VNHumanHandPoseObservation.JointName] = [
            .wrist,
            .thumbCMC, .thumbMP, .thumbIP, .thumbTip,
            .indexMCP, .indexPIP, .indexDIP, .indexTip,
            .middleMCP, .middlePIP, .middleDIP, .middleTip,
            .ringMCP, .ringPIP, .ringDIP, .ringTip,
            .littleMCP, .littlePIP, .littleDIP, .littleTip,
        ]

        var joints: [(name: VNHumanHandPoseObservation.JointName, point: CGPoint, confidence: Float)] = []
        for name in allJointNames {
            guard let point = try? observation.recognizedPoint(name) else { continue }
            joints.append((name: name, point: point.location, confidence: Float(point.confidence)))
        }
        guard !joints.isEmpty else { return nil }
        return HandJointData(joints: joints)
    }
}
