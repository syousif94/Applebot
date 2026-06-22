//
//  RemotePersonBoxOverlay.swift
//  RoboCar
//
//  Controller-side overlay that draws tappable person outlines on top of the
//  remote video stream. Bounding boxes arrive in Vision-normalised coordinates
//  (origin bottom-left, 0-1) matching the host overlay. Because the video is
//  displayed with `.scaleAspectFill`, boxes are mapped into the aspect-fill
//  rect of the source video within this view's bounds.
//

import UIKit

class RemotePersonBoxOverlay: UIView {

    /// Latest people received from the host.
    var people: [RemotePersonBox] = [] {
        didSet { setNeedsDisplay() }
    }

    /// Native size of the incoming video frames, used for aspect-fill mapping.
    var videoSize: CGSize = .zero {
        didSet { if videoSize != oldValue { setNeedsDisplay() } }
    }

    /// Tap on a person's body → follow them.
    var onTapBody: ((String) -> Void)?
    /// Tap on a person's name/ID label → name or rename them.
    var onTapName: ((String) -> Void)?
    /// Tap on the "✕" next to a saved name → delete the saved person.
    var onTapDelete: ((String) -> Void)?

    private struct HitTarget {
        let id: String
        let bodyRect: CGRect
        let labelRect: CGRect
        let deleteRect: CGRect?
    }
    private var hitTargets: [HitTarget] = []

    private static let palette: [UIColor] = [
        UIColor(red: 0.12, green: 0.72, blue: 0.96, alpha: 1),
        UIColor(red: 1.00, green: 0.42, blue: 0.42, alpha: 1),
        UIColor(red: 0.30, green: 0.90, blue: 0.40, alpha: 1),
        UIColor(red: 1.00, green: 0.78, blue: 0.20, alpha: 1),
        UIColor(red: 0.70, green: 0.40, blue: 1.00, alpha: 1),
        UIColor(red: 1.00, green: 0.55, blue: 0.15, alpha: 1),
        UIColor(red: 0.95, green: 0.45, blue: 0.75, alpha: 1),
        UIColor(red: 0.40, green: 0.85, blue: 0.75, alpha: 1),
        UIColor(red: 0.55, green: 0.72, blue: 1.00, alpha: 1),
        UIColor(red: 0.85, green: 0.70, blue: 0.45, alpha: 1),
        UIColor(red: 0.60, green: 0.90, blue: 0.20, alpha: 1),
        UIColor(red: 0.80, green: 0.60, blue: 0.90, alpha: 1),
    ]

    private func color(for id: String) -> UIColor {
        let hash = abs(id.hashValue)
        return Self.palette[hash % Self.palette.count]
    }

    override init(frame: CGRect) {
        super.init(frame: frame)
        isOpaque = false
        backgroundColor = .clear
        isUserInteractionEnabled = true
    }

    required init?(coder: NSCoder) { fatalError() }

    // MARK: - Coordinate mapping

    /// The rect (in this view's coordinates) that the aspect-fill video occupies.
    private func displayRect() -> CGRect {
        let viewW = bounds.width
        let viewH = bounds.height
        guard videoSize.width > 0, videoSize.height > 0, viewW > 0, viewH > 0 else {
            return bounds
        }
        let scale = max(viewW / videoSize.width, viewH / videoSize.height)
        let dispW = videoSize.width * scale
        let dispH = videoSize.height * scale
        return CGRect(x: (viewW - dispW) / 2, y: (viewH - dispH) / 2, width: dispW, height: dispH)
    }

    private func screenRect(for box: RemotePersonBox, in display: CGRect) -> CGRect {
        CGRect(
            x: display.minX + CGFloat(box.x) * display.width,
            y: display.minY + CGFloat(1 - box.y - box.height) * display.height,
            width: CGFloat(box.width) * display.width,
            height: CGFloat(box.height) * display.height
        )
    }

    // MARK: - Hit testing

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
        for target in hitTargets where target.deleteRect?.contains(point) == true {
            onTapDelete?(target.id); return
        }
        for target in hitTargets where target.labelRect.contains(point) {
            onTapName?(target.id); return
        }
        for target in hitTargets where target.bodyRect.contains(point) {
            onTapBody?(target.id); return
        }
    }

    // MARK: - Drawing

    override func draw(_ rect: CGRect) {
        hitTargets.removeAll(keepingCapacity: true)
        guard let ctx = UIGraphicsGetCurrentContext(), !people.isEmpty else { return }
        let display = displayRect()

        for person in people {
            let box = screenRect(for: person, in: display)
            let baseColor = person.isActive ? UIColor.white : color(for: person.id)

            ctx.setStrokeColor(baseColor.cgColor)
            ctx.setLineWidth(person.isActive ? 3.0 : 2.0)
            ctx.stroke(box)

            let isNamed = (person.name != nil)
            let labelText = (person.name ?? person.label) as NSString
            let font = UIFont.systemFont(ofSize: 12, weight: .semibold)
            let attrs: [NSAttributedString.Key: Any] = [.font: font, .foregroundColor: UIColor.white]
            let textSize = labelText.size(withAttributes: attrs)
            let padding: CGFloat = 4
            let labelBG = CGRect(
                x: box.minX,
                y: box.minY - textSize.height - padding * 2,
                width: textSize.width + padding * 2,
                height: textSize.height + padding * 2
            )
            ctx.setFillColor(baseColor.withAlphaComponent(0.8).cgColor)
            ctx.fill(labelBG)
            labelText.draw(at: CGPoint(x: labelBG.minX + padding, y: labelBG.minY + padding), withAttributes: attrs)

            var deleteRect: CGRect?
            if isNamed {
                let xText = "✕" as NSString
                let xFont = UIFont.systemFont(ofSize: 12, weight: .bold)
                let xAttrs: [NSAttributedString.Key: Any] = [.font: xFont, .foregroundColor: UIColor.white]
                let xSize = xText.size(withAttributes: xAttrs)
                let xBG = CGRect(x: labelBG.maxX + 4, y: labelBG.minY, width: xSize.width + padding * 2, height: labelBG.height)
                ctx.setFillColor(UIColor.systemRed.withAlphaComponent(0.9).cgColor)
                ctx.fill(xBG)
                xText.draw(at: CGPoint(x: xBG.minX + padding, y: xBG.minY + padding), withAttributes: xAttrs)
                deleteRect = xBG.insetBy(dx: -6, dy: -6)
            }

            if person.isActive {
                let badge = "FOLLOWING" as NSString
                let badgeFont = UIFont.systemFont(ofSize: 10, weight: .bold)
                let badgeAttrs: [NSAttributedString.Key: Any] = [.font: badgeFont, .foregroundColor: UIColor.black]
                let badgeSize = badge.size(withAttributes: badgeAttrs)
                let badgeBG = CGRect(x: box.minX, y: box.minY, width: badgeSize.width + padding * 2, height: badgeSize.height + padding)
                ctx.setFillColor(baseColor.cgColor)
                ctx.fill(badgeBG)
                badge.draw(at: CGPoint(x: badgeBG.minX + padding, y: badgeBG.minY + padding / 2), withAttributes: badgeAttrs)
            }

            hitTargets.append(HitTarget(
                id: person.id,
                bodyRect: box,
                labelRect: labelBG.insetBy(dx: -6, dy: -6),
                deleteRect: deleteRect
            ))
        }
    }
}
