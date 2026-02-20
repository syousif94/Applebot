//
//  ObstacleBannerContentView.swift
//  RoboCar
//
//  Created by Sammy Yousif on 2/19/26.
//

import UIKit

/// Custom-drawn obstacle banner that shows a rotated chevron per obstacle
/// pointing in its direction, alongside a compact text label.
class ObstacleBannerContentView: UIView {
    
    /// The obstacles to display (max 3 expected)
    var obstacles: [NearbyObstacle] = [] {
        didSet {
            invalidateIntrinsicContentSize()
        }
    }
    
    private let textFont = UIFont.monospacedSystemFont(ofSize: 11, weight: .semibold)
    private let textAttrs: [NSAttributedString.Key: Any]
    
    override init(frame: CGRect) {
        textAttrs = [
            .foregroundColor: UIColor.white,
            .font: UIFont.monospacedSystemFont(ofSize: 11, weight: .semibold)
        ]
        super.init(frame: frame)
        isOpaque = false
        contentMode = .redraw
    }
    
    required init?(coder: NSCoder) {
        fatalError("init(coder:) has not been implemented")
    }
    
    override func draw(_ rect: CGRect) {
        guard let ctx = UIGraphicsGetCurrentContext(), !obstacles.isEmpty else { return }
        
        let h = rect.height
        let chevronSize: CGFloat = 10
        let padding: CGFloat = 10
        let itemSpacing: CGFloat = 6
        let separatorSpacing: CGFloat = 8
        
        var cursorX = padding
        
        for (i, obs) in obstacles.enumerated() {
            if i > 0 {
                // Draw separator pipe
                let sepAttrs: [NSAttributedString.Key: Any] = [
                    .foregroundColor: UIColor(white: 1, alpha: 0.4),
                    .font: textFont
                ]
                let sep = "|" as NSString
                let sepSize = sep.size(withAttributes: sepAttrs)
                sep.draw(at: CGPoint(x: cursorX, y: (h - sepSize.height) / 2), withAttributes: sepAttrs)
                cursorX += sepSize.width + separatorSpacing
            }
            
            // Draw rotated chevron
            // angleDegrees: 0 = ahead (up), positive = right, negative = left
            // In screen space: up = -Y, right = +X
            let angleRad = CGFloat(obs.angleDegrees) * .pi / 180
            let chevronCenterX = cursorX + chevronSize / 2
            let chevronCenterY = h / 2
            
            ctx.saveGState()
            ctx.translateBy(x: chevronCenterX, y: chevronCenterY)
            ctx.rotate(by: angleRad)
            
            // Draw chevron pointing up (^) — two lines meeting at top
            ctx.setStrokeColor(UIColor.white.cgColor)
            ctx.setLineWidth(2.0)
            ctx.setLineCap(.round)
            ctx.setLineJoin(.round)
            
            ctx.move(to: CGPoint(x: -chevronSize * 0.4, y: chevronSize * 0.3))
            ctx.addLine(to: CGPoint(x: 0, y: -chevronSize * 0.3))
            ctx.addLine(to: CGPoint(x: chevronSize * 0.4, y: chevronSize * 0.3))
            ctx.strokePath()
            
            ctx.restoreGState()
            
            cursorX += chevronSize + 4
            
            // Build text: "Wall 12cm +10°"
            let tag: String
            if obs.isDepthBased {
                tag = "Dpth"
            } else {
                tag = obs.classification.label.isEmpty ? "Obj" : obs.classification.label
            }
            let distCM = Int(obs.distance * 100)
            let deg = obs.angleDegrees
            let elev = obs.elevationDegrees
            let degStr = deg >= 0 ? "+\(deg)°" : "\(deg)°"
            let elevStr = elev >= 0 ? "+\(elev)°" : "\(elev)°"
            let text = "\(tag) \(distCM)cm H:\(degStr) V:\(elevStr)" as NSString
            let textSize = text.size(withAttributes: textAttrs)
            
            text.draw(at: CGPoint(x: cursorX, y: (h - textSize.height) / 2), withAttributes: textAttrs)
            cursorX += textSize.width + itemSpacing
        }
    }
    
    override var intrinsicContentSize: CGSize {
        // Compute width needed for all items
        var width: CGFloat = 20  // padding
        let chevronSize: CGFloat = 10
        let itemSpacing: CGFloat = 6
        let separatorSpacing: CGFloat = 8
        
        for (i, obs) in obstacles.enumerated() {
            if i > 0 {
                let sep = "|" as NSString
                let sepAttrs: [NSAttributedString.Key: Any] = [.font: textFont]
                width += sep.size(withAttributes: sepAttrs).width + separatorSpacing
            }
            width += chevronSize + 4
            
            let tag: String
            if obs.isDepthBased {
                tag = "Dpth"
            } else {
                tag = obs.classification.label.isEmpty ? "Obj" : obs.classification.label
            }
            let distCM = Int(obs.distance * 100)
            let deg = obs.angleDegrees
            let elev = obs.elevationDegrees
            let degStr = deg >= 0 ? "+\(deg)°" : "\(deg)°"
            let elevStr = elev >= 0 ? "+\(elev)°" : "\(elev)°"
            let text = "\(tag) \(distCM)cm H:\(degStr) V:\(elevStr)" as NSString
            width += text.size(withAttributes: textAttrs).width + itemSpacing
        }
        width += 10  // trailing padding
        return CGSize(width: width, height: 32)
    }
}
