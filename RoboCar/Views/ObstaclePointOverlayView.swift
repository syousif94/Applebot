//
//  ObstaclePointOverlayView.swift
//  RoboCar
//
//  Created by Sammy Yousif on 2/20/26.
//

import UIKit

/// A screen-space point with its distance from the camera for perspective sizing.
struct ProjectedObstaclePoint {
    let position: CGPoint
    let distance: Float  // meters from camera
}

/// Draws round white dots at projected screen positions of 3D obstacle points
/// within the obstacle detector radius, overlaid on the AR camera view.
/// Dot size scales inversely with distance so closer surfaces look denser.
/// Also draws green dots for planned path waypoints.
class ObstaclePointOverlayView: UIView {
    
    /// Screen-space positions + distance to draw dots at
    var projectedPoints: [ProjectedObstaclePoint] = []
    
    /// Base dot diameter at reference distance (points)
    var baseDotSize: CGFloat = 3
    
    /// Reference distance for base size (meters)
    var referenceDistance: CGFloat = 0.15
    
    override init(frame: CGRect) {
        super.init(frame: frame)
        isOpaque = false
        isUserInteractionEnabled = false
        backgroundColor = .clear
    }
    
    required init?(coder: NSCoder) { fatalError() }
    
    override func draw(_ rect: CGRect) {
        guard let ctx = UIGraphicsGetCurrentContext() else { return }
        guard !projectedPoints.isEmpty else { return }
        
        let maxDot: CGFloat = 8
        let minDot: CGFloat = 1.5
        let expandedRect = rect.insetBy(dx: -maxDot, dy: -maxDot)
        
        // Draw obstacle points (white)
        if !projectedPoints.isEmpty {
            ctx.setFillColor(UIColor.white.cgColor)
            
            for pp in projectedPoints {
                guard expandedRect.contains(pp.position) else { continue }
                
                let d = max(CGFloat(pp.distance), 0.03)
                let size = min(maxDot, max(minDot, baseDotSize * referenceDistance / d))
                let half = size / 2
                
                let ellipseRect = CGRect(x: pp.position.x - half, y: pp.position.y - half, width: size, height: size)
                ctx.fillEllipse(in: ellipseRect)
            }
        }
    }
}
