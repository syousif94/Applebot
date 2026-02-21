//
//  GridMapView.swift
//  RoboCar
//
//  Created by Sammy Yousif on 1/3/26.
//

import UIKit

/// A view that renders the 2D top-down occupancy grid
class GridMapView: UIView {
    
    // MARK: - Configuration
    
    /// How many meters to show around the device
    var viewRadiusMeters: Float = 5.0
    
    /// Colors for different cell states
    let occupiedColor = UIColor.white
    let freeColor = UIColor(red: 0.15, green: 0.05, blue: 0.25, alpha: 1.0)  // Dark purple for floor
    let unknownColor = UIColor(white: 0.1, alpha: 1.0)
    let deviceColor = UIColor.cyan
    let trailColor = UIColor.cyan.withAlphaComponent(0.5)
    let gridLineColor = UIColor(white: 0.3, alpha: 0.5)
    
    /// Cached CGColors for each classification
    private lazy var classificationCGColors: [UInt8: CGColor] = {
        var colors: [UInt8: CGColor] = [:]
        for raw in 0...7 {
            let c = MeshClassification(rawValue: UInt8(raw)) ?? .none
            let rgb = c.color
            colors[UInt8(raw)] = UIColor(red: rgb.r, green: rgb.g, blue: rgb.b, alpha: 1.0).cgColor
        }
        return colors
    }()
    
    /// Reference to the occupancy grid
    private weak var occupancyGrid: OccupancyGrid?
    
    /// Reference to the obstacle detector for highlighting nearby obstacles
    weak var obstacleDetector: ObstacleDetector?
    
    /// Scale: pixels per meter
    private var scale: CGFloat = 50.0
    
    /// Cached free color CGColor
    private lazy var freeCGColor = freeColor.cgColor
    
    /// Position history for trail
    private var positionHistory: [(x: Float, y: Float)] = []
    private let maxHistoryCount = 50
    private var lastRecordedPosition: (x: Float, y: Float)?
    private let minRecordDistance: Float = 0.1  // Record position every 10cm
    
    /// Initial heading to use as reference (so initial forward = up)
    private var initialHeading: Float?
    
    /// Last computed relative heading and scale for tap coordinate conversion
    private var lastRelativeHeading: CGFloat = 0
    private var lastScale: CGFloat = 50.0
    
    /// Planned path in world coordinates (set externally after pathfinding)
    var plannedPath: [(x: Float, y: Float)] = []
    
    /// Target point the user tapped (world coordinates)
    var pathTargetPoint: (x: Float, y: Float)?
    
    /// Callback when the user taps a world position on the grid
    var onTapWorldPosition: ((Float, Float) -> Void)?
    
    // MARK: - Initialization
    
    init(occupancyGrid: OccupancyGrid) {
        self.occupancyGrid = occupancyGrid
        super.init(frame: .zero)
        
        backgroundColor = unknownColor
        contentMode = .redraw
        isOpaque = true  // Optimization hint
        clearsContextBeforeDrawing = false  // We fill the whole rect anyway
        
        // Add pinch gesture for zooming
        let pinch = UIPinchGestureRecognizer(target: self, action: #selector(handlePinch(_:)))
        addGestureRecognizer(pinch)
        
        // Add tap gesture for pathfinding
        let tap = UITapGestureRecognizer(target: self, action: #selector(handleTap(_:)))
        addGestureRecognizer(tap)
    }
    
    required init?(coder: NSCoder) {
        fatalError("init(coder:) has not been implemented")
    }
    
    // MARK: - Gestures
    
    @objc private func handlePinch(_ gesture: UIPinchGestureRecognizer) {
        if gesture.state == .changed {
            viewRadiusMeters = max(1.0, min(20.0, viewRadiusMeters / Float(gesture.scale)))
            gesture.scale = 1.0
            setNeedsDisplay()
        }
    }
    
    @objc private func handleTap(_ gesture: UITapGestureRecognizer) {
        guard gesture.state == .ended else { return }
        guard let grid = occupancyGrid else { return }
        
        let tapPoint = gesture.location(in: self)
        let centerX = bounds.midX
        let centerY = bounds.midY
        
        // Invert the rotation transform applied during drawing
        let heading = lastRelativeHeading
        let cosH = cos(-heading)
        let sinH = sin(-heading)
        
        let dx = tapPoint.x - centerX
        let dy = tapPoint.y - centerY
        let unrotatedX = centerX + dx * cosH - dy * sinH
        let unrotatedY = centerY + dx * sinH + dy * cosH
        
        // Convert from screen to world coordinates
        let devicePos = grid.devicePosition
        let worldX = devicePos.x + Float((centerX - unrotatedX) / lastScale)
        let worldY = devicePos.y + Float((centerY - unrotatedY) / lastScale)
        
        print("[GridMapView] Tap at screen (\(tapPoint.x), \(tapPoint.y)) → world (\(String(format: "%.2f", worldX)), \(String(format: "%.2f", worldY)))")
        
        onTapWorldPosition?(worldX, worldY)
    }
    
    // MARK: - Reset
    
    /// Reset the initial heading and position history
    func resetInitialHeading() {
        initialHeading = nil
        positionHistory.removeAll()
        lastRecordedPosition = nil
        plannedPath.removeAll()
        pathTargetPoint = nil
    }
    
    // MARK: - Position History
    
    private func updatePositionHistory(devicePos: DevicePosition) {
        let currentPos = (x: devicePos.x, y: devicePos.y)
        
        if let lastPos = lastRecordedPosition {
            let dx = currentPos.x - lastPos.x
            let dy = currentPos.y - lastPos.y
            let distance = sqrt(dx * dx + dy * dy)
            
            if distance >= minRecordDistance {
                positionHistory.append(currentPos)
                lastRecordedPosition = currentPos
                
                // Trim history if too long
                if positionHistory.count > maxHistoryCount {
                    positionHistory.removeFirst()
                }
            }
        } else {
            lastRecordedPosition = currentPos
        }
    }
    
    // MARK: - Color Helpers
    
    /// Convert normalized height (0-1) to rainbow color (blue=low, red=high)
    private func rainbowColor(for normalizedValue: CGFloat) -> CGColor {
        // Clamp value to 0-1
        let value = max(0, min(1, normalizedValue))
        
        // Rainbow: blue (low) -> cyan -> green -> yellow -> red (high)
        // Using HSB color space: hue goes from 0.66 (blue) to 0 (red)
        let hue = (1 - value) * 0.66  // 0.66 = blue, 0 = red
        
        return UIColor(hue: hue, saturation: 1.0, brightness: 1.0, alpha: 1.0).cgColor
    }
    
    // MARK: - Drawing
    
    override func draw(_ rect: CGRect) {
        guard let context = UIGraphicsGetCurrentContext(),
              let grid = occupancyGrid else { return }
        
        let devicePos = grid.devicePosition
        let centerX = rect.midX
        let centerY = rect.midY
        
        // Store initial heading on first draw
        if initialHeading == nil {
            initialHeading = devicePos.heading
        }
        
        // Calculate relative heading (difference from initial)
        let relativeHeading = CGFloat(devicePos.heading - (initialHeading ?? 0))
        
        // Store for tap coordinate conversion
        lastRelativeHeading = relativeHeading
        
        // Update position history
        updatePositionHistory(devicePos: devicePos)
        
        // Calculate scale (pixels per meter)
        scale = CGFloat(min(rect.width, rect.height) / CGFloat(viewRadiusMeters * 2))
        lastScale = scale
        
        // Fill background
        context.setFillColor(unknownColor.cgColor)
        context.fill(rect)
        
        // Save state before rotating
        context.saveGState()
        
        // Translate to center, rotate by relative heading (positive = grid rotates opposite to phone)
        context.translateBy(x: centerX, y: centerY)
        context.rotate(by: relativeHeading)
        context.translateBy(x: -centerX, y: -centerY)
        
        // Draw grid lines (1 meter spacing) - rotated
        drawGridLines(context: context, rect: rect, devicePos: devicePos)
        
        // Get region around device
        let region = grid.getRegion(centerX: devicePos.x, centerY: devicePos.y, radiusMeters: viewRadiusMeters * 1.5)  // Slightly larger for rotation
        
        // Calculate height range for rainbow gradient
        let minHeight = region.minHeight
        let maxHeight = region.maxHeight
        let heightRange = maxHeight - minHeight
        
        // Draw cells with classification-based colors
        let cellPixelSize = max(2, scale * CGFloat(grid.cellSize))
        let cellSize = grid.cellSize
        
        // Collect free rects to batch draw
        var freeRects: [CGRect] = []
        freeRects.reserveCapacity(500)
        
        // Collect classification rects for batch drawing by classification type
        var classifiedRects: [UInt8: [CGRect]] = [:]
        
        // Track classified regions for labels: aggregate screen positions per classification
        // Use a grid-based approach: accumulate cell positions per classification
        var classificationAccumulator: [UInt8: (sumX: CGFloat, sumY: CGFloat, count: Int)] = [:]
        
        for (xi, column) in region.cells.enumerated() {
            let worldX = region.originX + Float(xi) * cellSize
            let relX = CGFloat(worldX - devicePos.x) * scale
            // Negate X to fix horizontal flip
            let screenX = centerX - relX - cellPixelSize / 2
            
            for (yi, state) in column.enumerated() {
                guard state != .unknown else { continue }
                
                let worldY = region.originY + Float(yi) * cellSize
                let relY = CGFloat(worldY - devicePos.y) * scale
                // Y increases upward in world, but downward on screen, so negate
                let screenY = centerY - relY - cellPixelSize / 2
                
                let cellRect = CGRect(x: screenX, y: screenY, width: cellPixelSize, height: cellPixelSize)
                
                let classification = region.classifications[xi][yi]
                
                switch state {
                case .occupied:
                    let classRaw = classification.rawValue
                    if classification != .none {
                        // Use classification color
                        classifiedRects[classRaw, default: []].append(cellRect)
                        // Accumulate position for label
                        var acc = classificationAccumulator[classRaw] ?? (0, 0, 0)
                        acc.sumX += screenX + cellPixelSize / 2
                        acc.sumY += screenY + cellPixelSize / 2
                        acc.count += 1
                        classificationAccumulator[classRaw] = acc
                    } else {
                        // Fallback: height-based rainbow color for unclassified
                        let height = region.heights[xi][yi]
                        let normalizedHeight = heightRange > 0 ? (height - minHeight) / heightRange : 0.5
                        let color = rainbowColor(for: CGFloat(normalizedHeight))
                        context.setFillColor(color)
                        context.fill(cellRect)
                    }
                case .free:
                    if classification != .none && classification != .floor {
                        let classRaw = classification.rawValue
                        classifiedRects[classRaw, default: []].append(cellRect)
                        var acc = classificationAccumulator[classRaw] ?? (0, 0, 0)
                        acc.sumX += screenX + cellPixelSize / 2
                        acc.sumY += screenY + cellPixelSize / 2
                        acc.count += 1
                        classificationAccumulator[classRaw] = acc
                    } else {
                        freeRects.append(cellRect)
                    }
                case .unknown:
                    break
                }
            }
        }
        
        // Batch draw all classified cells by type
        for (classRaw, rects) in classifiedRects {
            if let color = classificationCGColors[classRaw] {
                context.setFillColor(color)
                context.fill(rects)
            }
        }
        
        // Batch draw all free cells
        if !freeRects.isEmpty {
            context.setFillColor(freeCGColor)
            context.fill(freeRects)
        }
        
        // Draw classification labels at the centroid of each classified region
        let labelFont = UIFont.boldSystemFont(ofSize: 11)
        let labelShadowAttrs: [NSAttributedString.Key: Any] = [
            .foregroundColor: UIColor.black,
            .font: labelFont
        ]
        for (classRaw, acc) in classificationAccumulator {
            guard acc.count > 10 else { continue }  // Only label regions with enough cells
            guard let classification = MeshClassification(rawValue: classRaw) else { continue }
            let label = classification.label
            guard !label.isEmpty else { continue }
            
            let cx = acc.sumX / CGFloat(acc.count)
            let cy = acc.sumY / CGFloat(acc.count)
            
            let rgb = classification.color
            let labelAttrs: [NSAttributedString.Key: Any] = [
                .foregroundColor: UIColor(red: rgb.r, green: rgb.g, blue: rgb.b, alpha: 1.0),
                .font: labelFont
            ]
            let size = label.size(withAttributes: labelAttrs)
            let drawPoint = CGPoint(x: cx - size.width / 2, y: cy - size.height / 2)
            
            // Draw shadow for readability
            label.draw(at: CGPoint(x: drawPoint.x + 1, y: drawPoint.y + 1), withAttributes: labelShadowAttrs)
            label.draw(at: drawPoint, withAttributes: labelAttrs)
        }
        
        // Draw position trail (in rotated coordinate system)
        drawTrail(context: context, centerX: centerX, centerY: centerY, devicePos: devicePos)
        
        // Restore state (remove rotation)
        context.restoreGState()
        
        // Draw device indicator (always pointing up, not rotated)
        drawDevice(context: context, centerX: centerX, centerY: centerY)
        
        // Draw scale indicator
        drawScaleIndicator(context: context, rect: rect)
        
        // Draw compass (shows which way is north relative to device heading)
        drawCompass(context: context, rect: rect, heading: relativeHeading)
        
        // Draw obstacle highlights LAST so they are always on top
        context.saveGState()
        context.translateBy(x: centerX, y: centerY)
        context.rotate(by: relativeHeading)
        context.translateBy(x: -centerX, y: -centerY)
        drawObstacleHighlights(context: context, centerX: centerX, centerY: centerY, devicePos: devicePos, cellPixelSize: cellPixelSize)
        drawPlannedPath(context: context, centerX: centerX, centerY: centerY, devicePos: devicePos, cellPixelSize: cellPixelSize)
        context.restoreGState()
    }
    
    // MARK: - Planned Path Drawing
    
    private func drawPlannedPath(context: CGContext, centerX: CGFloat, centerY: CGFloat, devicePos: DevicePosition, cellPixelSize: CGFloat) {
        let pathColor = UIColor(red: 0.0, green: 0.9, blue: 0.3, alpha: 0.9).cgColor
        
        if !plannedPath.isEmpty {
            // Draw path as a connected line from device to waypoints
            context.setStrokeColor(pathColor)
            context.setLineWidth(3)
            context.setLineCap(.round)
            context.setLineJoin(.round)
            
            // Start from device position (center of screen)
            context.move(to: CGPoint(x: centerX, y: centerY))
            
            for pt in plannedPath {
                let screenX = centerX - CGFloat(pt.x - devicePos.x) * scale
                let screenY = centerY - CGFloat(pt.y - devicePos.y) * scale
                context.addLine(to: CGPoint(x: screenX, y: screenY))
            }
            context.strokePath()
            
            // Draw waypoint dots
            let dotSize = max(cellPixelSize * 2, 6)
            context.setFillColor(pathColor)
            for pt in plannedPath {
                let screenX = centerX - CGFloat(pt.x - devicePos.x) * scale - dotSize / 2
                let screenY = centerY - CGFloat(pt.y - devicePos.y) * scale - dotSize / 2
                context.fillEllipse(in: CGRect(x: screenX, y: screenY, width: dotSize, height: dotSize))
            }
        } else if let target = pathTargetPoint {
            // No path yet — draw a dashed line from device to target
            context.setStrokeColor(UIColor(red: 0.0, green: 0.9, blue: 0.3, alpha: 0.5).cgColor)
            context.setLineWidth(2)
            context.setLineDash(phase: 0, lengths: [6, 4])
            context.setLineCap(.round)
            
            let targetScreenX = centerX - CGFloat(target.x - devicePos.x) * scale
            let targetScreenY = centerY - CGFloat(target.y - devicePos.y) * scale
            context.move(to: CGPoint(x: centerX, y: centerY))
            context.addLine(to: CGPoint(x: targetScreenX, y: targetScreenY))
            context.strokePath()
            context.setLineDash(phase: 0, lengths: [])  // Reset dash
        }
        
        // Always draw target point when set
        if let target = pathTargetPoint {
            let targetDotSize = max(cellPixelSize * 4, 14)
            let targetScreenX = centerX - CGFloat(target.x - devicePos.x) * scale - targetDotSize / 2
            let targetScreenY = centerY - CGFloat(target.y - devicePos.y) * scale - targetDotSize / 2
            context.setFillColor(UIColor(red: 0.0, green: 1.0, blue: 0.3, alpha: 1.0).cgColor)
            context.fillEllipse(in: CGRect(x: targetScreenX, y: targetScreenY, width: targetDotSize, height: targetDotSize))
            // White border
            context.setStrokeColor(UIColor.white.cgColor)
            context.setLineWidth(2)
            context.strokeEllipse(in: CGRect(x: targetScreenX, y: targetScreenY, width: targetDotSize, height: targetDotSize))
        }
    }
    
    private func drawObstacleHighlights(context: CGContext, centerX: CGFloat, centerY: CGFloat, devicePos: DevicePosition, cellPixelSize: CGFloat) {
        guard let detector = obstacleDetector, detector.obstacleDetected else { return }
        
        let positions = detector.obstacleWorldPositions
        guard !positions.isEmpty else { return }
        
        // Use a dot size at least 6pt so highlights are visible even when zoomed out
        let dotSize = max(cellPixelSize * 3, 6)
        
        // Bright red for obstacle cells
        context.setFillColor(UIColor(red: 1.0, green: 0.0, blue: 0.0, alpha: 0.9).cgColor)
        
        var rects: [CGRect] = []
        rects.reserveCapacity(positions.count)
        
        for pos in positions {
            let relX = CGFloat(pos.x - devicePos.x) * scale
            let relY = CGFloat(pos.y - devicePos.y) * scale
            let screenX = centerX - relX - dotSize / 2
            let screenY = centerY - relY - dotSize / 2
            rects.append(CGRect(x: screenX, y: screenY, width: dotSize, height: dotSize))
        }
        
        context.fill(rects)
        
        // Draw the stop-radius circle as a dashed ring
        let radiusPixels = CGFloat(detector.stopRadius) * scale
        context.setStrokeColor(UIColor(red: 1.0, green: 0.2, blue: 0.1, alpha: 0.8).cgColor)
        context.setLineWidth(2)
        context.setLineDash(phase: 0, lengths: [4, 4])
        context.addEllipse(in: CGRect(
            x: centerX - radiusPixels,
            y: centerY - radiusPixels,
            width: radiusPixels * 2,
            height: radiusPixels * 2
        ))
        context.strokePath()
        context.setLineDash(phase: 0, lengths: [])  // Reset dash
    }
    
    private func drawGridLines(context: CGContext, rect: CGRect, devicePos: DevicePosition) {
        context.setStrokeColor(gridLineColor.cgColor)
        context.setLineWidth(0.5)
        
        let centerX = rect.midX
        let centerY = rect.midY
        
        // Draw vertical lines (every meter)
        let startX = floor(devicePos.x - viewRadiusMeters * 1.5)
        let endX = ceil(devicePos.x + viewRadiusMeters * 1.5)
        
        for x in stride(from: startX, through: endX, by: 1.0) {
            let screenX = centerX - CGFloat(x - devicePos.x) * scale
            context.move(to: CGPoint(x: screenX, y: -rect.height))
            context.addLine(to: CGPoint(x: screenX, y: rect.height * 2))
        }
        
        // Draw horizontal lines (every meter)
        let startY = floor(devicePos.y - viewRadiusMeters * 1.5)
        let endY = ceil(devicePos.y + viewRadiusMeters * 1.5)
        
        for y in stride(from: startY, through: endY, by: 1.0) {
            let screenY = centerY - CGFloat(y - devicePos.y) * scale
            context.move(to: CGPoint(x: -rect.width, y: screenY))
            context.addLine(to: CGPoint(x: rect.width * 2, y: screenY))
        }
        
        context.strokePath()
    }
    
    private func drawTrail(context: CGContext, centerX: CGFloat, centerY: CGFloat, devicePos: DevicePosition) {
        guard positionHistory.count > 1 else { return }
        
        // Draw trail as a gradient line from old (faded) to new (bright)
        let trailCGColor = trailColor.cgColor
        
        for (index, pos) in positionHistory.enumerated() {
            let alpha = CGFloat(index) / CGFloat(positionHistory.count) * 0.7
            let relX = CGFloat(pos.x - devicePos.x) * scale
            let relY = CGFloat(pos.y - devicePos.y) * scale
            let screenX = centerX - relX
            let screenY = centerY - relY
            
            // Draw dot
            let dotSize: CGFloat = 4 + CGFloat(index) / CGFloat(positionHistory.count) * 4
            context.setFillColor(trailColor.withAlphaComponent(alpha).cgColor)
            context.fillEllipse(in: CGRect(
                x: screenX - dotSize / 2,
                y: screenY - dotSize / 2,
                width: dotSize,
                height: dotSize
            ))
        }
        
        // Draw connecting line
        if positionHistory.count > 1 {
            context.setStrokeColor(trailColor.withAlphaComponent(0.3).cgColor)
            context.setLineWidth(2)
            
            let firstPos = positionHistory[0]
            let firstX = centerX - CGFloat(firstPos.x - devicePos.x) * scale
            let firstY = centerY - CGFloat(firstPos.y - devicePos.y) * scale
            context.move(to: CGPoint(x: firstX, y: firstY))
            
            for pos in positionHistory.dropFirst() {
                let screenX = centerX - CGFloat(pos.x - devicePos.x) * scale
                let screenY = centerY - CGFloat(pos.y - devicePos.y) * scale
                context.addLine(to: CGPoint(x: screenX, y: screenY))
            }
            
            // Connect to current position (center)
            context.addLine(to: CGPoint(x: centerX, y: centerY))
            context.strokePath()
        }
    }
    
    private func drawDevice(context: CGContext, centerX: CGFloat, centerY: CGFloat) {
        let size: CGFloat = 20
        
        // Arrow always points UP (forward direction after rotation)
        context.setFillColor(deviceColor.cgColor)
        context.move(to: CGPoint(x: centerX, y: centerY - size))  // Top point
        context.addLine(to: CGPoint(x: centerX - size * 0.6, y: centerY + size * 0.5))  // Bottom left
        context.addLine(to: CGPoint(x: centerX + size * 0.6, y: centerY + size * 0.5))  // Bottom right
        context.closePath()
        context.fillPath()
        
        // Draw outline
        context.setStrokeColor(UIColor.white.cgColor)
        context.setLineWidth(2)
        context.move(to: CGPoint(x: centerX, y: centerY - size))
        context.addLine(to: CGPoint(x: centerX - size * 0.6, y: centerY + size * 0.5))
        context.addLine(to: CGPoint(x: centerX + size * 0.6, y: centerY + size * 0.5))
        context.closePath()
        context.strokePath()
    }
    
    private func drawScaleIndicator(context: CGContext, rect: CGRect) {
        let scaleBarMeters: CGFloat = 1.0
        let scaleBarPixels = scaleBarMeters * scale
        
        let x: CGFloat = 20
        let y = rect.height - 30
        
        // Draw scale bar
        context.setStrokeColor(UIColor.white.cgColor)
        context.setLineWidth(2)
        context.move(to: CGPoint(x: x, y: y))
        context.addLine(to: CGPoint(x: x + scaleBarPixels, y: y))
        context.strokePath()
        
        // End caps
        context.move(to: CGPoint(x: x, y: y - 5))
        context.addLine(to: CGPoint(x: x, y: y + 5))
        context.move(to: CGPoint(x: x + scaleBarPixels, y: y - 5))
        context.addLine(to: CGPoint(x: x + scaleBarPixels, y: y + 5))
        context.strokePath()
        
        // Label
        let label = "1m"
        let attrs: [NSAttributedString.Key: Any] = [
            .foregroundColor: UIColor.white,
            .font: UIFont.monospacedSystemFont(ofSize: 10, weight: .medium)
        ]
        let size = label.size(withAttributes: attrs)
        label.draw(at: CGPoint(x: x + scaleBarPixels / 2 - size.width / 2, y: y - 20), withAttributes: attrs)
    }
    
    private func drawCompass(context: CGContext, rect: CGRect, heading: CGFloat) {
        let radius: CGFloat = 25
        let compassX = rect.width - 40
        let compassY: CGFloat = 40
        
        // Draw circle
        context.setStrokeColor(UIColor.white.withAlphaComponent(0.5).cgColor)
        context.setLineWidth(1)
        context.addEllipse(in: CGRect(x: compassX - radius, y: compassY - radius, width: radius * 2, height: radius * 2))
        context.strokePath()
        
        // Draw north indicator (rotates to show where north is)
        context.saveGState()
        context.translateBy(x: compassX, y: compassY)
        context.rotate(by: -heading)  // Rotate by negative heading to show north direction
        
        // North arrow
        context.setFillColor(UIColor.red.cgColor)
        context.move(to: CGPoint(x: 0, y: -radius + 5))
        context.addLine(to: CGPoint(x: -5, y: -radius + 15))
        context.addLine(to: CGPoint(x: 5, y: -radius + 15))
        context.closePath()
        context.fillPath()
        
        // N label
        let nLabel = "N"
        let attrs: [NSAttributedString.Key: Any] = [
            .foregroundColor: UIColor.red,
            .font: UIFont.boldSystemFont(ofSize: 10)
        ]
        let size = nLabel.size(withAttributes: attrs)
        nLabel.draw(at: CGPoint(x: -size.width / 2, y: -radius - 15), withAttributes: attrs)
        
        context.restoreGState()
    }
}
