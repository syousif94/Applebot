//
//  GridMapView.swift
//  RoboCar
//
//  Created by Sammy Yousif on 1/3/26.
//

import UIKit
import simd

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

    /// True compass bearing in degrees (0=North, CW positive). Set by LiDARViewController via CLLocationManager.
    /// Negative means not yet received — compass falls back to ARKit-relative orientation.
    var compassBearingDegrees: Double = -1
    
    /// Planned path in world coordinates (set externally after pathfinding)
    var plannedPath: [(x: Float, y: Float)] = []
    
    /// Target point the user tapped (world coordinates) — legacy single target
    var pathTargetPoint: (x: Float, y: Float)?
    
    /// Multi-point route waypoints in world coordinates (ordered)
    var routeWaypoints: [(x: Float, y: Float)] = []
    
    /// Planned paths between consecutive route waypoints (for preview)
    /// Index i holds the path from waypoint[i] to waypoint[i+1] (index 0 = device → WP1)
    var routePreviewPaths: [[(x: Float, y: Float)]] = []
    
    /// Index of the waypoint currently being navigated to (for visual highlighting)
    var activeRouteWaypointIndex: Int = 0

    /// Detected people positions on the grid (id, world position, label)
    var personPositions: [PersonBoxInfo] = []
    
    /// Callback when the user taps a world position on the grid
    var onTapWorldPosition: ((Float, Float) -> Void)?
    
    // MARK: - Pan Offset
    
    /// Pan offset in world-coordinate meters (applied to shift the view center)
    private var panOffsetX: Float = 0
    private var panOffsetY: Float = 0
    
    /// Whether the view is currently panned away from center
    var isPanned: Bool { panOffsetX != 0 || panOffsetY != 0 }
    
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
        
        // Add 2-finger pan gesture for panning around the map
        let pan = UIPanGestureRecognizer(target: self, action: #selector(handlePan(_:)))
        pan.minimumNumberOfTouches = 2
        pan.maximumNumberOfTouches = 2
        addGestureRecognizer(pan)
        
        // Add tap gesture for adding route waypoints
        let tap = UITapGestureRecognizer(target: self, action: #selector(handleTap(_:)))
        addGestureRecognizer(tap)
        
        // Add double-tap gesture to recenter the view
        let doubleTap = UITapGestureRecognizer(target: self, action: #selector(handleDoubleTap(_:)))
        doubleTap.numberOfTapsRequired = 2
        addGestureRecognizer(doubleTap)
        tap.require(toFail: doubleTap)
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
    
    @objc private func handlePan(_ gesture: UIPanGestureRecognizer) {
        guard gesture.state == .changed else { return }
        let translation = gesture.translation(in: self)
        // Convert screen-pixel delta to world-meter delta
        // Invert because dragging right should move view left in world coords
        let heading = lastRelativeHeading
        let cosH = cos(-heading)
        let sinH = sin(-heading)
        // Undo the rotation to get world-aligned deltas
        let worldDx = Float(translation.x * cosH - translation.y * sinH) / Float(lastScale)
        let worldDy = Float(translation.x * sinH + translation.y * cosH) / Float(lastScale)
        panOffsetX += worldDx
        panOffsetY += worldDy
        gesture.setTranslation(.zero, in: self)
        setNeedsDisplay()
    }
    
    @objc private func handleDoubleTap(_ gesture: UITapGestureRecognizer) {
        guard gesture.state == .ended else { return }
        panOffsetX = 0
        panOffsetY = 0
        setNeedsDisplay()
    }
    
    /// Programmatically recenter the view on the device
    func recenterOnDevice() {
        panOffsetX = 0
        panOffsetY = 0
        setNeedsDisplay()
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
        
        // Convert from screen to world coordinates (account for pan offset)
        let devicePos = grid.devicePosition
        let viewCenterX = devicePos.x - panOffsetX
        let viewCenterY = devicePos.y - panOffsetY
        let worldX = viewCenterX - Float((unrotatedX - centerX) / lastScale)
        let worldY = viewCenterY - Float((unrotatedY - centerY) / lastScale)
        
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
        routeWaypoints.removeAll()
        routePreviewPaths.removeAll()
        activeRouteWaypointIndex = 0
        panOffsetX = 0
        panOffsetY = 0
    }
    
    /// Remove the last-added route waypoint
    func removeLastRouteWaypoint() {
        guard !routeWaypoints.isEmpty else { return }
        routeWaypoints.removeLast()
        setNeedsDisplay()
    }
    
    /// Clear all route waypoints
    func clearRouteWaypoints() {
        routeWaypoints.removeAll()
        routePreviewPaths.removeAll()
        activeRouteWaypointIndex = 0
        pathTargetPoint = nil
        plannedPath.removeAll()
        setNeedsDisplay()
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
        
        // View center in world coordinates (device pos shifted by pan offset)
        let viewCenterWorldX = devicePos.x - panOffsetX
        let viewCenterWorldY = devicePos.y - panOffsetY
        
        // Draw grid lines (1 meter spacing) - rotated
        drawGridLines(context: context, rect: rect, viewCenterX: viewCenterWorldX, viewCenterY: viewCenterWorldY)
        
        // Get region around view center (not device)
        let region = grid.getRegion(centerX: viewCenterWorldX, centerY: viewCenterWorldY, radiusMeters: viewRadiusMeters * 1.5)  // Slightly larger for rotation
        
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
            let relX = CGFloat(worldX - viewCenterWorldX) * scale
            let screenX = centerX - relX - cellPixelSize / 2

            for (yi, state) in column.enumerated() {
                guard state != .unknown else { continue }

                let worldY = region.originY + Float(yi) * cellSize
                let relY = CGFloat(worldY - viewCenterWorldY) * scale
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
            
            context.saveGState()
            context.translateBy(x: cx, y: cy)
            context.rotate(by: -relativeHeading)
            let drawPoint = CGPoint(x: -size.width / 2, y: -size.height / 2)
            
            // Draw shadow for readability
            label.draw(at: CGPoint(x: drawPoint.x + 1, y: drawPoint.y + 1), withAttributes: labelShadowAttrs)
            label.draw(at: drawPoint, withAttributes: labelAttrs)
            context.restoreGState()
        }
        
        // Draw position trail (in rotated coordinate system)
        drawTrail(context: context, centerX: centerX, centerY: centerY, viewCenterX: viewCenterWorldX, viewCenterY: viewCenterWorldY)
        
        // Restore state (remove rotation)
        context.restoreGState()
        
        // Draw device indicator (at its correct position relative to view center)
        let deviceScreenX = centerX - CGFloat(devicePos.x - viewCenterWorldX) * scale
        let deviceScreenY = centerY - CGFloat(devicePos.y - viewCenterWorldY) * scale
        drawDevice(context: context, centerX: deviceScreenX, centerY: deviceScreenY)
        
        // Draw scale indicator
        drawScaleIndicator(context: context, rect: rect)
        
        // Draw compass (shows which way is north relative to device heading)
        drawCompass(context: context, rect: rect, heading: relativeHeading)
        
        // Draw obstacle highlights LAST so they are always on top
        context.saveGState()
        context.translateBy(x: centerX, y: centerY)
        context.rotate(by: relativeHeading)
        context.translateBy(x: -centerX, y: -centerY)
        drawObstacleHighlights(context: context, centerX: centerX, centerY: centerY, viewCenterX: viewCenterWorldX, viewCenterY: viewCenterWorldY, cellPixelSize: cellPixelSize)
        drawPlannedPath(context: context, centerX: centerX, centerY: centerY, viewCenterX: viewCenterWorldX, viewCenterY: viewCenterWorldY, cellPixelSize: cellPixelSize)
        drawRouteWaypoints(context: context, centerX: centerX, centerY: centerY, viewCenterX: viewCenterWorldX, viewCenterY: viewCenterWorldY, cellPixelSize: cellPixelSize)
        drawPersonPositions(context: context, centerX: centerX, centerY: centerY, viewCenterX: viewCenterWorldX, viewCenterY: viewCenterWorldY)
        context.restoreGState()
        
        // Draw "panned" indicator if offset from device
        if isPanned {
            let attrs: [NSAttributedString.Key: Any] = [
                .foregroundColor: UIColor.white.withAlphaComponent(0.7),
                .font: UIFont.systemFont(ofSize: 11, weight: .medium)
            ]
            let label = "Double-tap to recenter"
            let size = label.size(withAttributes: attrs)
            label.draw(at: CGPoint(x: rect.midX - size.width / 2, y: 8), withAttributes: attrs)
        }
    }
    
    // MARK: - Planned Path Drawing
    
    private func drawPlannedPath(context: CGContext, centerX: CGFloat, centerY: CGFloat, viewCenterX: Float, viewCenterY: Float, cellPixelSize: CGFloat) {
        let pathColor = UIColor(red: 0.0, green: 0.9, blue: 0.3, alpha: 0.9).cgColor
        
        if !plannedPath.isEmpty {
            // Draw path as a connected line from device to waypoints
            context.setStrokeColor(pathColor)
            context.setLineWidth(3)
            context.setLineCap(.round)
            context.setLineJoin(.round)
            
            // Start from device position
            let deviceScreenX = centerX - CGFloat((occupancyGrid?.devicePosition.x ?? viewCenterX) - viewCenterX) * scale
            let deviceScreenY = centerY - CGFloat((occupancyGrid?.devicePosition.y ?? viewCenterY) - viewCenterY) * scale
            context.move(to: CGPoint(x: deviceScreenX, y: deviceScreenY))
            
            for pt in plannedPath {
                let screenX = centerX - CGFloat(pt.x - viewCenterX) * scale
                let screenY = centerY - CGFloat(pt.y - viewCenterY) * scale
                context.addLine(to: CGPoint(x: screenX, y: screenY))
            }
            context.strokePath()
            
            // Draw waypoint dots
            let dotSize = max(cellPixelSize * 2, 6)
            context.setFillColor(pathColor)
            for pt in plannedPath {
                let screenX = centerX - CGFloat(pt.x - viewCenterX) * scale - dotSize / 2
                let screenY = centerY - CGFloat(pt.y - viewCenterY) * scale - dotSize / 2
                context.fillEllipse(in: CGRect(x: screenX, y: screenY, width: dotSize, height: dotSize))
            }
        } else if let target = pathTargetPoint {
            // No path yet — draw a dashed line from device to target
            context.setStrokeColor(UIColor(red: 0.0, green: 0.9, blue: 0.3, alpha: 0.5).cgColor)
            context.setLineWidth(2)
            context.setLineDash(phase: 0, lengths: [6, 4])
            context.setLineCap(.round)
            
            let deviceScreenX = centerX - CGFloat((occupancyGrid?.devicePosition.x ?? viewCenterX) - viewCenterX) * scale
            let deviceScreenY = centerY - CGFloat((occupancyGrid?.devicePosition.y ?? viewCenterY) - viewCenterY) * scale
            let targetScreenX = centerX - CGFloat(target.x - viewCenterX) * scale
            let targetScreenY = centerY - CGFloat(target.y - viewCenterY) * scale
            context.move(to: CGPoint(x: deviceScreenX, y: deviceScreenY))
            context.addLine(to: CGPoint(x: targetScreenX, y: targetScreenY))
            context.strokePath()
            context.setLineDash(phase: 0, lengths: [])  // Reset dash
        }
        
        // Always draw target point when set
        if let target = pathTargetPoint {
            let targetDotSize = max(cellPixelSize * 4, 14)
            let targetScreenX = centerX - CGFloat(target.x - viewCenterX) * scale - targetDotSize / 2
            let targetScreenY = centerY - CGFloat(target.y - viewCenterY) * scale - targetDotSize / 2
            context.setFillColor(UIColor(red: 0.0, green: 1.0, blue: 0.3, alpha: 1.0).cgColor)
            context.fillEllipse(in: CGRect(x: targetScreenX, y: targetScreenY, width: targetDotSize, height: targetDotSize))
            // White border
            context.setStrokeColor(UIColor.white.cgColor)
            context.setLineWidth(2)
            context.strokeEllipse(in: CGRect(x: targetScreenX, y: targetScreenY, width: targetDotSize, height: targetDotSize))
        }
    }
    
    // MARK: - Route Waypoints Drawing
    
    private func drawRouteWaypoints(context: CGContext, centerX: CGFloat, centerY: CGFloat, viewCenterX: Float, viewCenterY: Float, cellPixelSize: CGFloat) {
        guard !routeWaypoints.isEmpty else { return }
        
        // Draw planned preview paths between waypoints (green lines)
        let previewPathColor = UIColor(red: 0.0, green: 0.9, blue: 0.3, alpha: 0.7).cgColor
        for segmentPath in routePreviewPaths {
            guard segmentPath.count >= 2 else { continue }
            context.setStrokeColor(previewPathColor)
            context.setLineWidth(2.5)
            context.setLineCap(.round)
            context.setLineJoin(.round)
            
            let first = segmentPath[0]
            let firstSX = centerX - CGFloat(first.x - viewCenterX) * scale
            let firstSY = centerY - CGFloat(first.y - viewCenterY) * scale
            context.move(to: CGPoint(x: firstSX, y: firstSY))
            
            for pt in segmentPath.dropFirst() {
                let sx = centerX - CGFloat(pt.x - viewCenterX) * scale
                let sy = centerY - CGFloat(pt.y - viewCenterY) * scale
                context.addLine(to: CGPoint(x: sx, y: sy))
            }
            context.strokePath()
        }
        
        // If no preview paths yet, draw dashed fallback lines
        if routePreviewPaths.isEmpty {
            // Dashed line from device to first waypoint
            if let grid = occupancyGrid {
                let dPos = grid.devicePosition
                let dSX = centerX - CGFloat(dPos.x - viewCenterX) * scale
                let dSY = centerY - CGFloat(dPos.y - viewCenterY) * scale
                let wp0 = routeWaypoints[0]
                let wp0SX = centerX - CGFloat(wp0.x - viewCenterX) * scale
                let wp0SY = centerY - CGFloat(wp0.y - viewCenterY) * scale
                context.setStrokeColor(UIColor(white: 1.0, alpha: 0.4).cgColor)
                context.setLineWidth(1.5)
                context.setLineDash(phase: 0, lengths: [4, 4])
                context.move(to: CGPoint(x: dSX, y: dSY))
                context.addLine(to: CGPoint(x: wp0SX, y: wp0SY))
                context.strokePath()
                context.setLineDash(phase: 0, lengths: [])
            }
            
            for i in 1..<routeWaypoints.count {
                let prev = routeWaypoints[i - 1]
                let cur = routeWaypoints[i]
                let prevSX = centerX - CGFloat(prev.x - viewCenterX) * scale
                let prevSY = centerY - CGFloat(prev.y - viewCenterY) * scale
                let curSX = centerX - CGFloat(cur.x - viewCenterX) * scale
                let curSY = centerY - CGFloat(cur.y - viewCenterY) * scale
                context.setStrokeColor(UIColor(white: 1.0, alpha: 0.4).cgColor)
                context.setLineWidth(1.5)
                context.setLineDash(phase: 0, lengths: [4, 4])
                context.move(to: CGPoint(x: prevSX, y: prevSY))
                context.addLine(to: CGPoint(x: curSX, y: curSY))
                context.strokePath()
                context.setLineDash(phase: 0, lengths: [])
            }
        }
        
        // Draw waypoint circles on top
        let waypointSize = max(cellPixelSize * 5, 18)
        let numberFont = UIFont.boldSystemFont(ofSize: 12)
        
        for (index, wp) in routeWaypoints.enumerated() {
            let screenX = centerX - CGFloat(wp.x - viewCenterX) * scale
            let screenY = centerY - CGFloat(wp.y - viewCenterY) * scale
            
            let isActive = index == activeRouteWaypointIndex
            let isCompleted = index < activeRouteWaypointIndex
            
            // Dot color: blue for pending, orange for active, green for completed
            let dotColor: UIColor
            if isCompleted {
                dotColor = UIColor(red: 0.2, green: 0.8, blue: 0.3, alpha: 0.9)
            } else if isActive {
                dotColor = UIColor(red: 1.0, green: 0.6, blue: 0.1, alpha: 1.0)
            } else {
                dotColor = UIColor(red: 0.3, green: 0.6, blue: 1.0, alpha: 0.9)
            }
            
            // Draw filled circle
            let dotRect = CGRect(x: screenX - waypointSize / 2, y: screenY - waypointSize / 2,
                                 width: waypointSize, height: waypointSize)
            context.setFillColor(dotColor.cgColor)
            context.fillEllipse(in: dotRect)
            
            // White border
            context.setStrokeColor(UIColor.white.cgColor)
            context.setLineWidth(2)
            context.strokeEllipse(in: dotRect)
            
            // Draw number label
            let label = "\(index + 1)"
            let attrs: [NSAttributedString.Key: Any] = [
                .foregroundColor: UIColor.white,
                .font: numberFont
            ]
            let size = label.size(withAttributes: attrs)
            label.draw(at: CGPoint(x: screenX - size.width / 2, y: screenY - size.height / 2), withAttributes: attrs)
        }
    }
    
    private func drawObstacleHighlights(context: CGContext, centerX: CGFloat, centerY: CGFloat, viewCenterX: Float, viewCenterY: Float, cellPixelSize: CGFloat) {
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
            let relX = CGFloat(pos.x - viewCenterX) * scale
            let relY = CGFloat(pos.y - viewCenterY) * scale
            let screenX = centerX - relX - dotSize / 2
            let screenY = centerY - relY - dotSize / 2
            rects.append(CGRect(x: screenX, y: screenY, width: dotSize, height: dotSize))
        }
        
        context.fill(rects)
        
        // Draw the stop-radius circle as a dashed ring (centered on device, not view center)
        let deviceScreenX = centerX - CGFloat((occupancyGrid?.devicePosition.x ?? viewCenterX) - viewCenterX) * scale
        let deviceScreenY = centerY - CGFloat((occupancyGrid?.devicePosition.y ?? viewCenterY) - viewCenterY) * scale
        let radiusPixels = CGFloat(detector.stopRadius) * scale
        context.setStrokeColor(UIColor(red: 1.0, green: 0.2, blue: 0.1, alpha: 0.8).cgColor)
        context.setLineWidth(2)
        context.setLineDash(phase: 0, lengths: [4, 4])
        context.addEllipse(in: CGRect(
            x: deviceScreenX - radiusPixels,
            y: deviceScreenY - radiusPixels,
            width: radiusPixels * 2,
            height: radiusPixels * 2
        ))
        context.strokePath()
        context.setLineDash(phase: 0, lengths: [])  // Reset dash
    }
    
    // MARK: - Person Positions Drawing

    /// Fixed palette matching PersonBoundingBoxOverlay.
    private static let personPalette: [UIColor] = [
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

    private func personColor(for id: UUID) -> UIColor {
        let hash = abs(id.hashValue)
        return Self.personPalette[hash % Self.personPalette.count]
    }

    private func drawPersonPositions(context: CGContext, centerX: CGFloat, centerY: CGFloat, viewCenterX: Float, viewCenterY: Float) {
        guard !personPositions.isEmpty else { return }

        let dotRadius: CGFloat = 5
        let labelFont = UIFont.systemFont(ofSize: 9, weight: .bold)

        for person in personPositions {
            guard let pos = person.worldPosition else { continue }

            let screenX = centerX - CGFloat(pos.x - viewCenterX) * scale
            let screenY = centerY - CGFloat(pos.y - viewCenterY) * scale

            let color = person.isActive ? UIColor.white : personColor(for: person.id)

            // Filled circle
            context.setFillColor(color.cgColor)
            context.fillEllipse(in: CGRect(
                x: screenX - dotRadius,
                y: screenY - dotRadius,
                width: dotRadius * 2,
                height: dotRadius * 2
            ))

            // White border
            context.setStrokeColor(UIColor.white.cgColor)
            context.setLineWidth(1.5)
            context.strokeEllipse(in: CGRect(
                x: screenX - dotRadius,
                y: screenY - dotRadius,
                width: dotRadius * 2,
                height: dotRadius * 2
            ))

            // Active indicator — outer ring
            if person.isActive {
                context.setStrokeColor(color.withAlphaComponent(0.5).cgColor)
                context.setLineWidth(2)
                let outerR = dotRadius + 4
                context.strokeEllipse(in: CGRect(
                    x: screenX - outerR,
                    y: screenY - outerR,
                    width: outerR * 2,
                    height: outerR * 2
                ))
            }

            // Label below
            let label = person.label as NSString
            let labelAttrs: [NSAttributedString.Key: Any] = [
                .font: labelFont,
                .foregroundColor: color,
            ]
            let labelSize = label.size(withAttributes: labelAttrs)
            let shadowAttrs: [NSAttributedString.Key: Any] = [
                .font: labelFont,
                .foregroundColor: UIColor.black,
            ]
            let labelPt = CGPoint(x: screenX - labelSize.width / 2, y: screenY + dotRadius + 2)
            label.draw(at: CGPoint(x: labelPt.x + 1, y: labelPt.y + 1), withAttributes: shadowAttrs)
            label.draw(at: labelPt, withAttributes: labelAttrs)
        }
    }

    private func drawGridLines(context: CGContext, rect: CGRect, viewCenterX: Float, viewCenterY: Float) {
        context.setStrokeColor(gridLineColor.cgColor)
        context.setLineWidth(0.5)
        
        let centerX = rect.midX
        let centerY = rect.midY
        
        // Draw vertical lines (every meter)
        let startX = floor(viewCenterX - viewRadiusMeters * 1.5)
        let endX = ceil(viewCenterX + viewRadiusMeters * 1.5)
        
        for x in stride(from: startX, through: endX, by: 1.0) {
            let screenX = centerX - CGFloat(x - viewCenterX) * scale
            context.move(to: CGPoint(x: screenX, y: -rect.height))
            context.addLine(to: CGPoint(x: screenX, y: rect.height * 2))
        }

        // Draw horizontal lines (every meter)
        let startY = floor(viewCenterY - viewRadiusMeters * 1.5)
        let endY = ceil(viewCenterY + viewRadiusMeters * 1.5)

        for y in stride(from: startY, through: endY, by: 1.0) {
            let screenY = centerY - CGFloat(y - viewCenterY) * scale
            context.move(to: CGPoint(x: -rect.width, y: screenY))
            context.addLine(to: CGPoint(x: rect.width * 2, y: screenY))
        }
        
        context.strokePath()
    }
    
    private func drawTrail(context: CGContext, centerX: CGFloat, centerY: CGFloat, viewCenterX: Float, viewCenterY: Float) {
        guard positionHistory.count > 1 else { return }
        
        // Draw trail as a gradient line from old (faded) to new (bright)
        let trailCGColor = trailColor.cgColor
        
        for (index, pos) in positionHistory.enumerated() {
            let alpha = CGFloat(index) / CGFloat(positionHistory.count) * 0.7
            let relX = CGFloat(pos.x - viewCenterX) * scale
            let relY = CGFloat(pos.y - viewCenterY) * scale
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
            let firstX = centerX - CGFloat(firstPos.x - viewCenterX) * scale
            let firstY = centerY - CGFloat(firstPos.y - viewCenterY) * scale
            context.move(to: CGPoint(x: firstX, y: firstY))
            
            for pos in positionHistory.dropFirst() {
                let screenX = centerX - CGFloat(pos.x - viewCenterX) * scale
                let screenY = centerY - CGFloat(pos.y - viewCenterY) * scale
                context.addLine(to: CGPoint(x: screenX, y: screenY))
            }
            
            // Connect to current device position
            let deviceScreenX = centerX - CGFloat((occupancyGrid?.devicePosition.x ?? viewCenterX) - viewCenterX) * scale
            let deviceScreenY = centerY - CGFloat((occupancyGrid?.devicePosition.y ?? viewCenterY) - viewCenterY) * scale
            context.addLine(to: CGPoint(x: deviceScreenX, y: deviceScreenY))
            context.strokePath()
        }
    }
    
    private func drawDevice(context: CGContext, centerX: CGFloat, centerY: CGFloat) {
        let size: CGFloat = 20
        
        // Pointed end marks the rear of the device; the broad edge faces forward.
        context.setFillColor(deviceColor.cgColor)
        context.move(to: CGPoint(x: centerX, y: centerY + size))
        context.addLine(to: CGPoint(x: centerX - size * 0.6, y: centerY - size * 0.5))
        context.addLine(to: CGPoint(x: centerX + size * 0.6, y: centerY - size * 0.5))
        context.closePath()
        context.fillPath()
        
        // Draw outline
        context.setStrokeColor(UIColor.white.cgColor)
        context.setLineWidth(2)
        context.move(to: CGPoint(x: centerX, y: centerY + size))
        context.addLine(to: CGPoint(x: centerX - size * 0.6, y: centerY - size * 0.5))
        context.addLine(to: CGPoint(x: centerX + size * 0.6, y: centerY - size * 0.5))
        context.closePath()
        context.strokePath()
    }
    
    private func drawScaleIndicator(context: CGContext, rect: CGRect) {
        let scaleBarMeters: CGFloat = 1.0
        let scaleBarPixels = scaleBarMeters * scale
        
        // Top-left, across from the compass (which sits at the top-right)
        let x: CGFloat = 20
        let y: CGFloat = 45
        
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

        // Use true compass bearing when available (0°=N, 90°=E, CW positive).
        // Screen "up" = current device facing direction, so rotating by -bearing places N correctly.
        // Fall back to ARKit-relative heading (radians) if no compass data yet.
        let rotation: CGFloat = compassBearingDegrees >= 0
            ? -CGFloat(compassBearingDegrees) * .pi / 180
            : -heading
        context.rotate(by: rotation)

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
