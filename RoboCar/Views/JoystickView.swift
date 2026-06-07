//
//  JoystickView.swift
//  RoboCar
//
//  Created by Sammy Yousif on 2/6/26.
//

import UIKit

/// A virtual joystick control that reports x,y values in -1…1
class JoystickView: UIView {
    
    // MARK: - Configuration
    
    /// Radius of the outer ring
    private var baseRadius: CGFloat { min(bounds.width, bounds.height) / 2 - 4 }
    
    /// Radius of the thumb knob
    private var knobRadius: CGFloat { baseRadius * 0.38 }
    
    /// Dead zone as fraction of baseRadius (values below this map to 0)
    var deadZone: CGFloat = 0.1
    
    /// Called continuously while the joystick is moved. Values in -1…1
    var onMove: ((_ x: Float, _ y: Float) -> Void)?
    
    /// Called when the joystick is released (snaps to center)
    var onRelease: (() -> Void)?
    
    /// Called when touch begins (true) or ends (false) — use to suppress sheet dismiss
    var onTouchStateChanged: ((_ isTouching: Bool) -> Void)?
    
    // MARK: - State
    
    /// Current knob offset from center (pixels)
    private var knobOffset: CGPoint = .zero
    
    /// Whether the user is currently touching
    private var isTouching = false
    
    /// Feedback generator
    private let impactFeedback = UIImpactFeedbackGenerator(style: .light)
    
    // MARK: - Init
    
    override init(frame: CGRect) {
        super.init(frame: frame)
        setup()
    }
    
    required init?(coder: NSCoder) {
        super.init(coder: coder)
        setup()
    }
    
    private func setup() {
        backgroundColor = .clear
        isMultipleTouchEnabled = false
        impactFeedback.prepare()
    }
    
    // MARK: - Drawing
    
    override func draw(_ rect: CGRect) {
        guard let ctx = UIGraphicsGetCurrentContext() else { return }
        
        let center = CGPoint(x: bounds.midX, y: bounds.midY)
        
        // Outer ring
        ctx.setStrokeColor(UIColor.white.withAlphaComponent(0.3).cgColor)
        ctx.setLineWidth(2)
        ctx.addArc(center: center, radius: baseRadius, startAngle: 0, endAngle: .pi * 2, clockwise: false)
        ctx.strokePath()
        
        // Cross hairs
        ctx.setStrokeColor(UIColor.white.withAlphaComponent(0.15).cgColor)
        ctx.setLineWidth(1)
        ctx.move(to: CGPoint(x: center.x - baseRadius, y: center.y))
        ctx.addLine(to: CGPoint(x: center.x + baseRadius, y: center.y))
        ctx.move(to: CGPoint(x: center.x, y: center.y - baseRadius))
        ctx.addLine(to: CGPoint(x: center.x, y: center.y + baseRadius))
        ctx.strokePath()
        
        // Knob
        let knobCenter = CGPoint(x: center.x + knobOffset.x, y: center.y + knobOffset.y)
        let knobColor = isTouching
            ? UIColor.cyan.withAlphaComponent(0.9)
            : UIColor.white.withAlphaComponent(0.5)
        
        // Knob shadow
        ctx.setShadow(offset: .zero, blur: 8, color: UIColor.cyan.withAlphaComponent(0.4).cgColor)
        ctx.setFillColor(knobColor.cgColor)
        ctx.addArc(center: knobCenter, radius: knobRadius, startAngle: 0, endAngle: .pi * 2, clockwise: false)
        ctx.fillPath()
        
        // Knob border
        ctx.setShadow(offset: .zero, blur: 0, color: nil)
        ctx.setStrokeColor(UIColor.white.withAlphaComponent(0.6).cgColor)
        ctx.setLineWidth(1.5)
        ctx.addArc(center: knobCenter, radius: knobRadius, startAngle: 0, endAngle: .pi * 2, clockwise: false)
        ctx.strokePath()
    }
    
    // MARK: - Touch handling
    
    override func touchesBegan(_ touches: Set<UITouch>, with event: UIEvent?) {
        guard let touch = touches.first else { return }
        isTouching = true
        onTouchStateChanged?(true)
        impactFeedback.impactOccurred()
        updateKnob(for: touch)
    }
    
    override func touchesMoved(_ touches: Set<UITouch>, with event: UIEvent?) {
        guard let touch = touches.first else { return }
        updateKnob(for: touch)
    }
    
    override func touchesEnded(_ touches: Set<UITouch>, with event: UIEvent?) {
        resetKnob()
    }
    
    override func touchesCancelled(_ touches: Set<UITouch>, with event: UIEvent?) {
        resetKnob()
    }
    
    private func updateKnob(for touch: UITouch) {
        let location = touch.location(in: self)
        let center = CGPoint(x: bounds.midX, y: bounds.midY)
        
        var dx = location.x - center.x
        var dy = location.y - center.y
        let distance = sqrt(dx * dx + dy * dy)
        
        // Clamp to base radius
        let maxDist = baseRadius - knobRadius / 2
        if distance > maxDist {
            let scale = maxDist / distance
            dx *= scale
            dy *= scale
        }
        
        knobOffset = CGPoint(x: dx, y: dy)
        setNeedsDisplay()
        
        // Normalize to -1…1
        let normX = Float(-dx / maxDist)  // Negate: left/right is inverted (phone faces backward)
        let normY = Float(-dy / maxDist)  // Negate: screen Y is inverted, up = forward (positive)
        
        // Apply dead zone
        let mag = sqrt(normX * normX + normY * normY)
        if mag < Float(deadZone) {
            onMove?(0, 0)
        } else {
            onMove?(normX, normY)
        }
    }
    
    private func resetKnob() {
        isTouching = false
        onTouchStateChanged?(false)
        knobOffset = .zero
        setNeedsDisplay()
        onMove?(0, 0)
        onRelease?()
    }
}

struct KeyboardDriveVector {
    let x: Float
    let y: Float

    var isActive: Bool { x != 0 || y != 0 }
}

final class KeyboardDriveState {
    private var activeKeys: Set<UIKeyboardHIDUsage> = []

    func pressesBegan(_ presses: Set<UIPress>) -> KeyboardDriveVector? {
        update(with: presses, isPressed: true)
    }

    func pressesEnded(_ presses: Set<UIPress>) -> KeyboardDriveVector? {
        update(with: presses, isPressed: false)
    }

    func reset() -> KeyboardDriveVector? {
        guard !activeKeys.isEmpty else { return nil }
        activeKeys.removeAll()
        return vector
    }

    private func update(with presses: Set<UIPress>, isPressed: Bool) -> KeyboardDriveVector? {
        var handledArrowKey = false
        for press in presses {
            guard let keyCode = press.key?.keyCode, Self.arrowKeyCodes.contains(keyCode) else { continue }
            handledArrowKey = true
            if isPressed {
                activeKeys.insert(keyCode)
            } else {
                activeKeys.remove(keyCode)
            }
        }
        return handledArrowKey ? vector : nil
    }

    private var vector: KeyboardDriveVector {
        let right: Float = activeKeys.contains(.keyboardRightArrow) ? 1 : 0
        let left: Float = activeKeys.contains(.keyboardLeftArrow) ? 1 : 0
        let up: Float = activeKeys.contains(.keyboardUpArrow) ? 1 : 0
        let down: Float = activeKeys.contains(.keyboardDownArrow) ? 1 : 0
        return KeyboardDriveVector(x: left - right, y: up - down)
    }

    private static let arrowKeyCodes: Set<UIKeyboardHIDUsage> = [
        .keyboardUpArrow,
        .keyboardDownArrow,
        .keyboardLeftArrow,
        .keyboardRightArrow
    ]
}

extension UIView {
    var hasFirstResponderTextInput: Bool {
        if isFirstResponder, self is UITextInput {
            return true
        }
        return subviews.contains { $0.hasFirstResponderTextInput }
    }
}
