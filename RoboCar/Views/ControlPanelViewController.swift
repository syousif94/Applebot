//
//  ControlPanelViewController.swift
//  RoboCar
//
//  Created by Sammy Yousif on 2/6/26.
//

import UIKit

/// Bottom sheet that shows ESP32 BLE connection status and a joystick for manual driving
class ControlPanelViewController: UIViewController {
    
    // MARK: - Views
    
    private let grabber = UIView()
    private let titleLabel = UILabel()
    private let statusDot = UIView()
    private let statusLabel = UILabel()
    private let connectButton = UIButton(type: .system)
    private let joystickView = JoystickView()
    private let joystickLabel = UILabel()
    private let motorLabel = UILabel()
    
    // WiFi config views
    private let wifiLabel = UILabel()
    private let ssidField = UITextField()
    private let passwordField = UITextField()
    private let sendWifiButton = UIButton(type: .custom)
    private let wifiStatusLabel = UILabel()
    
    private let ble = ESP32BLEManager.shared
    
    // MARK: - Lifecycle
    
    override func viewDidLoad() {
        super.viewDidLoad()
        
        view.backgroundColor = UIColor(white: 0.12, alpha: 1)
        
        setupGrabber()
        setupHeader()
        setupConnectionRow()
        setupJoystick()
        setupWiFiConfig()
        
        // Dismiss keyboard on tap outside text fields
        let tap = UITapGestureRecognizer(target: self, action: #selector(dismissKeyboard))
        tap.cancelsTouchesInView = false
        view.addGestureRecognizer(tap)
        
        // Listen for BLE state changes
        ble.onStateChanged = { [weak self] state in
            DispatchQueue.main.async { self?.updateConnectionUI(state) }
        }
        updateConnectionUI(ble.connectionState)
    }
    
    override func viewDidLayoutSubviews() {
        super.viewDidLayoutSubviews()
        statusDot.layer.cornerRadius = statusDot.bounds.width / 2
    }
    
    // MARK: - Setup
    
    private func setupGrabber() {
        grabber.translatesAutoresizingMaskIntoConstraints = false
        grabber.backgroundColor = UIColor.white.withAlphaComponent(0.3)
        grabber.layer.cornerRadius = 2.5
        view.addSubview(grabber)
        
        NSLayoutConstraint.activate([
            grabber.topAnchor.constraint(equalTo: view.topAnchor, constant: 8),
            grabber.centerXAnchor.constraint(equalTo: view.centerXAnchor),
            grabber.widthAnchor.constraint(equalToConstant: 36),
            grabber.heightAnchor.constraint(equalToConstant: 5)
        ])
    }
    
    private func setupHeader() {
        titleLabel.translatesAutoresizingMaskIntoConstraints = false
        titleLabel.text = "ESP32 Controller"
        titleLabel.font = .systemFont(ofSize: 18, weight: .bold)
        titleLabel.textColor = .white
        view.addSubview(titleLabel)
        
        NSLayoutConstraint.activate([
            titleLabel.topAnchor.constraint(equalTo: grabber.bottomAnchor, constant: 16),
            titleLabel.leadingAnchor.constraint(equalTo: view.leadingAnchor, constant: 20)
        ])
    }
    
    private func setupConnectionRow() {
        // Status dot
        statusDot.translatesAutoresizingMaskIntoConstraints = false
        statusDot.backgroundColor = .red
        view.addSubview(statusDot)
        
        // Status label
        statusLabel.translatesAutoresizingMaskIntoConstraints = false
        statusLabel.font = .systemFont(ofSize: 14, weight: .medium)
        statusLabel.textColor = .white
        statusLabel.text = "Disconnected"
        view.addSubview(statusLabel)
        
        // Connect button
        connectButton.translatesAutoresizingMaskIntoConstraints = false
        connectButton.setTitle("Connect", for: .normal)
        connectButton.titleLabel?.font = .systemFont(ofSize: 15, weight: .semibold)
        connectButton.setTitleColor(.white, for: .normal)
        connectButton.backgroundColor = UIColor(red: 0.2, green: 0.5, blue: 1.0, alpha: 1)
        connectButton.layer.cornerRadius = 8
        connectButton.contentEdgeInsets = UIEdgeInsets(top: 8, left: 20, bottom: 8, right: 20)
        connectButton.addTarget(self, action: #selector(connectTapped), for: .touchUpInside)
        view.addSubview(connectButton)
        
        NSLayoutConstraint.activate([
            statusDot.leadingAnchor.constraint(equalTo: view.leadingAnchor, constant: 20),
            statusDot.centerYAnchor.constraint(equalTo: connectButton.centerYAnchor),
            statusDot.widthAnchor.constraint(equalToConstant: 10),
            statusDot.heightAnchor.constraint(equalToConstant: 10),
            
            statusLabel.leadingAnchor.constraint(equalTo: statusDot.trailingAnchor, constant: 8),
            statusLabel.centerYAnchor.constraint(equalTo: connectButton.centerYAnchor),
            
            connectButton.topAnchor.constraint(equalTo: titleLabel.bottomAnchor, constant: 16),
            connectButton.trailingAnchor.constraint(equalTo: view.trailingAnchor, constant: -20)
        ])
    }
    
    private func setupJoystick() {
        // Joystick label
        joystickLabel.translatesAutoresizingMaskIntoConstraints = false
        joystickLabel.text = "Manual Drive"
        joystickLabel.font = .systemFont(ofSize: 14, weight: .semibold)
        joystickLabel.textColor = UIColor.white.withAlphaComponent(0.6)
        view.addSubview(joystickLabel)
        
        // Motor power readout
        motorLabel.translatesAutoresizingMaskIntoConstraints = false
        motorLabel.text = "L: 0%  R: 0%"
        motorLabel.font = .monospacedSystemFont(ofSize: 13, weight: .medium)
        motorLabel.textColor = UIColor.cyan.withAlphaComponent(0.8)
        motorLabel.textAlignment = .right
        view.addSubview(motorLabel)
        
        // Joystick
        joystickView.translatesAutoresizingMaskIntoConstraints = false
        view.addSubview(joystickView)
        
        NSLayoutConstraint.activate([
            joystickLabel.topAnchor.constraint(equalTo: connectButton.bottomAnchor, constant: 24),
            joystickLabel.leadingAnchor.constraint(equalTo: view.leadingAnchor, constant: 20),
            
            motorLabel.centerYAnchor.constraint(equalTo: joystickLabel.centerYAnchor),
            motorLabel.trailingAnchor.constraint(equalTo: view.trailingAnchor, constant: -20),
            
            joystickView.topAnchor.constraint(equalTo: joystickLabel.bottomAnchor, constant: 12),
            joystickView.centerXAnchor.constraint(equalTo: view.centerXAnchor),
            joystickView.widthAnchor.constraint(equalToConstant: 180),
            joystickView.heightAnchor.constraint(equalToConstant: 180),
        ])
        
        // Joystick callbacks
        joystickView.onMove = { [weak self] x, y in
            let powers = self?.ble.drive(x: x, y: y) ?? (left: Float(0), right: Float(0))
            self?.motorLabel.text = String(format: "L: %d%%  R: %d%%",
                                           Int(powers.left * 100), Int(powers.right * 100))
        }
        
        joystickView.onRelease = { [weak self] in
            self?.ble.stopAll()
            self?.motorLabel.text = "L: 0%  R: 0%"
        }
        
        joystickView.onTouchStateChanged = { [weak self] isTouching in
            self?.isModalInPresentation = isTouching
            // Disable the sheet's internal pan-to-dismiss gesture while joystick is active
            self?.setSheetGesturesEnabled(!isTouching)
        }
    }
    
    private func setupWiFiConfig() {
        // Section label
        wifiLabel.translatesAutoresizingMaskIntoConstraints = false
        wifiLabel.text = "WiFi Configuration"
        wifiLabel.font = .systemFont(ofSize: 14, weight: .semibold)
        wifiLabel.textColor = UIColor.white.withAlphaComponent(0.6)
        view.addSubview(wifiLabel)
        
        // SSID field
        ssidField.translatesAutoresizingMaskIntoConstraints = false
        ssidField.placeholder = "Network Name (SSID)"
        ssidField.font = .systemFont(ofSize: 15)
        ssidField.textColor = .white
        ssidField.backgroundColor = UIColor(white: 0.2, alpha: 1)
        ssidField.layer.cornerRadius = 8
        ssidField.autocorrectionType = .no
        ssidField.autocapitalizationType = .none
        ssidField.returnKeyType = .next
        ssidField.leftView = UIView(frame: CGRect(x: 0, y: 0, width: 12, height: 0))
        ssidField.leftViewMode = .always
        ssidField.attributedPlaceholder = NSAttributedString(
            string: "Network Name (SSID)",
            attributes: [.foregroundColor: UIColor(white: 0.5, alpha: 1)]
        )
        ssidField.delegate = self
        view.addSubview(ssidField)
        
        // Password field
        passwordField.translatesAutoresizingMaskIntoConstraints = false
        passwordField.placeholder = "Password"
        passwordField.font = .systemFont(ofSize: 15)
        passwordField.textColor = .white
        passwordField.backgroundColor = UIColor(white: 0.2, alpha: 1)
        passwordField.layer.cornerRadius = 8
        passwordField.isSecureTextEntry = true
        passwordField.autocorrectionType = .no
        passwordField.autocapitalizationType = .none
        passwordField.returnKeyType = .send
        passwordField.leftView = UIView(frame: CGRect(x: 0, y: 0, width: 12, height: 0))
        passwordField.leftViewMode = .always
        passwordField.attributedPlaceholder = NSAttributedString(
            string: "Password",
            attributes: [.foregroundColor: UIColor(white: 0.5, alpha: 1)]
        )
        passwordField.delegate = self
        view.addSubview(passwordField)
        
        // Send button
        sendWifiButton.translatesAutoresizingMaskIntoConstraints = false
        sendWifiButton.setTitle("Save to ESP32", for: .normal)
        sendWifiButton.titleLabel?.font = .systemFont(ofSize: 15, weight: .semibold)
        sendWifiButton.setTitleColor(.white, for: .normal)
        sendWifiButton.backgroundColor = UIColor(red: 0.2, green: 0.6, blue: 0.3, alpha: 1)
        sendWifiButton.layer.cornerRadius = 8
        sendWifiButton.contentEdgeInsets = UIEdgeInsets(top: 10, left: 20, bottom: 10, right: 20)
        sendWifiButton.addTarget(self, action: #selector(sendWifiTapped), for: .touchUpInside)
        view.addSubview(sendWifiButton)
        
        // Status label
        wifiStatusLabel.translatesAutoresizingMaskIntoConstraints = false
        wifiStatusLabel.font = .systemFont(ofSize: 13, weight: .medium)
        wifiStatusLabel.textColor = UIColor.white.withAlphaComponent(0.5)
        wifiStatusLabel.text = ""
        wifiStatusLabel.textAlignment = .center
        view.addSubview(wifiStatusLabel)
        
        NSLayoutConstraint.activate([
            wifiLabel.topAnchor.constraint(equalTo: joystickView.bottomAnchor, constant: 24),
            wifiLabel.leadingAnchor.constraint(equalTo: view.leadingAnchor, constant: 20),
            
            ssidField.topAnchor.constraint(equalTo: wifiLabel.bottomAnchor, constant: 10),
            ssidField.leadingAnchor.constraint(equalTo: view.leadingAnchor, constant: 20),
            ssidField.trailingAnchor.constraint(equalTo: view.trailingAnchor, constant: -20),
            ssidField.heightAnchor.constraint(equalToConstant: 44),
            
            passwordField.topAnchor.constraint(equalTo: ssidField.bottomAnchor, constant: 10),
            passwordField.leadingAnchor.constraint(equalTo: view.leadingAnchor, constant: 20),
            passwordField.trailingAnchor.constraint(equalTo: view.trailingAnchor, constant: -20),
            passwordField.heightAnchor.constraint(equalToConstant: 44),
            
            sendWifiButton.topAnchor.constraint(equalTo: passwordField.bottomAnchor, constant: 14),
            sendWifiButton.centerXAnchor.constraint(equalTo: view.centerXAnchor),
            
            wifiStatusLabel.topAnchor.constraint(equalTo: sendWifiButton.bottomAnchor, constant: 8),
            wifiStatusLabel.centerXAnchor.constraint(equalTo: view.centerXAnchor),
            wifiStatusLabel.bottomAnchor.constraint(lessThanOrEqualTo: view.safeAreaLayoutGuide.bottomAnchor, constant: -16)
        ])
        
        // WiFi write result callback
        ble.onWifiWriteResult = { [weak self] success in
            print("[WiFi] Write result callback — success: \(success)")
            DispatchQueue.main.async {
                if success {
                    self?.wifiStatusLabel.text = "✅ Saved! Will connect on next boot."
                    self?.wifiStatusLabel.textColor = UIColor(red: 0.2, green: 0.9, blue: 0.4, alpha: 1)
                } else {
                    self?.wifiStatusLabel.text = "❌ Failed to save credentials."
                    self?.wifiStatusLabel.textColor = UIColor(red: 1.0, green: 0.3, blue: 0.3, alpha: 1)
                }
                self?.sendWifiButton.isEnabled = true
                self?.sendWifiButton.alpha = 1.0
            }
        }
    }
    
    @objc private func sendWifiTapped() {
        dismissKeyboard()
        
        let ssid = ssidField.text?.trimmingCharacters(in: .whitespacesAndNewlines) ?? ""
        guard !ssid.isEmpty else {
            print("[WiFi] Send tapped but SSID is empty")
            wifiStatusLabel.text = "Enter an SSID"
            wifiStatusLabel.textColor = UIColor(red: 1.0, green: 0.8, blue: 0.2, alpha: 1)
            return
        }
        
        let password = passwordField.text ?? ""
        print("[WiFi] Sending credentials — SSID: \"\(ssid)\", password length: \(password.count)")
        
        sendWifiButton.isEnabled = false
        sendWifiButton.alpha = 0.5
        wifiStatusLabel.text = "Sending…"
        wifiStatusLabel.textColor = UIColor.white.withAlphaComponent(0.5)
        
        ble.setWiFiCredentials(ssid: ssid, password: password)
    }
    
    @objc private func dismissKeyboard() {
        view.endEditing(true)
    }
    
    // MARK: - Sheet gesture control
    
    /// Walk the view hierarchy above this VC's view to find and enable/disable
    /// pan gesture recognizers that drive the sheet dismissal.
    private func setSheetGesturesEnabled(_ enabled: Bool) {
        var current = view.superview
        while let v = current {
            for gr in v.gestureRecognizers ?? [] where gr is UIPanGestureRecognizer {
                gr.isEnabled = enabled
            }
            current = v.superview
        }
    }
    
    // MARK: - Actions
    
    @objc private func connectTapped() {
        ble.toggleConnection()
    }
    
    // MARK: - UI updates
    
    private func updateConnectionUI(_ state: ESP32ConnectionState) {
        let color = state.color
        let uiColor = UIColor(red: color.r, green: color.g, blue: color.b, alpha: 1)
        
        statusDot.backgroundColor = uiColor
        statusLabel.text = state.displayText
        
        if let name = ble.peripheralName, state == .connected {
            statusLabel.text = name
        }
        
        switch state {
        case .disconnected:
            connectButton.setTitle("Connect", for: .normal)
            connectButton.backgroundColor = UIColor(red: 0.2, green: 0.5, blue: 1.0, alpha: 1)
        case .scanning, .connecting:
            connectButton.setTitle("Cancel", for: .normal)
            connectButton.backgroundColor = UIColor(red: 0.5, green: 0.5, blue: 0.5, alpha: 1)
        case .connected:
            connectButton.setTitle("Disconnect", for: .normal)
            connectButton.backgroundColor = UIColor(red: 0.8, green: 0.2, blue: 0.2, alpha: 1)
        }
        
        // Enable/disable joystick visual feedback
        let connected = state == .connected
        joystickView.alpha = connected ? 1.0 : 0.4
        joystickView.isUserInteractionEnabled = connected
        
        // Enable/disable WiFi controls
        ssidField.isEnabled = connected
        passwordField.isEnabled = connected
        sendWifiButton.isEnabled = connected
        ssidField.alpha = connected ? 1.0 : 0.4
        passwordField.alpha = connected ? 1.0 : 0.4
        sendWifiButton.alpha = connected ? 1.0 : 0.4
    }
}

// MARK: - UITextFieldDelegate
extension ControlPanelViewController: UITextFieldDelegate {
    func textFieldShouldReturn(_ textField: UITextField) -> Bool {
        if textField == ssidField {
            passwordField.becomeFirstResponder()
        } else if textField == passwordField {
            sendWifiTapped()
        }
        return true
    }
}
