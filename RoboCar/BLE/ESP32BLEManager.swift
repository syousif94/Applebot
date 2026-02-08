//
//  ESP32BLEManager.swift
//  RoboCar
//
//  Created by Sammy Yousif on 2/6/26.
//

import Foundation
import CoreBluetooth

/// Connection state for the ESP32 BLE peripheral
enum ESP32ConnectionState {
    case disconnected
    case scanning
    case connecting
    case connected
    
    var displayText: String {
        switch self {
        case .disconnected: return "Disconnected"
        case .scanning:     return "Scanning…"
        case .connecting:   return "Connecting…"
        case .connected:    return "Connected"
        }
    }
    
    var color: (r: CGFloat, g: CGFloat, b: CGFloat) {
        switch self {
        case .disconnected: return (1.0, 0.3, 0.3)
        case .scanning:     return (1.0, 0.8, 0.2)
        case .connecting:   return (1.0, 0.8, 0.2)
        case .connected:    return (0.2, 0.9, 0.4)
        }
    }
}

/// Manages the BLE connection to the ESP32 Motor controller
class ESP32BLEManager: NSObject {
    
    // MARK: - UUIDs
    
    static let serviceUUID    = CBUUID(string: "E3910010-4567-4321-ABCD-ABCDEF012345")
    static let servoAngleUUID = CBUUID(string: "E3910002-4567-4321-ABCD-ABCDEF012345")
    static let motorsUUID     = CBUUID(string: "E3910003-4567-4321-ABCD-ABCDEF012345")
    static let wifiConfigUUID = CBUUID(string: "E3910004-4567-4321-ABCD-ABCDEF012345")
    
    /// Expected device advertisement name
    static let deviceName = "ESP32 Motor"
    
    // MARK: - Singleton
    
    static let shared = ESP32BLEManager()
    
    // MARK: - State
    
    private(set) var connectionState: ESP32ConnectionState = .disconnected {
        didSet { onStateChanged?(connectionState) }
    }
    
    /// Callback when connection state changes
    var onStateChanged: ((ESP32ConnectionState) -> Void)?
    
    /// Discovered peripheral name (for display)
    private(set) var peripheralName: String?
    
    // MARK: - BLE objects
    
    private var centralManager: CBCentralManager!
    private var peripheral: CBPeripheral?
    
    private var servoChar: CBCharacteristic?
    private var motorsChar: CBCharacteristic?
    private var wifiConfigChar: CBCharacteristic?
    
    /// Motor write state: latest-value-wins pattern
    private var isMotorWriteInFlight = false
    private var pendingMotorData: Data?
    
    /// Callback when WiFi credentials have been written
    var onWifiWriteResult: ((Bool) -> Void)?
    
    /// Scan timeout timer
    private var scanTimer: Timer?
    
    /// Whether we should start scanning as soon as BLE is powered on
    private var pendingScan = false
    
    // MARK: - Init
    
    private override init() {
        super.init()
        centralManager = CBCentralManager(delegate: self, queue: nil)
    }
    
    // MARK: - Public API
    
    /// Start scanning for the ESP32
    func startScanning() {
        guard connectionState == .disconnected else { return }
        
        if centralManager.state != .poweredOn {
            // BLE not ready yet — queue the scan for when it becomes ready
            pendingScan = true
            connectionState = .scanning
            print("[BLE] Bluetooth not powered on yet (state: \(centralManager.state.rawValue)), will scan when ready")
            return
        }
        
        connectionState = .scanning
        print("[BLE] Starting scan for service \(Self.serviceUUID)")
        centralManager.scanForPeripherals(withServices: [Self.serviceUUID], options: [
            CBCentralManagerScanOptionAllowDuplicatesKey: false
        ])
        
        // Timeout after 15 seconds
        scanTimer?.invalidate()
        scanTimer = Timer.scheduledTimer(withTimeInterval: 15, repeats: false) { [weak self] _ in
            guard let self, self.connectionState == .scanning else { return }
            self.stopScanning()
        }
    }
    
    /// Stop scanning
    func stopScanning() {
        scanTimer?.invalidate()
        scanTimer = nil
        pendingScan = false
        centralManager.stopScan()
        if connectionState == .scanning {
            print("[BLE] Scan stopped / timed out")
            connectionState = .disconnected
        }
    }
    
    /// Disconnect from the peripheral
    func disconnect() {
        scanTimer?.invalidate()
        scanTimer = nil
        pendingScan = false
        if let p = peripheral {
            centralManager.cancelPeripheralConnection(p)
        }
        clearCharacteristics()
        peripheral = nil
        connectionState = .disconnected
    }
    
    /// Toggle connection: scan if disconnected, disconnect if connected
    func toggleConnection() {
        switch connectionState {
        case .disconnected:
            startScanning()
        case .scanning, .connecting:
            disconnect()
        case .connected:
            disconnect()
        }
    }
    
    // MARK: - Motor control
    
    /// Set servo angle (0–180)
    func setServoAngle(_ angle: UInt8) {
        guard let char = servoChar, let p = peripheral else { return }
        let clamped = min(angle, 180)
        p.writeValue(Data([clamped]), for: char, type: .withResponse)
    }
    
    /// Set all four motor powers at once (single 4-byte write)
    /// Uses latest-value-wins: if a write is in flight, the new value
    /// replaces any pending value and is sent once the current write completes.
    func setAllMotors(a: Int8, b: Int8, c: Int8, d: Int8) {
        guard peripheral != nil, motorsChar != nil else { return }
        let data = Data([
            UInt8(bitPattern: max(-100, min(100, a))),
            UInt8(bitPattern: max(-100, min(100, b))),
            UInt8(bitPattern: max(-100, min(100, c))),
            UInt8(bitPattern: max(-100, min(100, d)))
        ])
        
        if isMotorWriteInFlight {
            // Replace any pending value with the latest
            pendingMotorData = data
        } else {
            sendMotorData(data)
        }
    }
    
    /// Actually write motor data over BLE
    private func sendMotorData(_ data: Data) {
        guard let p = peripheral, let char = motorsChar else { return }
        isMotorWriteInFlight = true
        p.writeValue(data, for: char, type: .withResponse)
    }
    
    /// Called when the motor write completes — sends pending value if any
    func motorWriteDidComplete() {
        isMotorWriteInFlight = false
        if let pending = pendingMotorData {
            pendingMotorData = nil
            sendMotorData(pending)
        }
    }
    
    /// Convenience: differential drive using joystick x,y (-1…1)
    /// x = turn (positive = right), y = throttle (positive = forward)
    func drive(x: Float, y: Float) {
        // Tank-style mixing: left = y + x, right = y - x
        let left  = max(-1, min(1, y + x))
        let right = max(-1, min(1, y - x))
        
        let leftPower  = Int8(left * 100)
        let rightPower = Int8(right * 100)
        
        // A & C = left side, B & D = right side
        setAllMotors(a: leftPower, b: rightPower, c: leftPower, d: rightPower)
    }
    
    /// Stop all motors
    func stopAll() {
        setAllMotors(a: 0, b: 0, c: 0, d: 0)
    }
    
    // MARK: - WiFi config
    
    /// Send WiFi credentials to the ESP32 (stored in flash, used on next boot)
    func setWiFiCredentials(ssid: String, password: String) {
        guard let char = wifiConfigChar, let p = peripheral else {
            print("[BLE] WiFi config char not available (char: \(wifiConfigChar != nil), peripheral: \(peripheral != nil))")
            onWifiWriteResult?(false)
            return
        }
        // Format: "SSID\0PASSWORD"
        var data = Data(ssid.utf8)
        data.append(0) // null separator
        data.append(contentsOf: password.utf8)
        let hex = data.map { String(format: "%02x", $0) }.joined(separator: " ")
        print("[BLE] WiFi write — SSID: \"\(ssid)\", password length: \(password.count), total: \(data.count) bytes")
        print("[BLE] WiFi data hex: \(hex)")
        print("[BLE] WiFi char properties: \(char.properties.rawValue)")
        p.writeValue(data, for: char, type: .withResponse)
        print("[BLE] WiFi write dispatched")
    }
    
    // MARK: - Helpers
    
    private func clearCharacteristics() {
        servoChar = nil
        motorsChar = nil
        wifiConfigChar = nil
        isMotorWriteInFlight = false
        pendingMotorData = nil
    }
}

// MARK: - CBCentralManagerDelegate

extension ESP32BLEManager: CBCentralManagerDelegate {
    
    func centralManagerDidUpdateState(_ central: CBCentralManager) {
        print("[BLE] Central manager state: \(central.state.rawValue) (\(central.state.debugName))")
        switch central.state {
        case .poweredOn:
            if pendingScan {
                pendingScan = false
                connectionState = .disconnected  // reset so startScanning guard passes
                startScanning()
            }
        case .unauthorized:
            print("[BLE] Bluetooth unauthorized — check Info.plist for NSBluetoothAlwaysUsageDescription")
            pendingScan = false
            connectionState = .disconnected
        default:
            disconnect()
        }
    }
    
    func centralManager(_ central: CBCentralManager,
                        didDiscover peripheral: CBPeripheral,
                        advertisementData: [String: Any],
                        rssi RSSI: NSNumber) {
        let name = peripheral.name
            ?? (advertisementData[CBAdvertisementDataLocalNameKey] as? String)
            ?? "ESP32 Motor"
        print("[BLE] ✅ Found \"\(name)\" RSSI: \(RSSI)")
        self.peripheral = peripheral
        self.peripheralName = name
        centralManager.stopScan()
        scanTimer?.invalidate()
        scanTimer = nil
        connectionState = .connecting
        central.connect(peripheral, options: nil)
    }
    
    func centralManager(_ central: CBCentralManager, didConnect peripheral: CBPeripheral) {
        print("[BLE] Connected to \(peripheral.name ?? "unknown")")
        peripheral.delegate = self
        peripheral.discoverServices([Self.serviceUUID])
    }
    
    func centralManager(_ central: CBCentralManager, didFailToConnect peripheral: CBPeripheral, error: Error?) {
        print("[BLE] Failed to connect: \(error?.localizedDescription ?? "unknown error")")
        self.peripheral = nil
        connectionState = .disconnected
    }
    
    func centralManager(_ central: CBCentralManager, didDisconnectPeripheral peripheral: CBPeripheral, error: Error?) {
        print("[BLE] Disconnected: \(error?.localizedDescription ?? "clean")")
        clearCharacteristics()
        self.peripheral = nil
        connectionState = .disconnected
    }
}

// MARK: - CBPeripheralDelegate

extension ESP32BLEManager: CBPeripheralDelegate {
    
    func peripheral(_ peripheral: CBPeripheral, didDiscoverServices error: Error?) {
        if let error = error {
            print("[BLE] Service discovery error: \(error.localizedDescription)")
            disconnect()
            return
        }
        print("[BLE] Discovered services: \(peripheral.services?.map { $0.uuid.uuidString } ?? [])")
        guard let service = peripheral.services?.first(where: { $0.uuid == Self.serviceUUID }) else {
            print("[BLE] Target service not found, disconnecting")
            disconnect()
            return
        }
        peripheral.discoverCharacteristics(nil, for: service)
    }
    
    func peripheral(_ peripheral: CBPeripheral,
                    didDiscoverCharacteristicsFor service: CBService,
                    error: Error?) {
        if let error = error {
            print("[BLE] Characteristic discovery error: \(error.localizedDescription)")
            disconnect()
            return
        }
        print("[BLE] Discovered characteristics: \(service.characteristics?.map { $0.uuid.uuidString } ?? [])")
        for char in service.characteristics ?? [] {
            switch char.uuid {
            case Self.servoAngleUUID:  servoChar = char
            case Self.motorsUUID:     motorsChar = char
            case Self.wifiConfigUUID: wifiConfigChar = char
            default: break
            }
            // Enable notifications
            if char.properties.contains(.notify) {
                peripheral.setNotifyValue(true, for: char)
            }
        }
        print("[BLE] Ready — servo: \(servoChar != nil), motors: \(motorsChar != nil), wifi: \(wifiConfigChar != nil)")
        connectionState = .connected
    }
    
    func peripheral(_ peripheral: CBPeripheral,
                    didUpdateValueFor characteristic: CBCharacteristic,
                    error: Error?) {
        // Handle notifications if needed in the future
    }
    
    func peripheral(_ peripheral: CBPeripheral,
                    didWriteValueFor characteristic: CBCharacteristic,
                    error: Error?) {
        if characteristic.uuid == Self.motorsUUID {
            if let error = error {
                print("[BLE] Motor write error: \(error.localizedDescription)")
            }
            motorWriteDidComplete()
        } else if characteristic.uuid == Self.wifiConfigUUID {
            if let error = error {
                print("[BLE] WiFi write error: \(error.localizedDescription) (code: \((error as NSError).code))")
                onWifiWriteResult?(false)
            } else {
                print("[BLE] WiFi write success — credentials saved to ESP32")
                onWifiWriteResult?(true)
            }
        }
    }
}

// MARK: - CBManagerState debug helper

extension CBManagerState {
    var debugName: String {
        switch self {
        case .unknown:      return "unknown"
        case .resetting:    return "resetting"
        case .unsupported:  return "unsupported"
        case .unauthorized: return "unauthorized"
        case .poweredOff:   return "poweredOff"
        case .poweredOn:    return "poweredOn"
        @unknown default:   return "unknown(\(rawValue))"
        }
    }
}
