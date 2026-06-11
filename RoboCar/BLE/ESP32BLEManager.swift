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

/// Last-read ST3215 servo telemetry frame.
struct ServoState {
    let id: UInt8
    let error: UInt8
    let position: UInt16
    let load: UInt16
    let voltage: UInt8
    let temperature: UInt8

    var isReadFailure: Bool { error == 0xFF }
}

/// Manages the BLE connection to the ESP32 Motor controller
class ESP32BLEManager: NSObject {
    
    // MARK: - UUIDs
    
    static let serviceUUID    = CBUUID(string: "E3910040-4567-4321-ABCD-ABCDEF012345")
    static let motorsUUID     = CBUUID(string: "E3910003-4567-4321-ABCD-ABCDEF012345")
    static let wifiConfigUUID = CBUUID(string: "E3910004-4567-4321-ABCD-ABCDEF012345")
    static let motorCountUUID = CBUUID(string: "E3910005-4567-4321-ABCD-ABCDEF012345")
    static let batteryUUID    = CBUUID(string: "E3910006-4567-4321-ABCD-ABCDEF012345")
    static let stListUUID     = CBUUID(string: "E3910011-4567-4321-ABCD-ABCDEF012345")
    static let stCmdUUID      = CBUUID(string: "E3910012-4567-4321-ABCD-ABCDEF012345")
    static let stStateUUID    = CBUUID(string: "E3910013-4567-4321-ABCD-ABCDEF012345")
    
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
    
    /// Callback when battery data updates (percentage 0-100, voltage in volts)
    var onBatteryUpdated: ((UInt8, Double) -> Void)?

    /// Callback when ST3215 servo IDs are discovered
    var onServoListUpdated: (([UInt8]) -> Void)?

    /// Callback when ST3215 servo state is refreshed
    var onServoStateUpdated: ((ServoState) -> Void)?
    private var servoListObservers: [UUID: ([UInt8]) -> Void] = [:]
    private var servoStateObservers: [UUID: (ServoState) -> Void] = [:]
    
    /// Latest battery percentage (0–100)
    private(set) var batteryPercentage: UInt8 = 0
    
    /// Latest battery voltage in volts
    private(set) var batteryVoltage: Double = 0.0
    
    /// Discovered peripheral name (for display)
    private(set) var peripheralName: String?
    
    // MARK: - BLE objects
    
    private var centralManager: CBCentralManager!
    private var peripheral: CBPeripheral?
    
    private var motorsChar: CBCharacteristic?
    private var wifiConfigChar: CBCharacteristic?
    private var motorCountChar: CBCharacteristic?
    private var batteryChar: CBCharacteristic?
    private var stListChar: CBCharacteristic?
    private var stCmdChar: CBCharacteristic?
    private var stStateChar: CBCharacteristic?
    private var shouldReadServoListAfterCommand = false
    private var pendingServoStateReadIDs: [UInt8] = []
    private var latestServoIDs: [UInt8] = []
    private var servoMotionTargets: [UInt8: UInt16] = [:]
    private var servoTorqueOffPollIDs = Set<UInt8>()
    private var servoWheelPollIDs = Set<UInt8>()
    private var servoPollTimer: Timer?
    private let servoPollInterval: TimeInterval = 0.35
    private let servoSettledTolerance: UInt16 = 12
    
    /// Number of active motors reported by the ESP32 (2 or 4, default 4)
    private(set) var motorCount: Int = 4
    
    /// Motor write state: latest-value-wins pattern
    private var isMotorWriteInFlight = false
    private var pendingMotorData: Data?
    
    /// Heartbeat timer that re-sends the last motor command to satisfy the ESP32 watchdog
    private var heartbeatTimer: Timer?
    
    /// Last motor data sent (kept for heartbeat re-sends)
    private var lastMotorData: Data?
    
    /// Public read-only access to last motor data (for obstacle detection)
    var lastMotorDataPublic: Data? { lastMotorData }
    
    /// Heartbeat interval — must be well under the ESP32's 500ms watchdog timeout
    private let heartbeatInterval: TimeInterval = 0.2
    
    /// Callback when WiFi credentials have been written
    var onWifiWriteResult: ((Bool) -> Void)?
    
    /// Battery poll timer (fallback if notifications aren't delivered)
    private var batteryPollTimer: Timer?
    
    /// Battery poll interval in seconds
    private let batteryPollInterval: TimeInterval = 5.0
    
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

    @discardableResult
    func addServoListObserver(_ observer: @escaping ([UInt8]) -> Void) -> UUID {
        let id = UUID()
        servoListObservers[id] = observer
        return id
    }

    @discardableResult
    func addServoStateObserver(_ observer: @escaping (ServoState) -> Void) -> UUID {
        let id = UUID()
        servoStateObservers[id] = observer
        return id
    }

    func removeServoObserver(_ id: UUID) {
        servoListObservers.removeValue(forKey: id)
        servoStateObservers.removeValue(forKey: id)
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
    
    /// Auto-connect: start scanning if disconnected (no-op if already connected/scanning)
    func autoConnect() {
        guard connectionState == .disconnected else { return }
        startScanning()
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
    
    /// Set motor powers (sends 2 or 4 bytes depending on motorCount)
    /// Uses latest-value-wins: if a write is in flight, the new value
    /// replaces any pending value and is sent once the current write completes.
    func setAllMotors(a: Int8, b: Int8, c: Int8, d: Int8) {
        // Negate: motor wiring is reversed (positive software = backward physical)
        let a = -a, b = -b, c = -c, d = -d
        guard peripheral != nil, motorsChar != nil else { return }
        
        let clampA = UInt8(bitPattern: max(-100, min(100, a)))
        let clampB = UInt8(bitPattern: max(-100, min(100, b)))
        
        let data: Data
        if motorCount == 2 {
            data = Data([clampA, clampB])
        } else {
            let clampC = UInt8(bitPattern: max(-100, min(100, c)))
            let clampD = UInt8(bitPattern: max(-100, min(100, d)))
            data = Data([clampA, clampB, clampC, clampD])
        }
        
        lastMotorData = data
        
        let allZero = a == 0 && b == 0 && c == 0 && d == 0
        if allZero {
            stopHeartbeat()
        } else {
            startHeartbeat()
        }
        
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
    ///
    /// Uses polar mixing with a power-curved turn component so that
    /// 45° drives forward in an arc (inner wheel stays positive) while
    /// 90° still spins in place.
    @discardableResult
    func drive(x: Float, y: Float) -> (left: Float, right: Float) {
        let fx = x
        let fy = y
        
        let magnitude = min(sqrtf(fx * fx + fy * fy), 1.0)
        guard magnitude > 0.01 else {
            stopAll()
            return (0, 0)
        }
        
        // Angle from forward: 0 = forward, ±π/2 = pure turn, ±π = backward
        let angle = atan2f(fx, fy)
        
        let rawDrive = cosf(angle)  // forward/back component
        let rawTurn  = sinf(angle)  // left/right component
        
        // Power-curve the turn so diagonals keep inner wheel moving.
        // Exponent 2: at 45° turn goes from 0.707 → 0.5, preserving
        // the inner wheel at ~17% instead of 0%. At 90° it stays 1.0.
        let turnExponent: Float = 2.0
        let turn = copysignf(powf(fabsf(rawTurn), turnExponent), rawTurn)
        
        // Mix into left / right
        var left  = rawDrive + turn
        var right = rawDrive - turn
        
        // Normalize so the larger side is ±1 (preserves ratio, avoids clipping)
        let maxAbs = max(fabsf(left), fabsf(right), 1.0)
        left  /= maxAbs
        right /= maxAbs
        
        // Scale by joystick magnitude (how far the stick is pushed)
        left  *= magnitude
        right *= magnitude
        
        let leftPower  = Int8(max(-100, min(100, Int(left * 100))))
        let rightPower = Int8(max(-100, min(100, Int(right * 100))))
        
        // A & C = left side, B & D = right side
        setAllMotors(a: leftPower, b: rightPower, c: leftPower, d: rightPower)
        
        return (left, right)
    }
    
    /// Stop all motors
    func stopAll() {
        setAllMotors(a: 0, b: 0, c: 0, d: 0)
    }

    // MARK: - ST3215 servo control

    func rescanServos(from: UInt8 = 1, to: UInt8 = 20) {
        shouldReadServoListAfterCommand = true
        writeServoCommand([0x06, from, to, 0, 0, 0])
    }

    func moveServo(id: UInt8, position: UInt16, speed: UInt16) {
        let clampedPosition = min(position, 4095)
        let clampedSpeed = min(speed, 4095)
        trackServoMotion(id: id, target: clampedPosition)
        writeServoCommand([
            0x01,
            id,
            UInt8(clampedPosition & 0xFF),
            UInt8(clampedPosition >> 8),
            UInt8(clampedSpeed & 0xFF),
            UInt8(clampedSpeed >> 8),
        ])
    }

    func moveDiscoveredServos(position: UInt16, speed: UInt16, acceleration: UInt8 = 50) {
        let clampedPosition = min(position, 4095)
        let clampedSpeed = min(speed, 4095)
        latestServoIDs.forEach { trackServoMotion(id: $0, target: clampedPosition) }
        writeServoCommand([
            0x07,
            UInt8(clampedPosition & 0xFF),
            UInt8(clampedPosition >> 8),
            UInt8(clampedSpeed & 0xFF),
            UInt8(clampedSpeed >> 8),
            acceleration,
        ])
    }

    func setServoTorque(id: UInt8, enabled: Bool) {
        if enabled {
            servoTorqueOffPollIDs.remove(id)
        } else {
            servoTorqueOffPollIDs.insert(id)
            servoMotionTargets.removeValue(forKey: id)
        }
        updateServoPollTimer()
        writeServoCommand([0x02, id, enabled ? 1 : 0, 0, 0, 0])
    }

    func stopServo(id: UInt8) {
        setServoTorque(id: id, enabled: false)
    }

    func changeServoID(currentID: UInt8, newID: UInt8) {
        writeServoCommand([0x03, currentID, newID, 0, 0, 0])
    }

    func calibrateServoZero(id: UInt8) {
        writeServoCommand([0x08, id, 0, 0, 0, 0])
    }

    func driveServoWheel(id: UInt8, speed: Int16, acceleration: UInt8 = 50) {
        let clampedSpeed = max(Int16(-4095), min(Int16(4095), speed))
        if clampedSpeed == 0 {
            servoWheelPollIDs.remove(id)
        } else {
            servoWheelPollIDs.insert(id)
        }
        updateServoPollTimer()
        let rawSpeed = UInt16(bitPattern: clampedSpeed)
        writeServoCommand([
            0x09,
            id,
            UInt8(rawSpeed & 0xFF),
            UInt8(rawSpeed >> 8),
            acceleration,
            0,
        ])
    }

    func setServoPositionMode(id: UInt8) {
        servoWheelPollIDs.remove(id)
        updateServoPollTimer()
        writeServoCommand([0x0A, id, 0, 0, 0, 0])
    }

    func refreshServoState(id: UInt8) {
        pendingServoStateReadIDs.append(id)
        writeServoCommand([0x05, id, 0, 0, 0, 0])
    }

    func readServoList() {
        guard let p = peripheral, let char = stListChar else { return }
        p.readValue(for: char)
    }

    private func readServoState() {
        guard let p = peripheral, let char = stStateChar else { return }
        p.readValue(for: char)
    }

    private func writeServoCommand(_ bytes: [UInt8]) {
        guard bytes.count == 6, let p = peripheral, let char = stCmdChar else { return }
        p.writeValue(Data(bytes), for: char, type: .withResponse)
    }

    private func trackServoMotion(id: UInt8, target: UInt16) {
        servoMotionTargets[id] = target
        updateServoPollTimer()
    }

    private func updateServoMotion(from state: ServoState) {
        if !state.isReadFailure, let target = servoMotionTargets[state.id] {
            let delta = abs(Int(state.position) - Int(target))
            if delta <= Int(servoSettledTolerance) {
                servoMotionTargets.removeValue(forKey: state.id)
            }
        }
        updateServoPollTimer()
    }

    private func updateServoPollTimer() {
        let ids = servoIDsNeedingPolling()
        if ids.isEmpty {
            stopServoPolling()
        } else {
            startServoPolling()
        }
    }

    private func servoIDsNeedingPolling() -> [UInt8] {
        Array(Set(servoMotionTargets.keys).union(servoTorqueOffPollIDs).union(servoWheelPollIDs)).sorted()
    }

    private func startServoPolling() {
        guard servoPollTimer == nil else { return }
        pollTrackedServoStates()
        servoPollTimer = Timer.scheduledTimer(withTimeInterval: servoPollInterval, repeats: true) { [weak self] _ in
            self?.pollTrackedServoStates()
        }
    }

    private func stopServoPolling() {
        servoPollTimer?.invalidate()
        servoPollTimer = nil
    }

    private func pollTrackedServoStates() {
        servoIDsNeedingPolling().enumerated().forEach { index, id in
            DispatchQueue.main.asyncAfter(deadline: .now() + (Double(index) * 0.08)) { [weak self] in
                self?.refreshServoState(id: id)
            }
        }
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
    
    // MARK: - Heartbeat
    
    /// Start the heartbeat timer (no-op if already running)
    private func startHeartbeat() {
        guard heartbeatTimer == nil else { return }
        heartbeatTimer = Timer.scheduledTimer(withTimeInterval: heartbeatInterval, repeats: true) { [weak self] _ in
            self?.sendHeartbeat()
        }
    }
    
    /// Stop the heartbeat timer
    private func stopHeartbeat() {
        heartbeatTimer?.invalidate()
        heartbeatTimer = nil
    }
    
    /// Re-send the last motor data to keep the ESP32 watchdog happy
    private func sendHeartbeat() {
        guard let data = lastMotorData, peripheral != nil, motorsChar != nil else { return }
        if isMotorWriteInFlight {
            // A write is already queued — just make sure the pending value is current
            pendingMotorData = data
        } else {
            sendMotorData(data)
        }
    }
    
    // MARK: - Battery polling
    
    /// Start polling the battery characteristic periodically
    private func startBatteryPolling() {
        stopBatteryPolling()
        batteryPollTimer = Timer.scheduledTimer(withTimeInterval: batteryPollInterval, repeats: true) { [weak self] _ in
            self?.pollBattery()
        }
    }
    
    /// Stop the battery poll timer
    private func stopBatteryPolling() {
        batteryPollTimer?.invalidate()
        batteryPollTimer = nil
    }
    
    /// Read battery characteristic value
    private func pollBattery() {
        guard let p = peripheral, let char = batteryChar else { return }
        p.readValue(for: char)
    }
    
    private func clearCharacteristics() {
        motorsChar = nil
        wifiConfigChar = nil
        motorCountChar = nil
        batteryChar = nil
        stListChar = nil
        stCmdChar = nil
        stStateChar = nil
        shouldReadServoListAfterCommand = false
        pendingServoStateReadIDs.removeAll()
        latestServoIDs.removeAll()
        servoMotionTargets.removeAll()
        servoTorqueOffPollIDs.removeAll()
        servoWheelPollIDs.removeAll()
        motorCount = 4
        isMotorWriteInFlight = false
        pendingMotorData = nil
        lastMotorData = nil
        stopHeartbeat()
        stopBatteryPolling()
        stopServoPolling()
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
            case Self.motorsUUID:     motorsChar = char
            case Self.wifiConfigUUID: wifiConfigChar = char
            case Self.motorCountUUID:
                motorCountChar = char
                // Read motor count on discovery
                peripheral.readValue(for: char)
            case Self.batteryUUID:
                batteryChar = char
                // Read initial battery value
                peripheral.readValue(for: char)
            case Self.stListUUID:
                stListChar = char
                peripheral.readValue(for: char)
            case Self.stCmdUUID:
                stCmdChar = char
            case Self.stStateUUID:
                stStateChar = char
            default: break
            }
            // Enable notifications
            if char.properties.contains(.notify) {
                peripheral.setNotifyValue(true, for: char)
            }
        }
        print("[BLE] Ready — motors: \(motorsChar != nil), wifi: \(wifiConfigChar != nil), motorCount: \(motorCountChar != nil), battery: \(batteryChar != nil), stList: \(stListChar != nil), stCmd: \(stCmdChar != nil), stState: \(stStateChar != nil)")
        if batteryChar != nil {
            startBatteryPolling()
        }
        connectionState = .connected
    }
    
    func peripheral(_ peripheral: CBPeripheral,
                    didUpdateValueFor characteristic: CBCharacteristic,
                    error: Error?) {
        guard let data = characteristic.value else { return }
        switch characteristic.uuid {
        case Self.motorCountUUID:
            if let byte = data.first {
                let count = Int(byte)
                if count == 2 || count == 4 {
                    motorCount = count
                    print("[BLE] Motor count: \(motorCount)")
                } else {
                    print("[BLE] Unexpected motor count value: \(count), keeping default \(motorCount)")
                }
            }
        case Self.motorsUUID:
            let powers = data.prefix(motorCount).map { Int8(bitPattern: $0) }
            print("[BLE] Motors notification: \(powers)")
        case Self.batteryUUID:
            if data.count >= 3 {
                let pct = data[0]
                let mv = (UInt16(data[1]) << 8) | UInt16(data[2])
                let volts = Double(mv) / 1000.0
                batteryPercentage = pct
                batteryVoltage = volts
                print("[BLE] Battery: \(pct)% \(volts)V")
                onBatteryUpdated?(pct, volts)
            }
        case Self.stListUUID:
            let ids = data.prefix(16).filter { $0 != 0 }
            latestServoIDs = Array(ids)
            print("[BLE] ST3215 IDs: \(Array(ids))")
            onServoListUpdated?(Array(ids))
            servoListObservers.values.forEach { $0(Array(ids)) }
        case Self.stStateUUID:
            guard data.count >= 8 else { return }
            let position = UInt16(data[2]) | (UInt16(data[3]) << 8)
            let load = UInt16(data[4]) | (UInt16(data[5]) << 8)
            let state = ServoState(
                id: data[0],
                error: data[1],
                position: position,
                load: load,
                voltage: data[6],
                temperature: data[7]
            )
            print("[BLE] ST3215 state: id=\(state.id) error=\(state.error) pos=\(state.position) load=\(state.load) voltage=\(state.voltage) temp=\(state.temperature)")
            updateServoMotion(from: state)
            onServoStateUpdated?(state)
            servoStateObservers.values.forEach { $0(state) }
        default:
            break
        }
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
        } else if characteristic.uuid == Self.stCmdUUID {
            if let error = error {
                print("[BLE] ST command write error: \(error.localizedDescription)")
                if !pendingServoStateReadIDs.isEmpty {
                    pendingServoStateReadIDs.removeFirst()
                }
                shouldReadServoListAfterCommand = false
                return
            }
            if shouldReadServoListAfterCommand {
                shouldReadServoListAfterCommand = false
                readServoList()
            }
            if !pendingServoStateReadIDs.isEmpty {
                pendingServoStateReadIDs.removeFirst()
                readServoState()
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
