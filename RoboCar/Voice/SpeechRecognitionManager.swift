//
//  SpeechRecognitionManager.swift
//  RoboCar
//
//  Created by Sammy Yousif on 2/17/26.
//

import Foundation
import Speech
import AVFoundation

/// Manages continuous speech recognition with wake-word detection.
/// Listens for "iphone" and captures subsequent speech, firing a callback
/// after 2 seconds of silence.
class SpeechRecognitionManager: NSObject {
    
    // MARK: - Configuration
    
    /// The wake word that activates command capture
    private let wakeWord = "iphone"
    
    /// Seconds of silence after last recognized speech before sending command
    private let silenceTimeout: TimeInterval = 1.5
    
    // MARK: - Callbacks
    
    /// Called when a command has been captured (text after wake word, after silence timeout)
    var onCommandCaptured: ((String) -> Void)?
    
    /// Called with status updates for the UI
    var onStatusChanged: ((ListeningStatus) -> Void)?
    
    /// Called when partial/live transcription updates (for UI feedback)
    var onPartialResult: ((String) -> Void)?
    
    // MARK: - State
    
    enum ListeningStatus {
        case idle
        case listening          // Waiting for wake word
        case capturing          // Wake word detected, capturing command
        case processing         // Silence detected, sending to model
        
        var displayText: String {
            switch self {
            case .idle:        return "Tap to listen"
            case .listening:   return "Say \"iPhone\" to start…"
            case .capturing:   return "Listening…"
            case .processing:  return "Processing…"
            }
        }
    }
    
    private(set) var status: ListeningStatus = .idle {
        didSet { onStatusChanged?(status) }
    }
    
    private var speechRecognizer: SFSpeechRecognizer?
    private var recognitionRequest: SFSpeechAudioBufferRecognitionRequest?
    private var recognitionTask: SFSpeechRecognitionTask?
    private let audioEngine = AVAudioEngine()
    
    /// Whether we've detected the wake word in the current recognition session
    private var wakeWordDetected = false
    
    /// The accumulated command text after the wake word
    private var capturedCommand = ""
    
    /// Timer that fires after silence timeout
    private var silenceTimer: Timer?
    
    /// The full transcript so far (used to detect wake word position)
    private var lastTranscript = ""
    
    // MARK: - Init
    
    override init() {
        super.init()
        speechRecognizer = SFSpeechRecognizer(locale: Locale(identifier: "en-US"))
        speechRecognizer?.delegate = self
    }
    
    // MARK: - Permissions
    
    /// Request both microphone and speech recognition permissions
    func requestPermissions(completion: @escaping (Bool) -> Void) {
        var micGranted = false
        var speechGranted = false
        let group = DispatchGroup()
        
        group.enter()
        AVAudioApplication.requestRecordPermission { granted in
            micGranted = granted
            group.leave()
        }
        
        group.enter()
        SFSpeechRecognizer.requestAuthorization { authStatus in
            speechGranted = (authStatus == .authorized)
            group.leave()
        }
        
        group.notify(queue: .main) {
            completion(micGranted && speechGranted)
        }
    }
    
    // MARK: - Public API
    
    /// Start continuous listening for the wake word
    func startListening() {
        guard status == .idle || status == .processing else { return }
        
        // Cancel any existing task
        stopRecognition()
        
        wakeWordDetected = false
        capturedCommand = ""
        lastTranscript = ""
        
        do {
            try startRecognitionSession()
            status = .listening
        } catch {
            print("[Speech] Failed to start recognition: \(error)")
            status = .idle
        }
    }
    
    /// Stop all listening
    func stopListening() {
        silenceTimer?.invalidate()
        silenceTimer = nil
        stopRecognition()
        status = .idle
    }
    
    /// Whether we're currently active (listening or capturing)
    var isActive: Bool {
        status != .idle
    }
    
    // MARK: - Recognition Session
    
    private func startRecognitionSession() throws {
        let audioSession = AVAudioSession.sharedInstance()
        try audioSession.setCategory(.playAndRecord, mode: .default, options: [.defaultToSpeaker, .allowBluetooth])
        try audioSession.setActive(true, options: .notifyOthersOnDeactivation)
        
        recognitionRequest = SFSpeechAudioBufferRecognitionRequest()
        guard let recognitionRequest = recognitionRequest else {
            throw NSError(domain: "SpeechRecognition", code: -1, userInfo: [NSLocalizedDescriptionKey: "Could not create recognition request"])
        }
        
        recognitionRequest.shouldReportPartialResults = true
        
        // Use on-device recognition if available for lower latency
        if speechRecognizer?.supportsOnDeviceRecognition == true {
            recognitionRequest.requiresOnDeviceRecognition = true
        }
        
        recognitionTask = speechRecognizer?.recognitionTask(with: recognitionRequest) { [weak self] result, error in
            guard let self else { return }
            
            if let result = result {
                let transcript = result.bestTranscription.formattedString.lowercased()
                self.processTranscript(transcript, isFinal: result.isFinal)
            }
            
            if let error = error {
                print("[Speech] Recognition error: \(error.localizedDescription)")
                // Restart if we were actively listening
                if self.status == .listening || self.status == .capturing {
                    self.restartListening()
                }
            }
        }
        
        let inputNode = audioEngine.inputNode
        let recordingFormat = inputNode.outputFormat(forBus: 0)
        
        inputNode.installTap(onBus: 0, bufferSize: 1024, format: recordingFormat) { [weak self] buffer, _ in
            self?.recognitionRequest?.append(buffer)
        }
        
        audioEngine.prepare()
        try audioEngine.start()
    }
    
    private func processTranscript(_ transcript: String, isFinal: Bool) {
        if !wakeWordDetected {
            // Look for the wake word
            if let range = transcript.range(of: wakeWord) {
                wakeWordDetected = true
                let afterWakeWord = String(transcript[range.upperBound...]).trimmingCharacters(in: .whitespaces)
                capturedCommand = afterWakeWord
                
                DispatchQueue.main.async {
                    self.status = .capturing
                    if !afterWakeWord.isEmpty {
                        self.onPartialResult?(afterWakeWord)
                    }
                }
                
                // Start silence timer
                resetSilenceTimer()
            }
        } else {
            // We're capturing — extract text after wake word
            if let range = transcript.range(of: wakeWord) {
                let afterWakeWord = String(transcript[range.upperBound...]).trimmingCharacters(in: .whitespaces)
                
                if afterWakeWord != capturedCommand {
                    capturedCommand = afterWakeWord
                    DispatchQueue.main.async {
                        self.onPartialResult?(afterWakeWord)
                    }
                    // Reset silence timer since we got new speech
                    resetSilenceTimer()
                }
            }
        }
        
        lastTranscript = transcript
    }
    
    private func resetSilenceTimer() {
        silenceTimer?.invalidate()
        silenceTimer = Timer.scheduledTimer(withTimeInterval: silenceTimeout, repeats: false) { [weak self] _ in
            self?.handleSilenceTimeout()
        }
    }
    
    private func handleSilenceTimeout() {
        guard wakeWordDetected else { return }
        
        let command = capturedCommand.trimmingCharacters(in: .whitespacesAndNewlines)
        
        if command.isEmpty {
            // Wake word detected but no command — go back to listening
            wakeWordDetected = false
            status = .listening
            return
        }
        
        print("[Speech] Command captured: \"\(command)\"")
        status = .processing
        stopRecognition()
        
        onCommandCaptured?(command)
    }
    
    private func restartListening() {
        DispatchQueue.main.asyncAfter(deadline: .now() + 0.3) { [weak self] in
            guard let self, self.status != .idle else { return }
            self.stopRecognition()
            self.wakeWordDetected = false
            self.capturedCommand = ""
            self.lastTranscript = ""
            
            do {
                try self.startRecognitionSession()
                self.status = .listening
            } catch {
                print("[Speech] Failed to restart: \(error)")
                self.status = .idle
            }
        }
    }
    
    private func stopRecognition() {
        audioEngine.stop()
        audioEngine.inputNode.removeTap(onBus: 0)
        recognitionRequest?.endAudio()
        recognitionRequest = nil
        recognitionTask?.cancel()
        recognitionTask = nil
    }
}

// MARK: - SFSpeechRecognizerDelegate

extension SpeechRecognitionManager: SFSpeechRecognizerDelegate {
    func speechRecognizer(_ speechRecognizer: SFSpeechRecognizer, availabilityDidChange available: Bool) {
        print("[Speech] Recognizer availability: \(available)")
        if !available && status != .idle {
            stopListening()
        }
    }
}
