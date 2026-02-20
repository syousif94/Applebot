//
//  VoiceAssistantManager.swift
//  RoboCar
//
//  Created by Sammy Yousif on 2/17/26.
//

import Foundation
import Speech
import AVFoundation

/// Orchestrates the full voice assistant pipeline:
/// Speech Recognition → Foundation Model → Text-to-Speech → Resume Listening
class VoiceAssistantManager {
    
    static let shared = VoiceAssistantManager()
    
    private let speechRecognition = SpeechRecognitionManager()
    private let modelService = FoundationModelService.shared
    private let tts = SpeechSynthesisManager.shared
    
    // MARK: - Callbacks
    
    /// Called when the listening status changes
    var onStatusChanged: ((SpeechRecognitionManager.ListeningStatus) -> Void)?
    
    /// Called with partial transcription text
    var onPartialTranscript: ((String) -> Void)?
    
    /// Called with the model's spoken response text
    var onModelResponse: ((String) -> Void)?
    
    /// Called when an error occurs
    var onError: ((String) -> Void)?
    
    /// Whether the assistant is currently active
    var isActive: Bool { speechRecognition.isActive }
    
    private init() {
        setupCallbacks()
    }
    
    // MARK: - Setup
    
    private func setupCallbacks() {
        speechRecognition.onStatusChanged = { [weak self] status in
            DispatchQueue.main.async {
                self?.onStatusChanged?(status)
            }
        }
        
        speechRecognition.onPartialResult = { [weak self] text in
            DispatchQueue.main.async {
                self?.onPartialTranscript?(text)
            }
        }
        
        speechRecognition.onCommandCaptured = { [weak self] command in
            self?.processCommand(command)
        }
        
        tts.onFinishedSpeaking = { [weak self] in
            // Resume listening after TTS finishes
            self?.speechRecognition.startListening()
        }
    }
    
    // MARK: - Public API
    
    /// Request permissions and start if granted
    func requestPermissionsAndStart(completion: @escaping (Bool) -> Void) {
        speechRecognition.requestPermissions { [weak self] granted in
            if granted {
                self?.speechRecognition.startListening()
            }
            completion(granted)
        }
    }
    
    /// Toggle the assistant on/off
    func toggle() {
        if isActive {
            stop()
        } else {
            start()
        }
    }
    
    /// Start the voice assistant (prompts for permissions if needed)
    func start() {
        speechRecognition.requestPermissions { [weak self] granted in
            if granted {
                self?.speechRecognition.startListening()
            } else {
                self?.onError?("Microphone or speech recognition permission denied.")
            }
        }
    }
    
    /// Start listening only if permissions are already granted (no prompts).
    /// Suitable for automatic resume on foreground.
    func startIfPermitted() {
        let micOK = AVAudioApplication.shared.recordPermission == .granted
        let speechOK = SFSpeechRecognizer.authorizationStatus() == .authorized
        guard micOK && speechOK else { return }
        guard !isActive else { return }
        speechRecognition.startListening()
    }
    
    /// Stop the voice assistant
    func stop() {
        tts.stop()
        speechRecognition.stopListening()
    }
    
    // MARK: - Command Processing
    
    private func processCommand(_ command: String) {
        print("[Assistant] Processing command: \"\(command)\"")
        
        Task {
            do {
                let response = try await modelService.sendCommand(command)
                print("[Assistant] Model response: \"\(response)\"")
                
                await MainActor.run {
                    self.onModelResponse?(response)
                    self.tts.speak(response)
                }
            } catch {
                print("[Assistant] Model error: \(error)")
                await MainActor.run {
                    let errorMsg = "Sorry, I had trouble processing that."
                    self.onError?(errorMsg)
                    self.tts.speak(errorMsg)
                }
            }
        }
    }
}
