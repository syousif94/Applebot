//
//  SpeechSynthesisManager.swift
//  RoboCar
//
//  Created by Sammy Yousif on 2/17/26.
//

import Foundation
import AVFoundation

/// Manages text-to-speech output for the voice assistant
class SpeechSynthesisManager: NSObject {
    
    static let shared = SpeechSynthesisManager()
    
    private let synthesizer = AVSpeechSynthesizer()
    
    /// Called when speech finishes
    var onFinishedSpeaking: (() -> Void)?
    
    /// Whether currently speaking
    var isSpeaking: Bool {
        synthesizer.isSpeaking
    }
    
    private override init() {
        super.init()
        synthesizer.delegate = self
    }
    
    /// Speak the given text aloud
    func speak(_ text: String) {
        // Stop any current speech
        if synthesizer.isSpeaking {
            synthesizer.stopSpeaking(at: .immediate)
        }
        
        let utterance = AVSpeechUtterance(string: text)
        utterance.voice = AVSpeechSynthesisVoice(language: "en-US")
        utterance.rate = AVSpeechUtteranceDefaultSpeechRate * 1.1 // Slightly faster
        utterance.pitchMultiplier = 1.0
        utterance.volume = 1.0
        
        // Ensure audio session is configured for playback
        do {
            let audioSession = AVAudioSession.sharedInstance()
            try audioSession.setCategory(.playAndRecord, mode: .default, options: [.defaultToSpeaker, .allowBluetooth])
            try audioSession.setActive(true)
        } catch {
            print("[TTS] Audio session error: \(error)")
        }
        
        synthesizer.speak(utterance)
    }
    
    /// Stop any current speech
    func stop() {
        if synthesizer.isSpeaking {
            synthesizer.stopSpeaking(at: .immediate)
        }
    }
}

// MARK: - AVSpeechSynthesizerDelegate

extension SpeechSynthesisManager: AVSpeechSynthesizerDelegate {
    func speechSynthesizer(_ synthesizer: AVSpeechSynthesizer, didFinish utterance: AVSpeechUtterance) {
        DispatchQueue.main.async { [weak self] in
            self?.onFinishedSpeaking?()
        }
    }
}
