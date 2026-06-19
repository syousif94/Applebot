//
//  SceneDelegate.swift
//  RoboCar
//
//  Created by Sammy Yousif on 1/3/26.
//

import UIKit

class SceneDelegate: UIResponder, UIWindowSceneDelegate {

    var window: UIWindow?
    private var currentRole: RoboCarAppRole?

    func scene(_ scene: UIScene, willConnectTo session: UISceneSession, options connectionOptions: UIScene.ConnectionOptions) {
        guard let windowScene = (scene as? UIWindowScene) else { return }
        
        #if targetEnvironment(macCatalyst)
        if let titlebar = windowScene.titlebar {
            titlebar.titleVisibility = .hidden
            titlebar.toolbar = nil
            titlebar.separatorStyle = .none
        }
        #endif
        
        window = UIWindow(windowScene: windowScene)
        NotificationCenter.default.addObserver(self, selector: #selector(handleAppRoleSwitchRequested(_:)), name: .appRoleSwitchRequested, object: nil)
        installInitialRootViewController()
        window?.makeKeyAndVisible()
    }

    private func installInitialRootViewController() {
        if let rawRole = UserDefaults.standard.string(forKey: RoboCarAppRole.userDefaultsKey),
           let role = RoboCarAppRole(rawValue: rawRole) {
            setRoot(for: role)
            return
        }

        let selection = RoleSelectionViewController()
        selection.onRoleSelected = { [weak self] role in
            UserDefaults.standard.set(role.rawValue, forKey: RoboCarAppRole.userDefaultsKey)
            self?.setRoot(for: role)
        }
        window?.rootViewController = selection
    }

    private func setRoot(for role: RoboCarAppRole) {
        if currentRole == .robot || role == .controller {
            RemoteControlHostService.shared.stop()
        }
        TelemetryService.shared.stop(reason: "mode_switch")
        currentRole = role
        switch role {
        case .robot:
            window?.rootViewController = LiDARViewController()
        case .controller:
            window?.rootViewController = RemoteControlViewController()
        }
    }

    @objc private func handleAppRoleSwitchRequested(_ notification: Notification) {
        guard let rawRole = notification.userInfo?["role"] as? String,
              let role = RoboCarAppRole(rawValue: rawRole) else { return }
        UserDefaults.standard.set(role.rawValue, forKey: RoboCarAppRole.userDefaultsKey)
        setRoot(for: role)
    }

    func sceneDidDisconnect(_ scene: UIScene) {
        // Called as the scene is being released by the system.
        // This occurs shortly after the scene enters the background, or when its session is discarded.
        // Release any resources associated with this scene that can be re-created the next time the scene connects.
        // The scene may re-connect later, as its session was not necessarily discarded (see `application:didDiscardSceneSessions` instead).
    }

    func sceneDidBecomeActive(_ scene: UIScene) {
        // Called when the scene has moved from an inactive state to an active state.
        // Use this method to restart any tasks that were paused (or not yet started) when the scene was inactive.
    }

    func sceneWillResignActive(_ scene: UIScene) {
        // Called when the scene will move from an active state to an inactive state.
        // This may occur due to temporary interruptions (ex. an incoming phone call).
    }

    func sceneWillEnterForeground(_ scene: UIScene) {
        guard currentRole == .robot else { return }
        // Auto-connect BLE if not already connected
        ESP32BLEManager.shared.autoConnect()
        // Resume voice listening if permissions were previously granted
        VoiceAssistantManager.shared.startIfPermitted()
        RemoteControlHostService.shared.start()
    }

    func sceneDidEnterBackground(_ scene: UIScene) {
        // Stop telemetry when app backgrounds
        TelemetryService.shared.stop(reason: "background")
        guard currentRole == .robot else { return }
        RemoteControlHostService.shared.stop()
    }


}

