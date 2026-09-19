//
//  AppDelegate.swift
//  RoboCar
//
//  Created by Sammy Yousif on 1/3/26.
//

import UIKit

@main
class AppDelegate: UIResponder, UIApplicationDelegate {



    func application(_ application: UIApplication, didFinishLaunchingWithOptions launchOptions: [UIApplication.LaunchOptionsKey: Any]?) -> Bool {
        // Override point for customization after application launch.
        return true
    }

    #if targetEnvironment(macCatalyst)
    override func buildMenu(with builder: UIMenuBuilder) {
        super.buildMenu(with: builder)
        guard builder.system == UIMenuSystem.main else { return }

        let modeMenu = UIMenu(title: "Mode", children: [
            UICommand(title: "Remote Controller", action: #selector(switchToController)),
            UICommand(title: "Robot Host", action: #selector(switchToHost))
        ])
        builder.insertSibling(modeMenu, beforeMenu: .window)
    }

    @objc private func switchToController() {
        requestAppRoleSwitch(.controller)
    }

    @objc private func switchToHost() {
        requestAppRoleSwitch(.robot)
    }
    #endif

    // MARK: UISceneSession Lifecycle

    func application(_ application: UIApplication, configurationForConnecting connectingSceneSession: UISceneSession, options: UIScene.ConnectionOptions) -> UISceneConfiguration {
        #if targetEnvironment(macCatalyst)
        if options.userActivities.contains(where: { $0.activityType == PanelWindows.activityType }) || connectingSceneSession.configuration.name == "Utility Panel" {
            let configuration = UISceneConfiguration(name: "Utility Panel", sessionRole: connectingSceneSession.role)
            configuration.delegateClass = PanelSceneDelegate.self
            configuration.storyboard = nil
            return configuration
        }
        if let main = application.connectedScenes.first(where: { $0.delegate is SceneDelegate }) {
            application.requestSceneSessionActivation(main.session, userActivity: nil, options: nil)
            let configuration = UISceneConfiguration(name: "Utility Panel", sessionRole: connectingSceneSession.role)
            configuration.delegateClass = PanelSceneDelegate.self
            configuration.storyboard = nil
            return configuration
        }
        #endif
        // Called when a new scene session is being created.
        // Use this method to select a configuration to create the new scene with.
        return UISceneConfiguration(name: "Default Configuration", sessionRole: connectingSceneSession.role)
    }

    func application(_ application: UIApplication, didDiscardSceneSessions sceneSessions: Set<UISceneSession>) {
        #if targetEnvironment(macCatalyst)
        for session in sceneSessions {
            PanelWindows.shared.closeAll(ownerID: session.persistentIdentifier)
        }
        #endif
        // Called when the user discards a scene session.
        // If any sessions were discarded while the application was not running, this will be called shortly after application:didFinishLaunchingWithOptions.
        // Use this method to release any resources that were specific to the discarded scenes, as they will not return.
    }


}

