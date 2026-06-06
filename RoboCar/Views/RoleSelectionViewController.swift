//
//  RoleSelectionViewController.swift
//  RoboCar
//

import UIKit

final class RoleSelectionViewController: UIViewController {
    var onRoleSelected: ((RoboCarAppRole) -> Void)?

    override func viewDidLoad() {
        super.viewDidLoad()
        view.backgroundColor = .black

        let title = UILabel()
        title.translatesAutoresizingMaskIntoConstraints = false
        title.text = "RoboCar"
        title.textColor = .white
        title.font = .systemFont(ofSize: 34, weight: .bold)
        view.addSubview(title)

        let robotButton = makeButton("Robot Host")
        let controllerButton = makeButton("Remote Controller")
        robotButton.addTarget(self, action: #selector(robotTapped), for: .touchUpInside)
        controllerButton.addTarget(self, action: #selector(controllerTapped), for: .touchUpInside)
        view.addSubview(robotButton)
        view.addSubview(controllerButton)

        NSLayoutConstraint.activate([
            title.centerXAnchor.constraint(equalTo: view.centerXAnchor),
            title.bottomAnchor.constraint(equalTo: robotButton.topAnchor, constant: -36),

            robotButton.centerXAnchor.constraint(equalTo: view.centerXAnchor),
            robotButton.centerYAnchor.constraint(equalTo: view.centerYAnchor, constant: -24),
            robotButton.widthAnchor.constraint(equalToConstant: 260),
            robotButton.heightAnchor.constraint(equalToConstant: 54),

            controllerButton.centerXAnchor.constraint(equalTo: view.centerXAnchor),
            controllerButton.topAnchor.constraint(equalTo: robotButton.bottomAnchor, constant: 16),
            controllerButton.widthAnchor.constraint(equalTo: robotButton.widthAnchor),
            controllerButton.heightAnchor.constraint(equalTo: robotButton.heightAnchor)
        ])
    }

    private func makeButton(_ title: String) -> UIButton {
        let button = UIButton(type: .system)
        button.translatesAutoresizingMaskIntoConstraints = false
        button.setTitle(title, for: .normal)
        button.setTitleColor(.white, for: .normal)
        button.titleLabel?.font = .systemFont(ofSize: 18, weight: .semibold)
        button.backgroundColor = UIColor(white: 0.2, alpha: 1)
        button.layer.cornerRadius = 10
        return button
    }

    @objc private func robotTapped() {
        onRoleSelected?(.robot)
    }

    @objc private func controllerTapped() {
        onRoleSelected?(.controller)
    }
}

enum RoboCarAppRole: String {
    case robot
    case controller

    static let userDefaultsKey = "robocarAppRole"
}

func requestAppRoleSwitch(_ role: RoboCarAppRole) {
    NotificationCenter.default.post(name: .appRoleSwitchRequested, object: nil, userInfo: ["role": role.rawValue])
}

extension Notification.Name {
    static let appRoleSwitchRequested = Notification.Name("appRoleSwitchRequested")
}

