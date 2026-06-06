//
//  RemoteControlBrowser.swift
//  RoboCar
//

import Foundation
import Network

struct RemoteControlDiscoveredHost: Equatable {
    let name: String
    let endpoint: NWEndpoint
}

final class RemoteControlBrowser {
    var onHostsChanged: (([RemoteControlDiscoveredHost]) -> Void)?
    var onStatusChanged: ((String) -> Void)?

    private let queue = DispatchQueue(label: "com.robocar.remote.browser", qos: .userInitiated)
    private var browser: NWBrowser?
    private var hosts: [RemoteControlDiscoveredHost] = []

    func start() {
        guard browser == nil else { return }
        let parameters = NWParameters.tcp
        parameters.includePeerToPeer = true
        let browser = NWBrowser(for: .bonjour(type: RemoteControlProtocol.serviceType, domain: nil), using: parameters)
        browser.stateUpdateHandler = { [weak self] state in
            switch state {
            case .ready:
                self?.publishStatus("Looking for RoboCar hosts")
            case .failed(let error):
                self?.publishStatus("Bonjour failed: \(error.localizedDescription)")
            default:
                break
            }
        }
        browser.browseResultsChangedHandler = { [weak self] results, _ in
            self?.hosts = results.map { result in
                RemoteControlDiscoveredHost(name: result.endpoint.displayName, endpoint: result.endpoint)
            }.sorted { $0.name < $1.name }
            self?.publishHosts()
        }
        self.browser = browser
        browser.start(queue: queue)
    }

    func stop() {
        browser?.cancel()
        browser = nil
        hosts.removeAll()
        publishHosts()
    }

    private func publishHosts() {
        let hosts = self.hosts
        DispatchQueue.main.async { [weak self] in
            self?.onHostsChanged?(hosts)
        }
    }

    private func publishStatus(_ status: String) {
        DispatchQueue.main.async { [weak self] in
            self?.onStatusChanged?(status)
        }
    }
}

private extension NWEndpoint {
    var displayName: String {
        switch self {
        case .service(let name, _, _, _): return name
        case .hostPort(let host, let port): return "\(host):\(port)"
        default: return "RoboCar"
        }
    }
}
