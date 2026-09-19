import UIKit
import AVFoundation
import Vision
import UniformTypeIdentifiers

final class RemoteConnectionStatusView: UIStackView {
    private let label = UILabel()
    private let disconnectButton = UIButton(type: .system)
    private let open: (UIViewController) -> Void

    init(open: @escaping (UIViewController) -> Void) {
        self.open = open
        super.init(frame: .zero)
        axis = .horizontal
        alignment = .center
        spacing = 8
        isLayoutMarginsRelativeArrangement = true
        directionalLayoutMargins = NSDirectionalEdgeInsets(top: 0, leading: 12, bottom: 0, trailing: 12)
        backgroundColor = UIColor(white: 0.12, alpha: 0.95)
        label.font = .systemFont(ofSize: 13, weight: .semibold)
        label.textColor = .white
        label.numberOfLines = 2
        label.lineBreakMode = .byTruncatingTail
        addArrangedSubview(label)
        let devices = UIButton(type: .system)
        devices.setImage(UIImage(systemName: "qrcode.viewfinder"), for: .normal)
        devices.accessibilityLabel = "Paired devices"
        devices.addTarget(self, action: #selector(showDevices), for: .touchUpInside)
        disconnectButton.setImage(UIImage(systemName: "xmark.circle.fill"), for: .normal)
        disconnectButton.accessibilityLabel = "Disconnect controller"
        disconnectButton.addTarget(self, action: #selector(disconnect), for: .touchUpInside)
        for button in [devices, disconnectButton] {
            button.tintColor = .white
            button.widthAnchor.constraint(equalToConstant: 44).isActive = true
            button.heightAnchor.constraint(equalToConstant: 44).isActive = true
            addArrangedSubview(button)
        }
        NotificationCenter.default.addObserver(self, selector: #selector(refresh), name: .remotePeersChanged, object: nil)
        refresh()
    }

    required init(coder: NSCoder) { fatalError("init(coder:) has not been implemented") }

    @objc private func refresh() {
        let transport = RemoteControlIrohSession.shared
        label.text = transport.connectedPeer.map { "Controller connected: \($0.name)" } ?? "No controller connected"
        label.textColor = transport.isConnected ? .systemGreen : .white
        disconnectButton.isEnabled = transport.isConnected
        disconnectButton.alpha = transport.isConnected ? 1 : 0.35
    }

    @objc private func disconnect() { RemoteControlIrohSession.shared.disconnect() }

    @objc private func showDevices() {
        var responder: UIResponder? = self
        while let current = responder {
            if let controller = current as? UIViewController { open(controller); return }
            responder = current.next
        }
    }
}

final class RemotePairingViewController: UITableViewController, AVCaptureMetadataOutputObjectsDelegate, UIDocumentPickerDelegate {
    private let transport = RemoteControlIrohSession.shared
    private let frameProvider: (() -> CVPixelBuffer?)?
    private let imageView = UIImageView()
    private let statusLabel = UILabel()
    private let capture = AVCaptureSession()
    private let captureQueue = DispatchQueue(label: "com.robocar.pairing.camera")
    private var preview: AVCaptureVideoPreviewLayer?
    private var scanning = false
    private var processing = false
    private var hasShownInitialQR = false
    private var scanAttempt = UUID()
    private let previewContext = CIContext()
    private var timer: Timer?
    private var scanTimer: Timer?
    private var operation: Task<Void, Never>?
    private var peers: [RemotePeer] { transport.store?.peers ?? [] }

    init(frameProvider: (() -> CVPixelBuffer?)? = nil) {
        self.frameProvider = frameProvider
        super.init(style: .insetGrouped)
    }

    required init?(coder: NSCoder) { fatalError("init(coder:) has not been implemented") }

    static func show(from owner: UIViewController, frameProvider: (() -> CVPixelBuffer?)? = nil) {
        let sheet = RemotePairingViewController(frameProvider: frameProvider)
        let navigation = UINavigationController(rootViewController: sheet)
        navigation.modalPresentationStyle = .overFullScreen
        owner.present(navigation, animated: true)
    }

    override func viewDidLoad() {
        super.viewDidLoad()
        title = "Paired Devices"
        navigationItem.leftBarButtonItem = UIBarButtonItem(barButtonSystemItem: .done, target: self, action: #selector(close))
        navigationItem.rightBarButtonItems = [
            UIBarButtonItem(image: UIImage(systemName: "qrcode"), style: .plain, target: self, action: #selector(showQR)),
            UIBarButtonItem(image: UIImage(systemName: "qrcode.viewfinder"), style: .plain, target: self, action: #selector(scan)),
            UIBarButtonItem(image: UIImage(systemName: "photo"), style: .plain, target: self, action: #selector(importQR))
        ]
        navigationItem.rightBarButtonItems?[0].accessibilityLabel = "Show pairing QR"
        navigationItem.rightBarButtonItems?[1].accessibilityLabel = "Scan pairing QR"
        navigationItem.rightBarButtonItems?[2].accessibilityLabel = "Import pairing QR image"
        let header = UIView(frame: CGRect(x: 0, y: 0, width: 320, height: 330))
        imageView.translatesAutoresizingMaskIntoConstraints = false
        imageView.contentMode = .scaleAspectFit
        imageView.backgroundColor = .white
        imageView.layer.magnificationFilter = .nearest
        imageView.clipsToBounds = true
        statusLabel.translatesAutoresizingMaskIntoConstraints = false
        statusLabel.font = .systemFont(ofSize: 15, weight: .medium)
        statusLabel.numberOfLines = 2
        statusLabel.textAlignment = .center
        header.addSubview(imageView)
        header.addSubview(statusLabel)
        NSLayoutConstraint.activate([
            imageView.topAnchor.constraint(equalTo: header.topAnchor, constant: 12),
            imageView.centerXAnchor.constraint(equalTo: header.centerXAnchor),
            imageView.widthAnchor.constraint(equalToConstant: 260),
            imageView.heightAnchor.constraint(equalTo: imageView.widthAnchor),
            statusLabel.topAnchor.constraint(equalTo: imageView.bottomAnchor, constant: 12),
            statusLabel.leadingAnchor.constraint(equalTo: header.leadingAnchor, constant: 16),
            statusLabel.trailingAnchor.constraint(equalTo: header.trailingAnchor, constant: -16)
        ])
        tableView.tableHeaderView = header
        NotificationCenter.default.addObserver(self, selector: #selector(refresh), name: .remotePeersChanged, object: nil)
        NotificationCenter.default.addObserver(self, selector: #selector(backgrounded), name: UIApplication.didEnterBackgroundNotification, object: nil)
        refresh()
    }

    override func viewDidAppear(_ animated: Bool) {
        super.viewDidAppear(animated)
        guard !hasShownInitialQR else { return }
        hasShownInitialQR = true
        showQR()
    }

    override func viewDidLayoutSubviews() {
        super.viewDidLayoutSubviews()
        preview?.frame = imageView.bounds
    }

    @objc private func refresh() {
        if timer != nil, transport.invitation == nil {
            timer?.invalidate()
            timer = nil
            imageView.image = nil
        }
        statusLabel.text = transport.status
        tableView.reloadData()
    }

    @objc private func showQR() {
        stopScanning()
        timer?.invalidate()
        timer = nil
        imageView.image = nil
        operation?.cancel()
        operation = Task {
            do {
                let text = try await transport.showInvitation()
                guard !Task.isCancelled else { return }
                let filter = CIFilter(name: "CIQRCodeGenerator")
                filter?.setValue(Data(text.utf8), forKey: "inputMessage")
                filter?.setValue("M", forKey: "inputCorrectionLevel")
                if let output = filter?.outputImage,
                   let image = CIContext().createCGImage(output.transformed(by: CGAffineTransform(scaleX: 6, y: 6)),
                       from: output.extent.applying(CGAffineTransform(scaleX: 6, y: 6))) {
                    imageView.image = UIImage(cgImage: image)
                }
                statusLabel.text = "Pairing open for 2 minutes"
                timer?.invalidate()
                timer = Timer.scheduledTimer(withTimeInterval: 120, repeats: false) { [weak self] _ in
                    guard let self, self.transport.invitation != nil else { return }
                    self.transport.cancelPairing()
                    self.timer = nil
                    self.imageView.image = nil
                    self.statusLabel.text = "Invitation expired"
                }
            } catch { showError(error) }
        }
    }

    @objc private func scan() {
        operation?.cancel()
        transport.cancelPairing()
        timer?.invalidate()
        imageView.image = nil
        guard !scanning else { stopScanning(); return }
        scanning = true
        let attempt = UUID()
        scanAttempt = attempt
        statusLabel.text = "Checking camera access"
        Task {
            let authorized: Bool
            switch AVCaptureDevice.authorizationStatus(for: .video) {
            case .authorized:
                authorized = true
            case .notDetermined:
                authorized = await AVCaptureDevice.requestAccess(for: .video)
            default:
                authorized = false
            }
            guard scanning, scanAttempt == attempt else { return }
            guard authorized else {
                stopScanning()
                showCameraAccessDenied()
                return
            }
            statusLabel.text = "Scanning"
            if let frameProvider {
                scanTimer = Timer.scheduledTimer(withTimeInterval: 0.3, repeats: true) { [weak self] _ in
                    guard let self, self.scanning, self.scanAttempt == attempt, !self.processing else { return }
                    guard let buffer = frameProvider() else {
                        self.statusLabel.text = "Camera frames unavailable"
                        return
                    }
                    self.statusLabel.text = "Scanning"
                    let image = CIImage(cvPixelBuffer: buffer)
                    if let preview = self.previewContext.createCGImage(image, from: image.extent) {
                        self.imageView.image = UIImage(cgImage: preview)
                    }
                    let request = VNDetectBarcodesRequest()
                    request.symbologies = [.qr]
                    try? VNImageRequestHandler(cvPixelBuffer: buffer).perform([request])
                    if let text = request.results?.first?.payloadStringValue { self.consume(text) }
                }
                return
            }
            do {
                if capture.inputs.isEmpty {
                    guard let camera = AVCaptureDevice.default(for: .video) else { throw RemotePairingError.cancelled }
                    let input = try AVCaptureDeviceInput(device: camera)
                    let output = AVCaptureMetadataOutput()
                    guard capture.canAddInput(input), capture.canAddOutput(output) else { throw RemotePairingError.cancelled }
                    capture.addInput(input)
                    capture.addOutput(output)
                    output.setMetadataObjectsDelegate(self, queue: .main)
                    output.metadataObjectTypes = [.qr]
                }
                let preview = AVCaptureVideoPreviewLayer(session: capture)
                preview.videoGravity = .resizeAspectFill
                preview.frame = imageView.bounds
                imageView.layer.addSublayer(preview)
                self.preview = preview
                let capture = capture
                captureQueue.async { capture.startRunning() }
            } catch { stopScanning(); showError(error) }
        }
    }

    private func showCameraAccessDenied() {
        let restricted = AVCaptureDevice.authorizationStatus(for: .video) == .restricted
        statusLabel.text = restricted ? "Camera access restricted" : "Camera access denied"
        guard presentedViewController == nil else { return }
        let alert = UIAlertController(title: "Camera Access", message: restricted
            ? "Camera access is restricted on this device."
            : "Allow camera access in Settings to scan a pairing QR code.", preferredStyle: .alert)
        alert.addAction(UIAlertAction(title: "Cancel", style: .cancel))
        if !restricted {
            alert.addAction(UIAlertAction(title: "Open Settings", style: .default) { _ in
                #if targetEnvironment(macCatalyst)
                let address = "x-apple.systempreferences:com.apple.preference.security?Privacy_Camera"
                #else
                let address = UIApplication.openSettingsURLString
                #endif
                if let url = URL(string: address) { UIApplication.shared.open(url) }
            })
        }
        present(alert, animated: true)
    }

    func metadataOutput(_ output: AVCaptureMetadataOutput, didOutput metadataObjects: [AVMetadataObject], from connection: AVCaptureConnection) {
        guard scanning, let text = (metadataObjects.first as? AVMetadataMachineReadableCodeObject)?.stringValue else { return }
        consume(text)
    }

    private func consume(_ text: String) {
        guard !processing else { return }
        processing = true
        stopScanning()
        timer?.invalidate()
        timer = nil
        operation?.cancel()
        statusLabel.text = "Connecting to device"
        operation = Task {
            defer { processing = false }
            do { try await transport.pair(text); refresh() }
            catch { showError(error) }
        }
    }

    @objc private func importQR() {
        stopScanning()
        timer?.invalidate()
        timer = nil
        operation?.cancel()
        transport.cancelPairing()
        imageView.image = nil
        let picker = UIDocumentPickerViewController(forOpeningContentTypes: [.image], asCopy: true)
        picker.delegate = self
        present(picker, animated: true)
    }

    func documentPicker(_ controller: UIDocumentPickerViewController, didPickDocumentsAt urls: [URL]) {
        guard let url = urls.first else { return }
        let scoped = url.startAccessingSecurityScopedResource()
        defer { if scoped { url.stopAccessingSecurityScopedResource() } }
        guard let image = UIImage(contentsOfFile: url.path)?.cgImage else { return }
        let request = VNDetectBarcodesRequest()
        request.symbologies = [.qr]
        try? VNImageRequestHandler(cgImage: image).perform([request])
        if let text = request.results?.first?.payloadStringValue { consume(text) }
        else { showError(RemotePairingError.invalidInvitation) }
    }

    private func stopScanning() {
        scanning = false
        scanAttempt = UUID()
        scanTimer?.invalidate()
        scanTimer = nil
        preview?.removeFromSuperlayer()
        preview = nil
        let capture = capture
        captureQueue.async { if capture.isRunning { capture.stopRunning() } }
    }

    @objc private func backgrounded() {
        stopScanning()
        timer?.invalidate()
        operation?.cancel()
        transport.cancelPairing()
        imageView.image = nil
    }

    @objc private func close() {
        backgrounded()
        dismiss(animated: true)
    }

    private func showError(_ error: Error) {
        guard view.window != nil else { return }
        statusLabel.text = error.localizedDescription
    }

    override func numberOfSections(in tableView: UITableView) -> Int { 2 }
    override func tableView(_ tableView: UITableView, numberOfRowsInSection section: Int) -> Int { section == 0 ? 1 : peers.count }
    override func tableView(_ tableView: UITableView, titleForHeaderInSection section: Int) -> String? {
        section == 0 ? "Connection" : "Remembered Devices"
    }

    override func tableView(_ tableView: UITableView, cellForRowAt indexPath: IndexPath) -> UITableViewCell {
        let cell = UITableViewCell(style: .subtitle, reuseIdentifier: nil)
        cell.textLabel?.numberOfLines = 2
        if indexPath.section == 0 {
            cell.textLabel?.text = transport.isConnected ? "Disconnect \(transport.connectedPeer?.name ?? "device")" : "No device connected"
            cell.imageView?.image = UIImage(systemName: "link")
        } else {
            let peer = peers[indexPath.row]
            cell.textLabel?.text = peer.name
            cell.detailTextLabel?.text = "\(peer.role.rawValue.capitalized)  \(peer.id.prefix(12))"
            cell.accessoryType = peer.id == transport.connectedPeer?.id ? .checkmark : .none
            let remove = UIButton(type: .system)
            remove.setImage(UIImage(systemName: "trash"), for: .normal)
            remove.tintColor = .systemRed
            remove.accessibilityLabel = "Delete \(peer.name)"
            remove.toolTip = "Delete remembered device"
            remove.frame = CGRect(x: 0, y: 0, width: 44, height: 44)
            remove.addAction(UIAction { [weak self] _ in
                self?.confirmRemoval(of: peer)
            }, for: .touchUpInside)
            cell.accessoryView = remove
            cell.imageView?.image = peer.id == transport.connectedPeer?.id
                ? UIImage(systemName: "checkmark.circle.fill") : UIImage(systemName: "desktopcomputer")
        }
        return cell
    }

    override func tableView(_ tableView: UITableView, didSelectRowAt indexPath: IndexPath) {
        tableView.deselectRow(at: indexPath, animated: true)
        if indexPath.section == 0 { transport.disconnect() }
        else if transport.role == .controller, peers[indexPath.row].role == .robot {
            transport.connectToPeer(peers[indexPath.row].id)
        }
    }

    override func tableView(_ tableView: UITableView, trailingSwipeActionsConfigurationForRowAt indexPath: IndexPath) -> UISwipeActionsConfiguration? {
        guard indexPath.section == 1 else { return nil }
        let peer = peers[indexPath.row]
        let action = UIContextualAction(style: .destructive, title: "Remove Pairing") { [weak self] _, _, completion in
            guard let self else { completion(false); return }
            self.confirmRemoval(of: peer, completion: completion)
        }
        return UISwipeActionsConfiguration(actions: [action])
    }

    private func confirmRemoval(of peer: RemotePeer, completion: @escaping (Bool) -> Void = { _ in }) {
        let alert = UIAlertController(title: "Remove \(peer.name)?", message: "A new QR scan will be required to connect again.", preferredStyle: .alert)
        alert.addAction(UIAlertAction(title: "Cancel", style: .cancel) { _ in completion(false) })
        alert.addAction(UIAlertAction(title: "Remove Pairing", style: .destructive) { [weak self] _ in
            guard let self else { completion(false); return }
            do { try self.transport.removePeer(peer.id); completion(true) }
            catch { self.showError(error); completion(false) }
        })
        present(alert, animated: true)
    }
}