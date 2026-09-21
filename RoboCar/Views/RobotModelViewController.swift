import UIKit
import UniformTypeIdentifiers
import SceneKit
import GLTFKit2

final class RobotModelViewController: PanelViewController, UIDocumentPickerDelegate, UITableViewDataSource, UITableViewDelegate, UISearchBarDelegate {
    let motors: RobotRigMotorController
    private var viewport = RobotModelSceneView()
    private let viewportHost = UIView()
    private let table = UITableView(frame: .zero, style: .plain)
    private let search = UISearchBar()
    private let status = UILabel()
    private let angle = UISlider()
    private let angleLabel = UILabel()
    private let groupButton = UIButton(type: .system)
    private let mode = UISegmentedControl(items: ["Preview", "Live"])
    private let split = UIStackView()
    private let inspector = UIStackView()
    private let inspectorScroll = UIScrollView()
    private let axisStateLabel = UILabel()
    private let selectionLabel = UILabel()
    private let viewportModeLabel = UILabel()
    private var actionButtons: [Selector: UIButton] = [:]
    private var document = RobotRigDocument(assetHash: "")
    private var assetURL: URL?
    private var selected = Set<String>()
    private var activeGroupID: UUID?
    private var angles: [UUID: Float] = [:]
    private var isolated = false
    private var pickingAxis = false
    private var movingAxis = false
    private var pickingIK = false
    private var ikTarget: SIMD3<Float>?
    private var ikDragAngles: [UUID: Float]?
    private var ikMessage: String?
    private let ikStateLabel = UILabel()
    private var pendingSurface: (String, RobotRigSurface)?
    private var undoStack: [RobotRigDocument] = []
    private var redoStack: [RobotRigDocument] = []
    private var importToken = UUID()
    private var importing = false
    private var importError: String?
    private var sidebarWidth: NSLayoutConstraint?
    private var sidebarHeight: NSLayoutConstraint?
    #if DEBUG
    private var runtimeCheck: (() -> Void)?
    #endif

    init(motors: RobotRigMotorController) {
        self.motors = motors
        super.init(nibName: nil, bundle: nil)
    }

    required init?(coder: NSCoder) { fatalError("init(coder:) has not been implemented") }

    override var keyCommands: [UIKeyCommand]? {
        let deselect = UIKeyCommand(input: "a", modifierFlags: [.command, .shift], action: #selector(deselectAllParts))
        deselect.discoverabilityTitle = "Deselect All"
        return (super.keyCommands ?? []) + [deselect]
    }

    override func canPerformAction(_ action: Selector, withSender sender: Any?) -> Bool {
        if action == #selector(deselectAllParts) { return canDeselectParts }
        return super.canPerformAction(action, withSender: sender)
    }

    private var canDeselectParts: Bool {
        !motors.isArmed && !importing && (!selected.isEmpty || pickingAxis || movingAxis || pickingIK || ikTarget != nil || pendingSurface != nil)
    }

    override func viewDidLoad() {
        super.viewDidLoad()
        view.backgroundColor = .systemBackground
        let toolbar = UIStackView(arrangedSubviews: [
            button("Import GLB", "square.and.arrow.down", #selector(importTapped)),
            button("Frame Model", "viewfinder", #selector(frameTapped)),
            button("Isolate Parts", "eye", #selector(isolateTapped)),
            button("Undo", "arrow.uturn.backward", #selector(undoTapped)),
            button("Redo", "arrow.uturn.forward", #selector(redoTapped)),
            button("Close", "xmark", #selector(closeTapped))
        ])
        toolbar.spacing = 8
        let toolbarScroll = UIScrollView()
        toolbarScroll.showsHorizontalScrollIndicator = true
        toolbarScroll.heightAnchor.constraint(equalToConstant: 52).isActive = true
        toolbar.translatesAutoresizingMaskIntoConstraints = false
        toolbarScroll.addSubview(toolbar)
        NSLayoutConstraint.activate([
            toolbar.topAnchor.constraint(equalTo: toolbarScroll.contentLayoutGuide.topAnchor),
            toolbar.bottomAnchor.constraint(equalTo: toolbarScroll.contentLayoutGuide.bottomAnchor),
            toolbar.leadingAnchor.constraint(equalTo: toolbarScroll.contentLayoutGuide.leadingAnchor),
            toolbar.trailingAnchor.constraint(equalTo: toolbarScroll.contentLayoutGuide.trailingAnchor),
            toolbar.heightAnchor.constraint(equalTo: toolbarScroll.frameLayoutGuide.heightAnchor)
        ])
        for control in toolbar.arrangedSubviews { control.widthAnchor.constraint(equalToConstant: 126).isActive = true }
        status.font = .systemFont(ofSize: 13)
        status.numberOfLines = 2
        status.text = "No model selected"
        status.heightAnchor.constraint(equalToConstant: 38).isActive = true
        viewportHost.addSubview(viewport)
        attachViewport()
        viewportModeLabel.translatesAutoresizingMaskIntoConstraints = false
        viewportModeLabel.font = .systemFont(ofSize: 14, weight: .semibold)
        viewportModeLabel.numberOfLines = 0
        viewportModeLabel.textColor = .white
        viewportModeLabel.backgroundColor = UIColor.black.withAlphaComponent(0.7)
        viewportModeLabel.textAlignment = .center
        viewportHost.addSubview(viewportModeLabel)
        NSLayoutConstraint.activate([
            viewportModeLabel.topAnchor.constraint(equalTo: viewportHost.topAnchor, constant: 8),
            viewportModeLabel.centerXAnchor.constraint(equalTo: viewportHost.centerXAnchor),
            viewportModeLabel.widthAnchor.constraint(lessThanOrEqualTo: viewportHost.widthAnchor, constant: -16),
            viewportModeLabel.heightAnchor.constraint(greaterThanOrEqualToConstant: 36)
        ])
        search.placeholder = "Parts"
        search.delegate = self
        search.searchBarStyle = .minimal
        table.dataSource = self
        table.delegate = self
        table.rowHeight = 44
        var groupConfiguration = UIButton.Configuration.tinted()
        groupConfiguration.title = "Choose Group"
        groupConfiguration.image = UIImage(systemName: "chevron.down")
        groupConfiguration.imagePlacement = .trailing
        groupConfiguration.imagePadding = 8
        groupConfiguration.cornerStyle = .small
        groupButton.configuration = groupConfiguration
        groupButton.showsMenuAsPrimaryAction = true
        let groupTools = actionGrid([
            button("Create Group", "folder.badge.plus", #selector(createGroup)),
            button("Group Settings", "slider.horizontal.3", #selector(groupSettings)),
            button("Add Parts", "plus", #selector(assignSelection)),
            button("Remove Parts", "minus", #selector(removeSelection))
        ])
        let axisTools = actionGrid([
            button("Select Axis Face", "scope", #selector(setAxis)),
            button("Use Selected Face", "checkmark", #selector(confirmAxis)),
            button("Flip Axis", "arrow.up.arrow.down", #selector(flipAxis)),
            button("Edit Axis", "pencil", #selector(editAxis)),
            button("Move Axis", "move.3d", #selector(toggleMoveAxis))
        ])
        let ikTools = actionGrid([
            button("Place IK Helper", "mappin.and.ellipse", #selector(placeIKHelper)),
            button("Edit IK Helper", "pencil", #selector(editIKHelper)),
            button("IK Chain Root", "point.3.connected.trianglepath.dotted", #selector(chooseIKRoot)),
            button("Drag IK Target", "move.3d", #selector(toggleIK)),
            button("Remove IK Helper", "trash", #selector(removeIKHelper))
        ])
        for label in [selectionLabel, axisStateLabel, ikStateLabel] {
            label.font = .systemFont(ofSize: 13, weight: .medium)
            label.numberOfLines = 0
        }
        axisStateLabel.accessibilityTraits.insert(.updatesFrequently)
        angle.minimumValue = -180
        angle.maximumValue = 180
        angle.addTarget(self, action: #selector(angleChanged), for: .valueChanged)
        angleLabel.font = .monospacedSystemFont(ofSize: 13, weight: .medium)
        angleLabel.textAlignment = .center
        angleLabel.isUserInteractionEnabled = true
        angleLabel.accessibilityLabel = "Joint angle, tap to enter a value"
        angleLabel.addGestureRecognizer(UITapGestureRecognizer(target: self, action: #selector(enterAngle)))
        angleLabel.heightAnchor.constraint(equalToConstant: 22).isActive = true
        mode.selectedSegmentIndex = 0
        mode.heightAnchor.constraint(equalToConstant: 32).isActive = true
        angle.heightAnchor.constraint(equalToConstant: 32).isActive = true
        mode.addTarget(self, action: #selector(modeChanged), for: .valueChanged)
        let motorTools = actionGrid([
            button("Motor Binding", "link", #selector(bindMotor)),
            button("Move to Target", "play.fill", #selector(moveMotor)),
            button("Stop Motion", "stop.fill", #selector(stopMotor)),
            button("Reset Preview", "arrow.counterclockwise", #selector(resetPreview))
        ])
        inspector.axis = .vertical
        inspector.spacing = 8
        let selectionTools = actionGrid([button("Deselect All", "selection.pin.in.out", #selector(deselectAllParts))])
        actionButtons[#selector(deselectAllParts)]?.toolTip = "Deselect All (Command-Shift-A)"
        for child in [sectionHeading("1. Parts & Groups"), groupButton, selectionLabel, selectionTools, groupTools,
                      sectionHeading("2. Rotation Axis"), axisStateLabel, axisTools,
                      sectionHeading("3. Inverse Kinematics"), ikStateLabel, ikTools,
                      sectionHeading("4. Rotation & Motor"), mode, angleLabel, angle, motorTools,
                      sectionHeading("Model Parts"), search, table] { inspector.addArrangedSubview(child) }
        groupButton.heightAnchor.constraint(equalToConstant: 44).isActive = true
        search.heightAnchor.constraint(equalToConstant: 44).isActive = true
        table.heightAnchor.constraint(equalToConstant: 200).isActive = true
        inspector.translatesAutoresizingMaskIntoConstraints = false
        inspectorScroll.addSubview(inspector)
        inspectorScroll.keyboardDismissMode = .interactive
        NSLayoutConstraint.activate([
            inspector.topAnchor.constraint(equalTo: inspectorScroll.contentLayoutGuide.topAnchor),
            inspector.bottomAnchor.constraint(equalTo: inspectorScroll.contentLayoutGuide.bottomAnchor),
            inspector.leadingAnchor.constraint(equalTo: inspectorScroll.contentLayoutGuide.leadingAnchor),
            inspector.trailingAnchor.constraint(equalTo: inspectorScroll.contentLayoutGuide.trailingAnchor),
            inspector.widthAnchor.constraint(equalTo: inspectorScroll.frameLayoutGuide.widthAnchor)
        ])
        split.spacing = 10
        split.addArrangedSubview(viewportHost)
        split.addArrangedSubview(inspectorScroll)
        let layout = UIStackView(arrangedSubviews: [toolbarScroll, split, status])
        layout.axis = .vertical
        layout.spacing = 6
        layout.translatesAutoresizingMaskIntoConstraints = false
        view.addSubview(layout)
        NSLayoutConstraint.activate([
            layout.topAnchor.constraint(equalTo: view.safeAreaLayoutGuide.topAnchor),
            layout.bottomAnchor.constraint(equalTo: view.keyboardLayoutGuide.topAnchor),
            layout.leadingAnchor.constraint(equalTo: view.safeAreaLayoutGuide.leadingAnchor, constant: 10),
            layout.trailingAnchor.constraint(equalTo: view.safeAreaLayoutGuide.trailingAnchor, constant: -10)
        ])
        sidebarWidth = inspectorScroll.widthAnchor.constraint(equalToConstant: 350)
        sidebarHeight = inspectorScroll.heightAnchor.constraint(equalToConstant: 360)
        motors.onChange = { [weak self] in self?.motorUpdate() }
          if !ProcessInfo.processInfo.arguments.contains("--robot-rig-checks"),
              let hash = UserDefaults.standard.string(forKey: "RobotModel.lastAsset"),
           hash.count == 64, hash.allSatisfy({ $0.isHexDigit }) {
            load(url: RobotModelStorage.root.appendingPathComponent(hash).appendingPathComponent("model.glb"))
        }
        refresh()
    }

    override func viewDidLayoutSubviews() {
        super.viewDidLayoutSubviews()
        let wide = view.bounds.width >= 700 || view.bounds.width > view.bounds.height
        sidebarWidth?.isActive = false
        sidebarHeight?.isActive = false
        split.axis = wide ? .horizontal : .vertical
        sidebarWidth?.isActive = wide
        sidebarHeight?.constant = min(360, view.bounds.height * 0.45)
        sidebarHeight?.isActive = !wide
    }

    override func viewDidAppear(_ animated: Bool) {
        super.viewDidAppear(animated)
        motors.setActive(true)
        #if DEBUG
        if let check = runtimeCheck {
            runtimeCheck = nil
            DispatchQueue.main.asyncAfter(deadline: .now() + 0.5) { check() }
        }
        #endif
    }

    override func viewWillDisappear(_ animated: Bool) {
        super.viewWillDisappear(animated)
        viewport.cancelAxisDrag()
        motors.setActive(false)
    }

    override func panelResignedKey() {
        viewport.cancelAxisDrag()
        motors.disarm()
        mode.selectedSegmentIndex = 0
    }

    private var activeGroup: RobotRigGroup? { document.groups.first { $0.id == activeGroupID } }
    private var partIDs: Set<String> { Set(viewport.parts.map(\.id)) }
    private var filteredParts: [RobotModelSceneView.Part] {
        let query = search.text ?? ""
        return viewport.parts.filter { query.isEmpty || $0.name.localizedCaseInsensitiveContains(query) }
    }

    private func button(_ title: String, _ symbol: String, _ action: Selector) -> UIButton {
        let result = UIButton(type: .system)
        var configuration = UIButton.Configuration.tinted()
        configuration.title = title
        configuration.image = UIImage(systemName: symbol)
        configuration.imagePadding = 6
        configuration.cornerStyle = .small
        configuration.contentInsets = NSDirectionalEdgeInsets(top: 6, leading: 8, bottom: 6, trailing: 8)
        configuration.titleTextAttributesTransformer = UIConfigurationTextAttributesTransformer { incoming in
            var attributes = incoming
            attributes.font = .systemFont(ofSize: 13, weight: .medium)
            return attributes
        }
        result.configuration = configuration
        result.titleLabel?.numberOfLines = 2
        result.accessibilityLabel = title
        result.toolTip = title
        result.addTarget(self, action: action, for: .touchUpInside)
        actionButtons[action] = result
        return result
    }

    private func actionGrid(_ buttons: [UIButton]) -> UIStackView {
        let grid = UIStackView()
        grid.axis = .vertical
        grid.spacing = 6
        for offset in stride(from: 0, to: buttons.count, by: 2) {
            let row = UIStackView(arrangedSubviews: Array(buttons[offset..<min(offset + 2, buttons.count)]))
            row.distribution = .fillEqually
            row.spacing = 6
            row.heightAnchor.constraint(equalToConstant: 50).isActive = true
            grid.addArrangedSubview(row)
        }
        return grid
    }

    private func sectionHeading(_ title: String) -> UILabel {
        let label = UILabel()
        label.text = title
        label.font = .systemFont(ofSize: 15, weight: .semibold)
        label.numberOfLines = 0
        return label
    }

    private func updateAction(_ action: Selector, enabled: Bool, title: String? = nil) {
        guard let button = actionButtons[action] else { return }
        button.isEnabled = enabled
        if let title {
            button.configuration?.title = title
            button.accessibilityLabel = title
            button.toolTip = title
        }
    }

    private func attachViewport() {
        viewport.translatesAutoresizingMaskIntoConstraints = false
        NSLayoutConstraint.activate([
            viewport.topAnchor.constraint(equalTo: viewportHost.topAnchor), viewport.bottomAnchor.constraint(equalTo: viewportHost.bottomAnchor),
            viewport.leadingAnchor.constraint(equalTo: viewportHost.leadingAnchor), viewport.trailingAnchor.constraint(equalTo: viewportHost.trailingAnchor)
        ])
        viewport.onPick = { [weak self] part, triangle in self?.picked(part: part, triangle: triangle) }
        viewport.onAxisMove = { [weak self] axis, state in self?.axisMoved(axis, state: state) }
        viewport.onIKDragBegan = { [weak self] in self?.ikDragAngles = self?.angles }
        viewport.onIKMove = { [weak self] target, state in self?.moveIK(target, state: state) }
    }

    private func refresh() {
        if motors.isArmed || importing || activeGroup == nil {
            ikTarget = nil
            pickingIK = false
        }
        viewport.onHelperPick = pickingIK ? { [weak self] part, point in self?.pickedIKHelper(part: part, point: point) } : nil
        if movingAxis && (activeGroup?.axis == nil || motors.isArmed || importing || pickingAxis) {
            movingAxis = false
            viewport.cancelAxisDrag()
        }
        groupButton.configuration?.title = activeGroup.map { "Group: \($0.name)" } ?? "Unassigned Parts"
        groupButton.accessibilityLabel = groupButton.configuration?.title
        groupButton.menu = UIMenu(children: [UIAction(title: "Unassigned Parts", state: activeGroupID == nil ? .on : .off) { [weak self] _ in self?.selectGroup(nil) }] + document.groups.map { group in
            UIAction(title: group.name, state: group.id == activeGroupID ? .on : .off) { [weak self] _ in self?.selectGroup(group.id) }
        })
        viewport.update(document: document, angles: angles, selected: selected, isolated: isolated)
        if let target = ikTarget, let group = activeGroup, let endpoint = document.ikEndpoint(for: group.id, angles: angles) {
            viewport.showIK(target: target, endpoint: endpoint, parts: group.parts)
        } else if let pendingSurface {
            viewport.showAxis(pendingSurface.1.axis, surface: pendingSurface.1, partID: pendingSurface.0)
        } else {
            let transform = activeGroupID.flatMap { document.transforms(angles: angles)[$0] } ?? matrix_identity_float4x4
            viewport.showAxis(activeGroup?.axis, transform: transform, movable: movingAxis)
        }
        let binding = activeGroup?.motor
        angle.minimumValue = Float(binding?.minimum ?? -180)
        angle.maximumValue = Float(binding?.maximum ?? 180)
        angle.isEnabled = activeGroup?.axis != nil && !importing
        let measured = activeGroupID.flatMap { angles[$0] } ?? 0
        if !motors.isArmed { angle.value = measured }
        angleLabel.text = motors.isArmed ? String(format: "Actual %.1f / Target %.1f deg", measured, angle.value) : String(format: "%.1f deg", measured)
        mode.selectedSegmentIndex = motors.isArmed ? 1 : 0
        let editable = !motors.isArmed && !importing && assetURL != nil
        let group = activeGroup
        viewportModeLabel.text = pickingAxis
            ? (pendingSurface == nil ? "  Select Axis Face: \(group?.name ?? "")  " : "  Selected Face: Confirm or Select Another  ")
            : (motors.isArmed ? "  Live Motor Control  " : "  Select Parts  ")
        if movingAxis { viewportModeLabel.text = "  Move Axis: X / Y / Z  " }
        if pickingIK { viewportModeLabel.text = "  Place IK Helper: \(group?.name ?? "")  " }
        if ikTarget != nil { viewportModeLabel.text = "  IK Preview: Drag Child or XYZ Target  " }
        viewportHost.bringSubviewToFront(viewportModeLabel)
        selectionLabel.text = "\(selected.count) selected parts" + (group.map { " / \($0.parts.count) in group" } ?? "")
        if pickingAxis {
            axisStateLabel.text = pendingSurface == nil
                ? "Selecting face on \(group?.name ?? "group")"
                : "Face selected: \(pendingSurface!.1.triangles.count) flat triangles. Axis awaiting confirmation."
            axisStateLabel.textColor = .systemOrange
        } else {
            axisStateLabel.text = group == nil ? "No active group" : (group?.axis == nil ? "Rotation axis not set" : "Rotation axis set")
            axisStateLabel.textColor = group?.axis == nil ? .secondaryLabel : .systemGreen
        }
        updateAction(#selector(createGroup), enabled: editable && !selected.isEmpty && !pickingAxis)
        updateAction(#selector(deselectAllParts), enabled: canDeselectParts)
        updateAction(#selector(groupSettings), enabled: editable && group != nil && !pickingAxis)
        updateAction(#selector(assignSelection), enabled: editable && group != nil && !selected.isEmpty && !pickingAxis)
        updateAction(#selector(removeSelection), enabled: editable && group != nil && !selected.isEmpty && !pickingAxis)
        updateAction(#selector(setAxis), enabled: editable && group?.parts.isEmpty == false,
                     title: pickingAxis ? "Cancel Face Selection" : "Select Axis Face")
        updateAction(#selector(confirmAxis), enabled: editable && pendingSurface != nil)
        updateAction(#selector(flipAxis), enabled: editable && (pendingSurface != nil || group?.axis != nil))
        updateAction(#selector(editAxis), enabled: editable && group != nil && !pickingAxis)
        updateAction(#selector(toggleMoveAxis), enabled: editable && group?.axis != nil && !pickingAxis,
                 title: movingAxis ? "Done Moving Axis" : "Move Axis")
        updateAction(#selector(bindMotor), enabled: editable && group != nil && !pickingAxis)
        let helper = group?.ikHelper
        let rootName = document.groups.first { $0.id == helper?.rootID }?.name ?? ""
        ikStateLabel.text = pickingIK ? "Selecting helper point on child" : (ikMessage ?? (helper == nil ? "No IK helper" : "Chain root: \(rootName) / Preview only"))
        updateAction(#selector(placeIKHelper), enabled: editable && group?.parts.isEmpty == false,
                 title: pickingIK ? "Cancel Helper Pick" : "Place IK Helper")
        updateAction(#selector(editIKHelper), enabled: editable && group != nil)
        updateAction(#selector(chooseIKRoot), enabled: editable && helper != nil)
        updateAction(#selector(toggleIK), enabled: editable && helper != nil,
                 title: ikTarget == nil ? "Drag IK Target" : "Done With IK")
        updateAction(#selector(removeIKHelper), enabled: editable && helper != nil)
        updateAction(#selector(moveMotor), enabled: motors.isArmed && !importing)
        updateAction(#selector(stopMotor), enabled: motors.isArmed)
        updateAction(#selector(resetPreview), enabled: !viewport.parts.isEmpty && !importing)
        updateAction(#selector(undoTapped), enabled: editable && !undoStack.isEmpty)
        updateAction(#selector(redoTapped), enabled: editable && !redoStack.isEmpty)
        updateAction(#selector(isolateTapped), enabled: !viewport.parts.isEmpty && !importing, title: isolated ? "Show All Parts" : "Isolate Parts")
        angle.isEnabled = angle.isEnabled && !pickingAxis && !movingAxis && !pickingIK && ikTarget == nil
        mode.isEnabled = !movingAxis && !pickingIK && ikTarget == nil
        table.reloadData()
    }

    private func selectGroup(_ id: UUID?) {
        stopIK()
        viewport.cancelAxisDrag()
        movingAxis = false
        motors.disarm()
        activeGroupID = id
        selected = activeGroup?.parts ?? []
        if id == nil { isolated = false }
        pendingSurface = nil
        pickingAxis = false
        refresh()
    }

    @objc private func deselectAllParts() {
        guard canDeselectParts else { return }
        stopIK()
        viewport.cancelAxisDrag()
        movingAxis = false
        selected.removeAll()
        pendingSurface = nil
        pickingAxis = false
        isolated = false
        refresh()
    }

    private func edit(_ body: (inout RobotRigDocument) throws -> Void) {
        guard !motors.isArmed, !importing, let url = assetURL else { showError("Return to Preview before editing"); return }
        stopIK()
        viewport.cancelAxisDrag()
        do {
            var updated = document
            try body(&updated)
            try updated.validate(partIDs: partIDs)
            try RobotModelStorage.save(updated, at: url)
            undoStack.append(document)
            if undoStack.count > 50 { undoStack.removeFirst() }
            redoStack.removeAll()
            document = updated
            angles.removeAll()
            pendingSurface = nil
            pickingAxis = false
            refresh()
            status.text = "Saved"
        } catch { showError(error.localizedDescription) }
    }

    @objc private func importTapped() {
        motors.disarm()
        importToken = UUID()
        importing = false
        let picker = UIDocumentPickerViewController(forOpeningContentTypes: [UTType(filenameExtension: "glb") ?? .data], asCopy: false)
        picker.delegate = self
        present(picker, animated: true)
    }

    func documentPicker(_ controller: UIDocumentPickerViewController, didPickDocumentsAt urls: [URL]) {
        if let url = urls.first { load(url: url) }
    }

    private func load(url: URL) {
        stopIK()
        viewport.cancelAxisDrag()
        movingAxis = false
        motors.disarm()
        importError = nil
        status.textColor = .label
        let token = UUID()
        importToken = token
        importing = true
        status.text = "Loading model..."
        do {
            let (local, hash) = try RobotModelStorage.stage(url)
            GLTFAsset.load(with: local, options: [:]) { [weak self] progress, state, asset, error, _ in
                DispatchQueue.main.async {
                    guard let self, self.importToken == token else { return }
                    if let error {
                        self.importing = false
                        self.showImportError(error.localizedDescription)
                        return
                    }
                    if state == .error {
                        self.importing = false
                        self.showImportError("GLB importer failed")
                        return
                    }
                    guard state == .complete, let asset else {
                        self.status.text = "Loading \(Int(progress * 100))%"
                        return
                    }
                    do {
                        let source = GLTFSCNSceneSource(asset: asset)
                        guard let scene = source.defaultScene else { throw RobotRigError.invalid("GLB has no default scene") }
                        let next = RobotModelSceneView()
                        try next.install(scene)
                        let saved = try RobotModelStorage.loadDocument(at: local, hash: hash)
                        try saved.validate(partIDs: Set(next.parts.map(\.id)))
                        try RobotModelStorage.save(saved, at: local)
                        self.viewport.removeFromSuperview()
                        self.viewport = next
                        self.viewportHost.addSubview(next)
                        self.attachViewport()
                        self.document = saved
                        self.assetURL = local
                        self.selected.removeAll()
                        self.activeGroupID = nil
                        self.angles.removeAll()
                        self.undoStack.removeAll()
                        self.redoStack.removeAll()
                        self.pendingSurface = nil
                        self.pickingAxis = false
                        self.isolated = false
                        self.importing = false
                        self.status.text = "\(next.parts.count) parts"
                        self.refresh()
                    } catch { self.importing = false; self.showImportError(error.localizedDescription) }
                }
            }
        } catch { importing = false; showImportError(error.localizedDescription) }
    }

    private func picked(part: String, triangle: Int) {
        guard !motors.isArmed, !importing, !movingAxis, !pickingIK, ikTarget == nil else { return }
        if pickingAxis {
            guard activeGroup?.parts.contains(part) == true, let mesh = viewport.parts.first(where: { $0.id == part })?.mesh else {
                showError("Choose a face belonging to the selected group")
                return
            }
            do {
                let surface = try mesh.surface(at: triangle)
                pendingSurface = (part, surface)
                status.text = "\(surface.triangles.count) coplanar triangles selected"
            } catch { showError(error.localizedDescription) }
        } else {
            if !selected.insert(part).inserted { selected.remove(part) }
        }
        refresh()
    }

    @objc private func createGroup() {
        guard !selected.isEmpty else { showError("Select at least one part"); return }
        guard selected.isDisjoint(with: document.groups.reduce(into: Set<String>()) { $0.formUnion($1.parts) }) else {
            showError("Remove these parts from their current group first")
            return
        }
        prompt(title: "New Group", fields: [("Name", "")]) { [weak self] values in
            guard let self else { return }
            let group = RobotRigGroup(name: values[0].trimmingCharacters(in: .whitespacesAndNewlines), parts: self.selected)
            self.edit { $0.groups.append(group) }
            if self.document.groups.contains(where: { $0.id == group.id }) {
                self.activeGroupID = group.id
                self.pickingAxis = true
                self.refresh()
                self.inspectorScroll.scrollRectToVisible(self.axisStateLabel.convert(self.axisStateLabel.bounds, to: self.inspector), animated: true)
            }
        }
    }

    @objc private func assignSelection() {
        guard let id = activeGroupID else { return }
        let chosen = selected
        let transfer = document.groups.contains { $0.id != id && !$0.parts.isDisjoint(with: chosen) }
        let apply = { [weak self] in self?.edit { draft in
            for index in draft.groups.indices {
                if draft.groups[index].id == id { draft.groups[index].parts.formUnion(chosen) }
                else { draft.groups[index].parts.subtract(chosen) }
            }
        } }
        if transfer { confirm("Transfer Parts", message: "Move selected parts out of their current groups?", action: { apply() }) }
        else { apply() }
    }

    @objc private func removeSelection() {
        guard let id = activeGroupID else { return }
        let chosen = selected
        edit { draft in
            if let index = draft.groups.firstIndex(where: { $0.id == id }) { draft.groups[index].parts.subtract(chosen) }
        }
    }

    @objc private func groupSettings() {
        guard let group = activeGroup else { return }
        let menu = UIAlertController(title: group.name, message: nil, preferredStyle: .actionSheet)
        menu.addAction(UIAlertAction(title: "Rename", style: .default) { [weak self] _ in
            self?.prompt(title: "Rename Group", fields: [("Name", group.name)]) { values in
                self?.edit { draft in if let index = draft.groups.firstIndex(where: { $0.id == group.id }) { draft.groups[index].name = values[0] } }
            }
        })
        menu.addAction(UIAlertAction(title: "Parent Group", style: .default) { [weak self] _ in self?.chooseParent() })
        menu.addAction(UIAlertAction(title: "Delete Group", style: .destructive) { [weak self] _ in
            self?.edit { draft in
                draft.groups.removeAll { $0.id == group.id }
                for index in draft.groups.indices where draft.groups[index].parentID == group.id { draft.groups[index].parentID = group.parentID }
                for index in draft.groups.indices where draft.groups[index].ikHelper?.rootID == group.id {
                    let rootID = group.parentID ?? draft.groups[index].id
                    draft.groups[index].ikHelper?.rootID = rootID
                }
            }
            self?.activeGroupID = nil
            self?.refresh()
        })
        menu.addAction(UIAlertAction(title: "Cancel", style: .cancel))
        showSheet(menu)
    }

    private func chooseParent() {
        guard let group = activeGroup else { return }
        let menu = UIAlertController(title: "Parent Group", message: nil, preferredStyle: .actionSheet)
        func add(_ id: UUID?, _ name: String) {
            var test = document
            guard let index = test.groups.firstIndex(where: { $0.id == group.id }) else { return }
            test.groups[index].parentID = id
            guard (try? test.validate(partIDs: partIDs)) != nil else { return }
            menu.addAction(UIAlertAction(title: name, style: .default) { [weak self] _ in self?.edit { $0.groups[index].parentID = id } })
        }
        add(nil, "Model Root")
        for candidate in document.groups { add(candidate.id, candidate.name) }
        menu.addAction(UIAlertAction(title: "Cancel", style: .cancel))
        showSheet(menu)
    }

    @objc private func setAxis() {
        guard !motors.isArmed, activeGroup != nil else { return }
        stopIK()
        viewport.cancelAxisDrag()
        movingAxis = false
        angles.removeAll()
        pickingAxis.toggle()
        pendingSurface = nil
        status.text = pickingAxis ? "Select axis face" : "Preview"
        refresh()
    }

    @objc private func confirmAxis() {
        guard let id = activeGroupID, let axis = pendingSurface?.1.axis else { return }
        edit { draft in if let index = draft.groups.firstIndex(where: { $0.id == id }) { draft.groups[index].axis = axis } }
    }

    @objc private func flipAxis() {
        if pendingSurface != nil {
            pendingSurface?.1.axis.direction *= -1
            refresh()
        } else if let id = activeGroupID {
            edit { draft in if let index = draft.groups.firstIndex(where: { $0.id == id }), let axis = draft.groups[index].axis {
                draft.groups[index].axis = RobotRigAxis(origin: axis.origin, direction: -axis.direction)
            } }
        }
    }

    @objc private func editAxis() {
        guard let group = activeGroup else { return }
        let axis = group.axis ?? RobotRigAxis(origin: .zero, direction: SIMD3(0, 1, 0))
        prompt(title: "Axis (Model Rest Coordinates)", fields: [
            ("Origin X", "\(axis.origin.x)"), ("Origin Y", "\(axis.origin.y)"), ("Origin Z", "\(axis.origin.z)"),
            ("Direction X", "\(axis.direction.x)"), ("Direction Y", "\(axis.direction.y)"), ("Direction Z", "\(axis.direction.z)")
        ]) { [weak self] values in
            let numbers = values.compactMap(Float.init)
            guard numbers.count == 6, numbers.allSatisfy(\.isFinite) else { self?.showError("Enter six finite numbers"); return }
            let direction = SIMD3(numbers[3], numbers[4], numbers[5])
            guard simd_length(direction) > 0.000001 else { self?.showError("Direction cannot be zero"); return }
            self?.edit { draft in if let index = draft.groups.firstIndex(where: { $0.id == group.id }) {
                draft.groups[index].axis = RobotRigAxis(origin: SIMD3(numbers[0], numbers[1], numbers[2]), direction: simd_normalize(direction))
            } }
        }
    }

    @objc private func toggleMoveAxis() {
        guard !motors.isArmed, !importing, activeGroup?.axis != nil, !pickingAxis else { return }
        stopIK()
        viewport.cancelAxisDrag()
        movingAxis.toggle()
        angles.removeAll()
        refresh()
    }

    private func axisMoved(_ axis: RobotRigAxis, state: UIGestureRecognizer.State) {
        guard movingAxis, !motors.isArmed, !importing, let id = activeGroupID else { return }
        if state == .ended {
            guard axis != activeGroup?.axis else { refresh(); return }
            edit { draft in
                if let index = draft.groups.firstIndex(where: { $0.id == id }) { draft.groups[index].axis = axis }
            }
            refresh()
        } else if state == .cancelled || state == .failed {
            refresh()
        } else {
            axisStateLabel.text = String(format: "Pivot X %.4f / Y %.4f / Z %.4f", axis.origin.x, axis.origin.y, axis.origin.z)
        }
    }

    @objc private func angleChanged() {
        guard let id = activeGroupID else { return }
        if !motors.isArmed { angles[id] = angle.value }
        refresh()
    }

    private func stopIK() {
        viewport.cancelAxisDrag()
        pickingIK = false
        ikTarget = nil
        ikDragAngles = nil
        ikMessage = nil
    }

    private func defaultIKRoot(_ group: RobotRigGroup) -> UUID {
        var root = group
        while let parent = document.groups.first(where: { $0.id == root.parentID }) { root = parent }
        return root.id
    }

    private func revealIKControls() {
        view.layoutIfNeeded()
        let maximum = max(0, inspectorScroll.contentSize.height - inspectorScroll.bounds.height)
        inspectorScroll.setContentOffset(CGPoint(x: 0, y: min(maximum, max(0, ikStateLabel.frame.minY - 30))), animated: false)
    }

    @objc private func placeIKHelper() {
        guard !motors.isArmed, !importing, activeGroup != nil else { return }
        let wasPicking = pickingIK
        stopIK()
        movingAxis = false
        pickingAxis = false
        pendingSurface = nil
        pickingIK = !wasPicking
        refresh()
        revealIKControls()
    }

    private func pickedIKHelper(part: String, point: SIMD3<Float>) {
        guard pickingIK, let group = activeGroup, group.parts.contains(part) else {
            showError("Choose a point on the active child group")
            return
        }
        saveIKHelper(point: point, group: group)
    }

    private func saveIKHelper(point: SIMD3<Float>, group: RobotRigGroup) {
        let helper = RobotRigIKHelper(point: point, rootID: group.ikHelper?.rootID ?? defaultIKRoot(group))
        edit { draft in
            if let index = draft.groups.firstIndex(where: { $0.id == group.id }) { draft.groups[index].ikHelper = helper }
        }
    }

    @objc private func editIKHelper() {
        guard let group = activeGroup else { return }
        let point = group.ikHelper?.point ?? group.axis?.origin ?? .zero
        prompt(title: "IK Helper (Model Rest Coordinates)", fields: [("X", "\(point.x)"), ("Y", "\(point.y)"), ("Z", "\(point.z)")]) { [weak self] values in
            let numbers = values.compactMap(Float.init)
            guard numbers.count == 3, numbers.allSatisfy(\.isFinite) else { self?.showError("Enter three finite coordinates"); return }
            self?.saveIKHelper(point: SIMD3(numbers[0], numbers[1], numbers[2]), group: group)
        }
    }

    @objc private func chooseIKRoot() {
        guard let group = activeGroup, group.ikHelper != nil else { return }
        let menu = UIAlertController(title: "IK Chain Root", message: nil, preferredStyle: .actionSheet)
        var current: RobotRigGroup? = group
        while let joint = current {
            let rootID = joint.id
            menu.addAction(UIAlertAction(title: joint.name, style: .default) { [weak self] _ in
                self?.edit { draft in
                    if let index = draft.groups.firstIndex(where: { $0.id == group.id }) { draft.groups[index].ikHelper?.rootID = rootID }
                }
            })
            current = document.groups.first { $0.id == joint.parentID }
        }
        menu.addAction(UIAlertAction(title: "Cancel", style: .cancel))
        showSheet(menu)
    }

    @objc private func removeIKHelper() {
        guard let id = activeGroupID else { return }
        edit { draft in
            if let index = draft.groups.firstIndex(where: { $0.id == id }) { draft.groups[index].ikHelper = nil }
        }
    }

    @objc private func toggleIK() {
        guard !motors.isArmed, !importing, let group = activeGroup else { return }
        let wasActive = ikTarget != nil
        stopIK()
        movingAxis = false
        pickingAxis = false
        pendingSurface = nil
        if !wasActive {
            do {
                let chain = try document.ikChain(for: group.id)
                guard chain.contains(where: { $0.axis != nil }) else { throw RobotRigError.invalid("Set rotation axes on the IK chain first") }
                ikTarget = document.ikEndpoint(for: group.id, angles: angles)
            } catch { showError(error.localizedDescription) }
        }
        refresh()
        revealIKControls()
    }

    private func moveIK(_ target: SIMD3<Float>, state: UIGestureRecognizer.State) {
        guard ikTarget != nil, !motors.isArmed, !importing, let group = activeGroup else { return }
        if state == .cancelled || state == .failed {
            if let previous = ikDragAngles { angles = previous }
            ikDragAngles = nil
            ikTarget = document.ikEndpoint(for: group.id, angles: angles)
            ikMessage = nil
        } else {
            do {
                let result = try document.solveIK(for: group.id, target: target, angles: angles)
                angles = result.angles
                ikTarget = target
                ikMessage = String(format: "%@ / Distance %.4f", result.reached ? "Target reached" : "Target not reached", result.error)
                if state == .ended { ikDragAngles = nil }
            } catch { showError(error.localizedDescription) }
        }
        refresh()
    }

    @objc private func enterAngle() {
        guard activeGroup?.axis != nil, !motors.isArmed, !movingAxis, !pickingIK, ikTarget == nil else { return }
        prompt(title: "Preview Angle", fields: [("Degrees", "\(angle.value)")]) { [weak self] values in
            guard let self, let degrees = Float(values[0]), degrees.isFinite,
                  degrees >= self.angle.minimumValue, degrees <= self.angle.maximumValue else { self?.showError("Angle exceeds joint limits"); return }
            self.angle.value = degrees
            self.angleChanged()
        }
    }

    @objc private func resetPreview() {
        stopIK()
        viewport.cancelAxisDrag()
        movingAxis = false
        motors.disarm()
        angles.removeAll()
        pendingSurface = nil
        pickingAxis = false
        refresh()
    }

    @objc private func modeChanged() {
        guard mode.selectedSegmentIndex == 1 else { motors.disarm(); refresh(); return }
        mode.selectedSegmentIndex = 0
        guard let group = activeGroup, group.axis != nil, let binding = group.motor else { showError("Set an axis and bind a motor first"); return }
        confirm("Enable Live Control", message: "Confirm this model's motor ID, zero offset, direction, gearing and limits match the connected robot. Clear its travel and keep a physical stop accessible.") { [weak self] in
            guard let self else { return }
            do {
                try self.motors.arm(binding)
                self.pendingSurface = nil
                self.pickingAxis = false
                self.angle.value = Float(self.motors.measuredAngle(binding) ?? 0)
                self.motorUpdate()
            } catch { self.showError(error.localizedDescription) }
        }
    }

    @objc private func bindMotor() {
        guard let group = activeGroup, !motors.isArmed else { return }
        let menu = UIAlertController(title: "Motor Binding", message: nil, preferredStyle: .actionSheet)
        for motorMode in RobotRigMotorBinding.Mode.allCases {
            menu.addAction(UIAlertAction(title: motorMode == .multiTurn ? "Tracked Multi-Turn" : "Position (Preview Only)", style: .default) { [weak self] _ in self?.editBinding(group: group, motorMode: motorMode) })
        }
        menu.addAction(UIAlertAction(title: "Remove Binding", style: .destructive) { [weak self] _ in
            self?.edit { draft in if let index = draft.groups.firstIndex(where: { $0.id == group.id }) { draft.groups[index].motor = nil } }
        })
        menu.addAction(UIAlertAction(title: "Cancel", style: .cancel))
        showSheet(menu)
    }

    private func editBinding(group: RobotRigGroup, motorMode: RobotRigMotorBinding.Mode) {
        let binding = group.motor ?? RobotRigMotorBinding(servoID: Int(motors.servoIDs.first ?? 1), mode: motorMode, offset: motorMode == .position ? 180 : 0)
        prompt(title: "Motor Calibration", fields: [
            ("Servo ID", "\(binding.servoID)"), ("Direction (+1 or -1)", binding.reversed ? "-1" : "1"),
            ("Motor / joint ratio", "\(binding.ratio)"), ("Motor degrees at rest", "\(binding.offset)"),
            ("Joint minimum degrees", "\(binding.minimum)"), ("Joint maximum degrees", "\(binding.maximum)")
        ]) { [weak self] values in
            guard let id = Int(values[0]), let sign = Int(values[1]), [-1, 1].contains(sign),
                  let ratio = Double(values[2]), let offset = Double(values[3]), let lower = Double(values[4]), let upper = Double(values[5]) else {
                self?.showError("Invalid motor calibration values"); return
            }
            let updated = RobotRigMotorBinding(servoID: id, mode: motorMode, reversed: sign == -1, ratio: ratio, offset: offset, minimum: lower, maximum: upper, robotID: self?.motors.robotIdentity())
            self?.edit { draft in if let index = draft.groups.firstIndex(where: { $0.id == group.id }) { draft.groups[index].motor = updated } }
        }
    }

    @objc private func moveMotor() {
        guard let binding = activeGroup?.motor else { return }
        do { try motors.move(binding, to: Double(angle.value)); status.text = "Target sent" }
        catch { showError(error.localizedDescription) }
    }

    @objc private func stopMotor() { motors.disarm(); refresh() }

    private func motorUpdate() {
        if motors.isArmed {
            for group in document.groups {
                if let binding = group.motor, let measured = motors.measuredAngle(binding) { angles[group.id] = Float(measured) }
            }
        }
        if !importing, importError == nil, let message = motors.status { status.text = message }
        refresh()
    }

    @objc private func frameTapped() { viewport.cancelAxisDrag(); viewport.frameAll() }
    @objc private func isolateTapped() { isolated.toggle(); refresh() }
    @objc private func closeTapped() { importToken = UUID(); motors.setActive(false); PanelPresentation.close(self) }
    @objc private func undoTapped() { restoreHistory(undo: true) }
    @objc private func redoTapped() { restoreHistory(undo: false) }

    private func restoreHistory(undo: Bool) {
        guard !motors.isArmed, !importing, let url = assetURL, let restored = undo ? undoStack.last : redoStack.last else { return }
        stopIK()
        viewport.cancelAxisDrag()
        do {
            try restored.validate(partIDs: partIDs)
            try RobotModelStorage.save(restored, at: url)
            if undo { undoStack.removeLast(); redoStack.append(document) }
            else { redoStack.removeLast(); undoStack.append(document) }
            document = restored
            angles.removeAll()
            pendingSurface = nil
            pickingAxis = false
            refresh()
        } catch { showError(error.localizedDescription) }
    }

    private func prompt(title: String, fields: [(String, String)], action: @escaping ([String]) -> Void) {
        guard !motors.isArmed else { return }
        let alert = UIAlertController(title: title, message: nil, preferredStyle: .alert)
        for (label, value) in fields {
            alert.addTextField { field in field.placeholder = label; field.accessibilityLabel = label; field.text = value; field.autocorrectionType = .no }
        }
        alert.addAction(UIAlertAction(title: "Cancel", style: .cancel))
        alert.addAction(UIAlertAction(title: "Save", style: .default) { _ in action(alert.textFields?.map { $0.text ?? "" } ?? []) })
        present(alert, animated: true)
    }

    private func confirm(_ title: String, message: String, action: @escaping () -> Void) {
        let alert = UIAlertController(title: title, message: message, preferredStyle: .alert)
        alert.addAction(UIAlertAction(title: "Cancel", style: .cancel))
        alert.addAction(UIAlertAction(title: "Confirm", style: .default) { _ in action() })
        present(alert, animated: true)
    }

    private func showSheet(_ alert: UIAlertController) {
        alert.popoverPresentationController?.sourceView = groupButton
        alert.popoverPresentationController?.sourceRect = groupButton.bounds
        present(alert, animated: true)
    }

    private func showError(_ message: String) { status.text = message; status.textColor = .systemRed }

    private func showImportError(_ message: String) {
        importError = message
        showError(message)
        if presentedViewController == nil {
            let alert = UIAlertController(title: "Could Not Open Model", message: message, preferredStyle: .alert)
            alert.addAction(UIAlertAction(title: "OK", style: .default))
            present(alert, animated: true)
        }
    }

    func tableView(_ tableView: UITableView, numberOfRowsInSection section: Int) -> Int { filteredParts.count }
    func tableView(_ tableView: UITableView, cellForRowAt indexPath: IndexPath) -> UITableViewCell {
        let part = filteredParts[indexPath.row]
        let cell = tableView.dequeueReusableCell(withIdentifier: "part") ?? UITableViewCell(style: .subtitle, reuseIdentifier: "part")
        cell.textLabel?.text = part.name
        cell.textLabel?.font = .systemFont(ofSize: 14)
        cell.detailTextLabel?.text = document.groups.first { $0.parts.contains(part.id) }?.name ?? "Unassigned"
        cell.accessoryType = selected.contains(part.id) ? .checkmark : .none
        return cell
    }
    func tableView(_ tableView: UITableView, didSelectRowAt indexPath: IndexPath) {
        guard !motors.isArmed, !pickingAxis, !movingAxis, !pickingIK, ikTarget == nil else { return }
        let part = filteredParts[indexPath.row]
        if !selected.insert(part.id).inserted { selected.remove(part.id) }
        refresh()
    }
    func searchBar(_ searchBar: UISearchBar, textDidChange searchText: String) { table.reloadData() }

    #if DEBUG
    static func makeRuntimeChecks() -> RobotModelViewController {
        let commander = RobotRigCheckCommander()
        var connected = true
        var session = "test-session"
        let motors = RobotRigMotorController(commander: commander, connectionAvailable: { connected }, connectionIdentity: { session })
        let editor = RobotModelViewController(motors: motors)
        editor.runtimeCheck = { [weak editor] in
            guard let editor else { return }
            do {
                let vertices = [SCNVector3(0, 0, 0), SCNVector3(4, 0, 0), SCNVector3(0, 2, 0), SCNVector3(4, 2, 0),
                                SCNVector3(6, 0, 0), SCNVector3(7, 0, 0), SCNVector3(6, 1, 0)]
                let geometry = SCNGeometry(sources: [SCNGeometrySource(vertices: vertices)], elements: [
                    SCNGeometryElement(indices: [UInt32(0), 1, 2, 4, 5, 6], primitiveType: .triangles),
                    SCNGeometryElement(indices: [UInt32(1), 3, 2], primitiveType: .triangles)
                ])
                let teal = SCNMaterial()
                teal.diffuse.contents = UIColor.systemTeal
                teal.isDoubleSided = true
                let red = SCNMaterial()
                red.diffuse.contents = UIColor.systemRed
                red.isDoubleSided = true
                geometry.materials = [teal, red]
                let scene = SCNScene()
                let node = SCNNode(geometry: geometry)
                node.name = "Fixture"
                scene.rootNode.addChildNode(node)
                try editor.viewport.install(scene)
                precondition(editor.viewport.parts.count == 2)
                let part = editor.viewport.parts[0]
                precondition(part.node.geometry?.elements.count == 2)
                let surface = try part.mesh.surface(at: 0)
                precondition(surface.triangles.count == 2)
                precondition(simd_distance(surface.axis.origin, SIMD3(2, 1, 0)) < 0.0001)
                let folder = FileManager.default.temporaryDirectory.appendingPathComponent("RigChecks")
                try FileManager.default.createDirectory(at: folder, withIntermediateDirectories: true)
                let url = folder.appendingPathComponent("model.glb")
                editor.assetURL = url
                editor.document = RobotRigDocument(assetHash: "fixture", groups: [RobotRigGroup(name: "Shoulder", parts: [part.id], axis: surface.axis)])
                editor.activeGroupID = editor.document.groups[0].id
                editor.selected = [part.id]
                editor.pendingSurface = (part.id, surface)
                editor.refresh()
                editor.view.layoutIfNeeded()
                SCNTransaction.flush()
                _ = editor.viewport.snapshot()
                let projected = editor.viewport.projectPoint(SCNVector3(1, 0.5, 0))
                var pickedID: String?
                let originalPick = editor.viewport.onPick
                editor.viewport.onPick = { id, _ in pickedID = id }
                editor.viewport.pick(at: CGPoint(x: CGFloat(projected.x), y: CGFloat(projected.y)))
                editor.viewport.onPick = originalPick
                guard pickedID == part.id else {
                    throw RobotRigError.invalid("Triangle picking failed at \(projected); viewport \(editor.viewport.bounds)")
                }
                let priorDefault = UserDefaults.standard.string(forKey: "RobotModel.lastAsset")
                try RobotModelStorage.save(editor.document, at: url)
                let restored = try RobotModelStorage.loadDocument(at: url, hash: "fixture")
                precondition(restored == editor.document)
                editor.pendingSurface = nil
                editor.pickingAxis = false
                editor.document.groups[0].axis = nil
                editor.refresh()
                precondition(editor.actionButtons[#selector(confirmAxis)]?.isEnabled == false)
                editor.setAxis()
                precondition(editor.pickingAxis)
                precondition(editor.actionButtons[#selector(setAxis)]?.configuration?.title == "Cancel Face Selection")
                editor.picked(part: part.id, triangle: 0)
                precondition(editor.pendingSurface != nil)
                precondition(editor.actionButtons[#selector(confirmAxis)]?.isEnabled == true)
                editor.confirmAxis()
                precondition(!editor.pickingAxis && editor.pendingSurface == nil)
                precondition(editor.activeGroup?.axis == surface.axis)
                precondition(editor.angle.isEnabled)
                precondition(editor.actionButtons.values.allSatisfy { !($0.configuration?.title ?? "").isEmpty })
                let savedDocument = editor.document
                let savedGroupID = editor.activeGroupID
                editor.search.text = "no matching parts"
                editor.isolated = true
                editor.selectGroup(nil)
                precondition(editor.selected.isEmpty && !editor.isolated)
                precondition(editor.activeGroupID == nil)
                editor.selectGroup(savedGroupID)
                editor.actionButtons[#selector(deselectAllParts)]?.sendActions(for: .touchUpInside)
                precondition(editor.selected.isEmpty)
                precondition(editor.actionButtons[#selector(deselectAllParts)]?.isEnabled == false)
                editor.selectGroup(savedGroupID)
                precondition(editor.selected == [part.id])
                editor.setAxis()
                editor.picked(part: part.id, triangle: 0)
                editor.isolated = true
                let deselectCommand = editor.keyCommands!.first { $0.action == #selector(deselectAllParts) }!
                let deselectAction = deselectCommand.action!
                precondition(deselectCommand.input == "a" && deselectCommand.modifierFlags == [.command, .shift])
                precondition(editor.canPerformAction(deselectAction, withSender: deselectCommand))
                precondition(UIApplication.shared.sendAction(deselectAction, to: editor, from: deselectCommand, for: nil))
                precondition(editor.selected.isEmpty && editor.pendingSurface == nil && !editor.pickingAxis && !editor.isolated)
                precondition(editor.document == savedDocument && editor.activeGroupID == savedGroupID)
                precondition(!editor.canPerformAction(deselectAction, withSender: deselectCommand))
                editor.selectGroup(nil)
                precondition(editor.selected.isEmpty)
                editor.selectGroup(savedGroupID)
                editor.importing = true
                editor.deselectAllParts()
                precondition(editor.selected == [part.id])
                editor.importing = false
                editor.search.text = ""
                editor.selectGroup(savedGroupID)
                print("PASS: Unassigned Parts clears selection, Deselect All button and Command-Shift-A action, axis cancellation and document preservation")
                editor.angles[savedGroupID!] = 45
                editor.toggleMoveAxis()
                precondition(editor.movingAxis && editor.angles.isEmpty && !editor.angle.isEnabled)
                let startingAxis = editor.activeGroup!.axis!
                for (coordinate, name) in ["X", "Y", "Z"].enumerated() {
                    SCNTransaction.flush()
                    _ = editor.viewport.snapshot()
                    let handle = editor.viewport.scene!.rootNode.childNode(withName: name, recursively: true)!
                    let tip = handle.childNodes.first { $0.geometry is SCNCone }!
                    let startWorld = tip.simdWorldPosition
                    let distance = simd_length(startWorld - startingAxis.origin) * 0.3
                    var displacement = SIMD3<Float>.zero
                    displacement[coordinate] = distance
                    func screenPoint(_ world: SIMD3<Float>) -> CGPoint {
                        let screen = editor.viewport.projectPoint(SCNVector3(world))
                        return CGPoint(x: CGFloat(screen.x), y: CGFloat(screen.y))
                    }
                    let startPoint = screenPoint(startWorld)
                    let endPoint = screenPoint(startWorld + displacement)
                    let historyCount = editor.undoStack.count
                    precondition(editor.viewport.beginAxisDrag(at: startPoint), "Could not pick \(name) handle")
                    editor.viewport.moveAxisDrag(to: endPoint, state: .changed)
                    precondition(editor.activeGroup!.axis == startingAxis)
                    editor.viewport.moveAxisDrag(to: endPoint, state: .ended)
                    let movedAxis = editor.activeGroup!.axis!
                    precondition(simd_length(movedAxis.origin - startingAxis.origin - displacement) < distance * 0.01)
                    precondition(movedAxis.direction == startingAxis.direction)
                    precondition(editor.undoStack.count == historyCount + 1)
                    let persisted = try RobotModelStorage.loadDocument(at: url, hash: "fixture")
                    precondition(persisted == editor.document)
                    editor.undoTapped()
                    precondition(editor.activeGroup!.axis == startingAxis)
                    SCNTransaction.flush()
                    _ = editor.viewport.snapshot()
                    precondition(editor.viewport.beginAxisDrag(at: startPoint))
                    editor.viewport.moveAxisDrag(to: endPoint, state: .changed)
                    editor.viewport.cancelAxisDrag()
                    precondition(editor.activeGroup!.axis == startingAxis && editor.undoStack.count == historyCount)
                }
                SCNTransaction.flush()
                try editor.viewport.snapshot().pngData()?.write(to: folder.appendingPathComponent("gizmo.png"))
                editor.toggleMoveAxis()
                precondition(editor.angle.isEnabled && editor.document == savedDocument && commander.motion.isEmpty)
                print("PASS: XYZ cone picking, constrained pivot dragging, direction preservation, persistence, undo and cancellation")
                editor.angles[savedGroupID!] = 30
                editor.placeIKHelper()
                SCNTransaction.flush()
                _ = editor.viewport.snapshot()
                let helperRest = SIMD3<Float>(1, 0.5, 0)
                let helperWorld = editor.document.transforms(angles: editor.angles)[savedGroupID!]! * SIMD4(helperRest, 1)
                let helperScreen = editor.viewport.projectPoint(SCNVector3(helperWorld.x, helperWorld.y, helperWorld.z))
                editor.viewport.pick(at: CGPoint(x: CGFloat(helperScreen.x), y: CGFloat(helperScreen.y)))
                precondition(simd_distance(editor.activeGroup!.ikHelper!.point, helperRest) < 0.001)
                let helperDocument = try RobotModelStorage.loadDocument(at: url, hash: "fixture")
                precondition(helperDocument == editor.document && !editor.pickingIK)
                let root = RobotRigGroup(name: "IK Base", parts: [], axis: RobotRigAxis(origin: .zero, direction: SIMD3(0, 0, 1)))
                editor.edit { draft in
                    draft.groups.append(root)
                    draft.groups[0].parentID = root.id
                    draft.groups[0].axis = RobotRigAxis(origin: SIMD3(1, 0, 0), direction: SIMD3(0, 0, 1))
                    draft.groups[0].ikHelper = RobotRigIKHelper(point: SIMD3(2, 0, 0), rootID: root.id)
                }
                let ikDocument = editor.document
                editor.toggleIK()
                precondition(editor.ikTarget != nil && !editor.mode.isEnabled)
                editor.moveIK(SIMD3(0.5, 1.5, 0), state: .ended)
                precondition(simd_distance(editor.document.ikEndpoint(for: savedGroupID!, angles: editor.angles)!, SIMD3(0.5, 1.5, 0)) < 0.003)
                precondition(abs(editor.angles[root.id] ?? 0) > 1)
                precondition(editor.document == ikDocument && commander.motion.isEmpty)
                SCNTransaction.flush()
                _ = editor.viewport.snapshot()
                let targetScreen = editor.viewport.projectPoint(SCNVector3(editor.ikTarget!))
                let dragStart = CGPoint(x: CGFloat(targetScreen.x), y: CGFloat(targetScreen.y))
                let dragEnd = CGPoint(x: dragStart.x + 25, y: dragStart.y - 10)
                let poseBeforeDrag = editor.angles
                precondition(editor.viewport.beginAxisDrag(at: dragStart))
                editor.viewport.moveAxisDrag(to: dragEnd, state: .changed)
                precondition(editor.angles != poseBeforeDrag)
                editor.viewport.cancelAxisDrag()
                precondition(editor.angles == poseBeforeDrag)
                SCNTransaction.flush()
                _ = editor.viewport.snapshot()
                let childTransform = editor.document.transforms(angles: editor.angles)[savedGroupID!]!
                let childWorld = childTransform * SIMD4<Float>(0.6, 1.4, 0, 1)
                let childScreen = editor.viewport.projectPoint(SCNVector3(childWorld.x, childWorld.y, childWorld.z))
                let childStart = CGPoint(x: CGFloat(childScreen.x), y: CGFloat(childScreen.y))
                precondition(editor.viewport.beginAxisDrag(at: childStart), "Could not drag child mesh")
                editor.viewport.moveAxisDrag(to: CGPoint(x: childStart.x + 15, y: childStart.y - 8), state: .changed)
                precondition(editor.angles != poseBeforeDrag)
                editor.viewport.cancelAxisDrag()
                precondition(editor.angles == poseBeforeDrag)
                SCNTransaction.flush()
                _ = editor.viewport.snapshot()
                let ikHandle = editor.viewport.scene!.rootNode.childNode(withName: "X", recursively: true)!
                let ikCone = ikHandle.childNodes.first { $0.geometry is SCNCone }!
                let coneScreen = editor.viewport.projectPoint(SCNVector3(ikCone.simdWorldPosition))
                let coneEnd = editor.viewport.projectPoint(SCNVector3(ikCone.simdWorldPosition + SIMD3<Float>(0.1, 0, 0)))
                let targetBeforeHandle = editor.ikTarget!
                precondition(editor.viewport.beginAxisDrag(at: CGPoint(x: CGFloat(coneScreen.x), y: CGFloat(coneScreen.y))))
                editor.viewport.moveAxisDrag(to: CGPoint(x: CGFloat(coneEnd.x), y: CGFloat(coneEnd.y)), state: .ended)
                precondition(simd_distance(editor.ikTarget!, targetBeforeHandle + SIMD3(0.1, 0, 0)) < 0.001)
                editor.moveIK(SIMD3(10, 10, 10), state: .ended)
                precondition(editor.ikMessage!.contains("not reached") && commander.motion.isEmpty)
                editor.moveIK(SIMD3(1, 1, 0), state: .ended)
                editor.revealIKControls()
                SCNTransaction.flush()
                let ikImage = editor.viewport.snapshot()
                try ikImage.pngData()?.write(to: folder.appendingPathComponent("ik.png"))
                let ikEditorImage = UIGraphicsImageRenderer(bounds: editor.view.bounds).image { context in
                    editor.view.layer.render(in: context.cgContext)
                    ikImage.draw(in: editor.viewport.convert(editor.viewport.bounds, to: editor.view))
                    let badgeFrame = editor.viewportModeLabel.convert(editor.viewportModeLabel.bounds, to: editor.view)
                    context.cgContext.saveGState()
                    context.cgContext.translateBy(x: badgeFrame.minX, y: badgeFrame.minY)
                    editor.viewportModeLabel.layer.render(in: context.cgContext)
                    context.cgContext.restoreGState()
                }
                try ikEditorImage.pngData()?.write(to: folder.appendingPathComponent("ik-editor.png"))
                editor.resetPreview()
                precondition(editor.ikTarget == nil && editor.angles.isEmpty && editor.mode.isEnabled)
                editor.document = savedDocument
                try RobotModelStorage.save(savedDocument, at: url)
                editor.refresh()
                print("PASS: IK rest-coordinate helper picking, persistence, parent-chain solving, target drag cancellation, unreachable state and zero motor commands")
                UserDefaults.standard.set(priorDefault, forKey: "RobotModel.lastAsset")
                let binding = RobotRigMotorBinding(servoID: 1, reversed: true, ratio: 2, offset: 720, robotID: "test")
                motors.updateIDs([1])
                func feed(valid: Bool = true) {
                    motors.receive(ServoState(id: 1, error: 0, position: 2048, load: 0, voltage: 120, temperature: 25, torqueEnabled: valid))
                    motors.receive(ServoAxisStatus(id: 1, isTracked: true, hasMin: true, hasMax: true, hasZero: valid,
                                                  isMoving: false, isError: false, cumulativeTicks: 8192, angleDegrees: 720, percent: 50, totalDegrees: 720))
                }
                feed()
                do { try motors.move(binding, to: 30); preconditionFailure("Preview sent motion") } catch {}
                precondition(commander.motion.isEmpty)
                try motors.arm(binding)
                try motors.move(binding, to: 30)
                precondition(commander.motion == [660])
                do { try motors.move(binding, to: 40); preconditionFailure("Duplicate pending motion accepted") } catch {}
                motors.disarm()
                precondition(commander.stops == 1)
                var wrongRobot = binding
                wrongRobot.robotID = "other-robot"
                do { try motors.arm(wrongRobot); preconditionFailure("Wrong robot accepted") } catch {}
                try motors.arm(binding)
                session = "replacement-session"
                do { try motors.move(binding, to: 30); preconditionFailure("Changed session accepted") } catch {}
                motors.connectionChanged()
                feed(valid: false)
                do { try motors.arm(binding); preconditionFailure("Invalid telemetry armed") } catch {}
                feed()
                try motors.arm(binding)
                connected = false
                do { try motors.move(binding, to: 30); preconditionFailure("Disconnected motion accepted") } catch {}
                motors.connectionChanged()
                precondition(!motors.isArmed)
                connected = true
                motors.setActive(false)
                precondition(commander.motion.count == 1)
                print("PASS: actual SceneKit components, multi-material face picking, centroid, persistence, preview isolation, motor conversion, pending-move guard, invalid telemetry and disconnect")
                var report = "PASS: SceneKit picking, centroid, persistence and guarded motor commands\nPASS: wrong robot and changed session rejected\nPASS: visible action labels, face selection, confirmation and preview enablement\n"
                let args = ProcessInfo.processInfo.arguments
                if let index = args.firstIndex(of: "--rig-model"), args.indices.contains(index + 1) {
                    let asset = try GLTFAsset(url: URL(fileURLWithPath: args[index + 1]), options: [:])
                    guard let sample = GLTFSCNSceneSource(asset: asset).defaultScene else { throw RobotRigError.invalid("Sample scene missing") }
                    do {
                        try RobotModelStorage.validateGLB(Data(contentsOf: URL(fileURLWithPath: args[index + 1])))
                        let importedViewport = RobotModelSceneView()
                        try importedViewport.install(sample)
                        editor.viewport.removeFromSuperview()
                        editor.viewport = importedViewport
                        editor.viewportHost.addSubview(importedViewport)
                        editor.attachViewport()
                        editor.view.layoutIfNeeded()
                        editor.document = RobotRigDocument(assetHash: "sample")
                        editor.activeGroupID = nil
                        editor.assetURL = nil
                        editor.pendingSurface = nil
                        editor.selected.removeAll()
                        editor.refresh()
                        SCNTransaction.flush()
                        print("PASS: sample GLB decoded into \(editor.viewport.parts.count) parts")
                        report += "PASS: sample GLB decoded into \(editor.viewport.parts.count) parts\n"
                        report += "Viewport bounds: \(editor.viewport.bounds)\n"
                    } catch {
                        print("SAMPLE LIMITATION: \(error.localizedDescription)")
                        throw error
                    }
                }
                let image = editor.viewport.snapshot()
                if let data = image.cgImage?.dataProvider?.data {
                    let bytes = CFDataGetBytePtr(data)!
                    let count = CFDataGetLength(data)
                    let unique = Set(stride(from: 0, to: count, by: 97).map { bytes[$0] })
                    precondition(unique.count > 12, "Viewport rendered blank")
                    report += "PASS: nonblank viewport pixel check\n"
                }
                try image.pngData()?.write(to: folder.appendingPathComponent("viewport.png"))
                let editorImage = UIGraphicsImageRenderer(bounds: editor.view.bounds).image { context in
                    editor.view.layer.render(in: context.cgContext)
                    image.draw(in: editor.viewport.convert(editor.viewport.bounds, to: editor.view))
                    let badgeFrame = editor.viewportModeLabel.convert(editor.viewportModeLabel.bounds, to: editor.view)
                    context.cgContext.saveGState()
                    context.cgContext.translateBy(x: badgeFrame.minX, y: badgeFrame.minY)
                    editor.viewportModeLabel.layer.render(in: context.cgContext)
                    context.cgContext.restoreGState()
                }
                try editorImage.pngData()?.write(to: folder.appendingPathComponent("editor.png"))
                try report.write(to: folder.appendingPathComponent("results.txt"), atomically: true, encoding: .utf8)
                print("RIG_CHECKS_COMPLETE \(folder.path)")
                editor.status.text = "Runtime checks passed"
                editor.status.textColor = .systemGreen
                if ProcessInfo.processInfo.arguments.contains("--rig-exit") { exit(0) }
            } catch {
                editor.showError(error.localizedDescription)
                print("RIG_CHECKS_FAILED: \(error)")
                let folder = FileManager.default.temporaryDirectory.appendingPathComponent("RigChecks")
                try? FileManager.default.createDirectory(at: folder, withIntermediateDirectories: true)
                try? "FAIL: \(error.localizedDescription)".write(to: folder.appendingPathComponent("results.txt"), atomically: true, encoding: .utf8)
                if ProcessInfo.processInfo.arguments.contains("--rig-exit") { exit(1) }
            }
        }
        return editor
    }
    #endif
}

#if DEBUG
private final class RobotRigCheckCommander: ServoCommanding {
    var motion: [Double] = []
    var stops = 0
    func rescanServos(from: UInt8, to: UInt8) {}
    func moveServo(id: UInt8, position: UInt16, speed: UInt16) { preconditionFailure("Unexpected position command") }
    func moveDiscoveredServos(position: UInt16, speed: UInt16, acceleration: UInt8) { preconditionFailure("Unexpected broadcast") }
    func setServoTorque(id: UInt8, enabled: Bool) { preconditionFailure("Unexpected torque command") }
    func changeServoID(currentID: UInt8, newID: UInt8) { preconditionFailure("Unexpected ID command") }
    func calibrateServoCenter(id: UInt8) { preconditionFailure("Unexpected calibration") }
    func driveServoWheel(id: UInt8, speed: Int16, acceleration: UInt8) { preconditionFailure("Unexpected wheel command") }
    func setServoPositionMode(id: UInt8) { preconditionFailure("Unexpected mode command") }
    func refreshServoState(id: UInt8) {}
    func trackServoAxis(id: UInt8) { preconditionFailure("Unexpected tracking command") }
    func jogServo(id: UInt8, degrees: Double) { preconditionFailure("Unexpected jog") }
    func beginServoJog(id: UInt8, direction: ServoJogDirection, speed: UInt16) { preconditionFailure("Unexpected jog") }
    func stopServoMotion(id: UInt8) { stops += 1 }
    func markServoTravel(id: UInt8, _ mark: ServoTravelMark) { preconditionFailure("Unexpected mark") }
    func moveServoToPercent(id: UInt8, percent: Double, speed: UInt16) { preconditionFailure("Unexpected percent move") }
    func moveServoToAngle(id: UInt8, degrees: Double) { motion.append(degrees) }
    func refreshServoAxisStatus(id: UInt8) {}
}
#endif