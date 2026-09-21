import UIKit
import UniformTypeIdentifiers
import SceneKit
import GLTFKit2
import RobotCollisionQueries

final class RobotModelViewController: PanelViewController, UIDocumentPickerDelegate, UITableViewDataSource, UITableViewDelegate, UITableViewDragDelegate, UITableViewDropDelegate, UISearchBarDelegate {
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
    private let errorBanner = UIStackView()
    private let errorLabel = UILabel()
    private var errorTimer: Timer?
    private var errorDeadline: Date?
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
    private var collisionPreparation: Task<RigCollisionWorld, Never>?
    private var collisionCheck: Task<RigCollisionWorld.Outcome, Never>?
    private var pendingIKMove: (target: SIMD3<Float>, state: UIGestureRecognizer.State)?
    private var ikDisplayLink: CADisplayLink?
    private final class IKFrameTarget {
        weak var editor: RobotModelViewController?
        init(_ editor: RobotModelViewController) { self.editor = editor }
        @objc func tick() { editor?.processIKFrame() }
    }
    private var collisionGeneration = UUID()
    private var collisionPartIDs = Set<String>()
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
    private struct TreeNode {
        let id: String
        let name: String
        let parts: Set<String>
        var groupID: UUID? = nil
        var children: [TreeNode] = []
    }
    private struct TreeRow {
        let node: TreeNode
        let depth: Int
    }
    private struct TreeDrag {
        let groupID: UUID?
        let parts: Set<String>
    }
    private struct TreeState: Equatable {
        let viewportID: ObjectIdentifier
        let document: RobotRigDocument
        let query: String
        let collapsed: Set<String>
        let selected: Set<String>
        let activeGroupID: UUID?
        let editable: Bool
    }
    private var displayedTreeState: TreeState?
    private var treeRows: [TreeRow] = []
    private var collapsedNodes = Set<String>()
    private var sidebarSections: [String: (UIButton, UIStackView)] = [:]
    #if DEBUG
    private var runtimeCheck: (() async -> Void)?
    #endif

    init(motors: RobotRigMotorController) {
        self.motors = motors
        super.init(nibName: nil, bundle: nil)
    }

    required init?(coder: NSCoder) { fatalError("init(coder:) has not been implemented") }

    isolated deinit { ikDisplayLink?.invalidate() }

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
        errorBanner.translatesAutoresizingMaskIntoConstraints = false
        errorBanner.axis = .horizontal
        errorBanner.alignment = .center
        errorBanner.isLayoutMarginsRelativeArrangement = true
        errorBanner.layoutMargins = UIEdgeInsets(top: 12, left: 12, bottom: 12, right: 12)
        errorBanner.backgroundColor = .clear
        errorBanner.isHidden = true
        errorBanner.addGestureRecognizer(UITapGestureRecognizer(target: self, action: #selector(hideError)))
        errorBanner.isAccessibilityElement = true
        errorBanner.accessibilityCustomActions = [UIAccessibilityCustomAction(name: "Dismiss error", target: self, selector: #selector(dismissAccessibleError))]
        errorLabel.font = .systemFont(ofSize: 14, weight: .medium)
        errorLabel.textColor = .systemRed
        errorLabel.numberOfLines = 0
        errorBanner.addArrangedSubview(errorLabel)
        viewportHost.addSubview(errorBanner)
        NSLayoutConstraint.activate([
            errorBanner.bottomAnchor.constraint(equalTo: viewportHost.safeAreaLayoutGuide.bottomAnchor, constant: -8),
            errorBanner.centerXAnchor.constraint(equalTo: viewportHost.centerXAnchor),
            errorBanner.heightAnchor.constraint(greaterThanOrEqualToConstant: 44),
            errorBanner.widthAnchor.constraint(equalTo: viewportHost.widthAnchor, constant: -24)
        ])
        search.placeholder = "Groups, collections, parts"
        search.delegate = self
        search.searchBarStyle = .minimal
        table.dataSource = self
        table.delegate = self
        table.dragDelegate = self
        table.dropDelegate = self
        table.dragInteractionEnabled = true
        table.accessibilityLabel = "Model hierarchy"
        table.rowHeight = 44
        table.estimatedRowHeight = 0
        table.estimatedSectionHeaderHeight = 0
        table.estimatedSectionFooterHeight = 0
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
        addSidebarSection("Hierarchy", views: [search, table, selectionLabel, selectionTools])
        addSidebarSection("Parts & Groups", views: [groupButton, groupTools])
        addSidebarSection("Rotation Axis", views: [axisStateLabel, axisTools])
        addSidebarSection("Inverse Kinematics", views: [ikStateLabel, ikTools])
        addSidebarSection("Rotation & Motor", views: [mode, angleLabel, angle, motorTools])
        groupButton.heightAnchor.constraint(equalToConstant: 44).isActive = true
        search.heightAnchor.constraint(equalToConstant: 44).isActive = true
        table.heightAnchor.constraint(equalToConstant: 320).isActive = true
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
            DispatchQueue.main.asyncAfter(deadline: .now() + 0.5) { Task { await check() } }
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
    private var canEditTree: Bool {
        assetURL != nil && !motors.isArmed && !importing && !pickingAxis && !movingAxis && !pickingIK && ikTarget == nil
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

    private func addSidebarSection(_ title: String, views: [UIView]) {
        let content = UIStackView(arrangedSubviews: views)
        content.axis = .vertical
        content.spacing = 8
        let header = UIButton(type: .system)
        var configuration = UIButton.Configuration.plain()
        configuration.title = title
        configuration.image = UIImage(systemName: "chevron.down")
        configuration.imagePadding = 8
        configuration.contentInsets = NSDirectionalEdgeInsets(top: 8, leading: 0, bottom: 8, trailing: 0)
        header.configuration = configuration
        header.contentHorizontalAlignment = .leading
        header.heightAnchor.constraint(equalToConstant: 44).isActive = true
        header.addAction(UIAction { [weak self, weak content] _ in
            guard let content else { return }
            self?.setSidebarSection(title, collapsed: !content.isHidden)
        }, for: .touchUpInside)
        sidebarSections[title] = (header, content)
        inspector.addArrangedSubview(header)
        inspector.addArrangedSubview(content)
        setSidebarSection(title, collapsed: false)
    }

    private func setSidebarSection(_ title: String, collapsed: Bool) {
        guard let (header, content) = sidebarSections[title] else { return }
        content.isHidden = collapsed
        header.configuration?.image = UIImage(systemName: collapsed ? "chevron.right" : "chevron.down")
        header.accessibilityValue = collapsed ? "Collapsed" : "Expanded"
        header.toolTip = "\(collapsed ? "Expand" : "Collapse") \(title)"
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
        groupButton.configuration?.title = activeGroup.map { "Group: \($0.name)" } ?? "No Active Group"
        groupButton.accessibilityLabel = groupButton.configuration?.title
        groupButton.menu = UIMenu(children: [UIAction(title: "No Active Group", state: activeGroupID == nil ? .on : .off) { [weak self] _ in self?.selectGroup(nil) }] + document.groups.map { group in
            UIAction(title: group.name, state: group.id == activeGroupID ? .on : .off) { [weak self] _ in self?.selectGroup(group.id) }
        })
        viewport.update(document: document, angles: angles, selected: selected, isolated: isolated, colliding: collisionPartIDs)
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
        viewportModeLabel.isHidden = !motors.isArmed
        viewportHost.bringSubviewToFront(viewportModeLabel)
        viewportHost.bringSubviewToFront(errorBanner)
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
        reloadTree()
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
                        self.resetTreeExpansion()
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
                self.setSidebarSection("Rotation Axis", collapsed: false)
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
        status.text = pickingAxis ? "Select axis face" : nil
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
        collisionGeneration = UUID()
        collisionPreparation?.cancel()
        collisionPreparation = nil
        collisionCheck?.cancel()
        collisionCheck = nil
        pendingIKMove = nil
        ikDisplayLink?.invalidate()
        ikDisplayLink = nil
        collisionPartIDs.removeAll()
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
                var moving = Set(chain.filter { $0.axis != nil }.map { $0.id.uuidString })
                for _ in document.groups.indices {
                    for item in document.groups where item.parentID.map({ moving.contains($0.uuidString) }) == true {
                        moving.insert(item.id.uuidString)
                    }
                }
                let parts = viewport.parts.map { part in
                    RigCollisionWorld.Part(id: part.id, name: part.name,
                                           owner: document.groups.first { $0.parts.contains(part.id) }?.id.uuidString,
                                           vertices: part.mesh.triangles.flatMap { [$0.first, $0.second, $0.third] })
                }
                let joints = document.groups.compactMap { item in
                    item.axis.map { RigCollisionWorld.Joint(id: item.id.uuidString, parent: item.parentID?.uuidString, pivot: $0.origin, direction: $0.direction) }
                }
                let movingIDs = moving
                collisionPreparation = Task.detached(priority: .userInitiated) {
                    RigCollisionWorld(parts: parts, joints: joints, moving: movingIDs)
                }
                ikTarget = document.ikEndpoint(for: group.id, angles: angles)
            } catch { showError(error.localizedDescription) }
        }
        refresh()
        revealIKControls()
    }

    private func moveIK(_ target: SIMD3<Float>, state: UIGestureRecognizer.State) {
        guard ikTarget != nil, !motors.isArmed, !importing else { return }
        if state == .cancelled || state == .failed {
            ikDisplayLink?.invalidate()
            ikDisplayLink = nil
            performIKMove(target, state: state)
            return
        }
        ikTarget = target
        pendingIKMove = (target, state)
        if ikDisplayLink == nil {
            let link = CADisplayLink(target: IKFrameTarget(self), selector: #selector(IKFrameTarget.tick))
            link.preferredFrameRateRange = CAFrameRateRange(minimum: 30, maximum: 60, preferred: 60)
            link.add(to: .main, forMode: .common)
            ikDisplayLink = link
        }
        ikDisplayLink?.isPaused = false
    }

    private func processIKFrame() {
        guard collisionCheck == nil, let pending = pendingIKMove else { return }
        pendingIKMove = nil
        ikDisplayLink?.isPaused = true
        performIKMove(pending.target, state: pending.state)
    }

    private func performIKMove(_ target: SIMD3<Float>, state: UIGestureRecognizer.State) {
        guard ikTarget != nil, !motors.isArmed, !importing, let group = activeGroup else { return }
        if state == .cancelled || state == .failed {
            collisionGeneration = UUID()
            collisionCheck?.cancel()
            collisionCheck = nil
            pendingIKMove = nil
            collisionPartIDs.removeAll()
            if let previous = ikDragAngles { angles = previous }
            ikDragAngles = nil
            ikTarget = document.ikEndpoint(for: group.id, angles: angles)
            ikMessage = nil
        } else {
            ikTarget = target
            if collisionCheck != nil {
                pendingIKMove = (target, state)
                return
            }
            do {
                let result = try document.solveIK(for: group.id, target: target, angles: angles)
                guard let preparation = collisionPreparation else { return }
                let generation = UUID()
                collisionGeneration = generation
                let initial = angles
                let ids = Set(initial.keys).union(result.angles.keys)
                let travel = ids.reduce(Float(0)) { $0 + abs((result.angles[$1] ?? 0) - (initial[$1] ?? 0)) }
                let stepCount = min(8, max(1, Int(ceil(travel / 5))))
                let samples = (0...stepCount).map { step in
                    Dictionary(uniqueKeysWithValues: ids.map { id in
                        (id, (initial[id] ?? 0) + ((result.angles[id] ?? 0) - (initial[id] ?? 0)) * Float(step) / Float(stepCount))
                    })
                }
                let poses = samples.map { sample in
                    Dictionary(uniqueKeysWithValues: document.transforms(angles: sample).map { ($0.key.uuidString, $0.value) })
                }
                ikMessage = "Checking mesh collisions"
                let worker = Task.detached(priority: .userInitiated) {
                    let world = await preparation.value
                    return world.check(poses: poses, measureOverlap: false)
                }
                collisionCheck = worker
                Task { [weak self] in
                    let outcome = await worker.value
                    guard let self, self.collisionGeneration == generation, self.ikTarget != nil,
                          !self.motors.isArmed, !self.importing else { return }
                                        self.collisionCheck = nil
                    self.angles = samples[outcome.accepted]
                    self.collisionPartIDs = Set(outcome.blockedParts)
                    let reached = result.reached && outcome.accepted == stepCount
                    let endpoint = self.document.ikEndpoint(for: group.id, angles: self.angles) ?? result.endpoint
                    self.ikMessage = String(format: "%@ / Distance %.4f", reached ? "Target reached" : "Target not reached", simd_distance(endpoint, target))
                    if let message = Self.collisionError(outcome, proposedSamples: samples.count - 1) {
                        self.ikMessage = (self.ikMessage ?? "") + " / " + message
                        self.showError(message)
                    }
                    self.refresh()
                    if self.pendingIKMove != nil {
                        self.ikDisplayLink?.isPaused = false
                        return
                    }
                    if state == .ended { self.ikDragAngles = nil }
                }
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
        if !importing, importError == nil, let message = motors.status { status.text = message == "Preview" ? nil : message }
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

    private static func collisionError(_ outcome: RigCollisionWorld.Outcome, proposedSamples: Int) -> String? {
        outcome.accepted < proposedSamples || !outcome.blockedParts.isEmpty ? outcome.message : nil
    }

    private func showError(_ message: String) {
        let announce = errorBanner.isHidden || errorLabel.text != message
        errorTimer?.invalidate()
        errorLabel.text = message
        errorBanner.accessibilityLabel = message
        errorBanner.isHidden = false
        viewportHost.bringSubviewToFront(errorBanner)
        errorDeadline = Date().addingTimeInterval(20)
        errorTimer = Timer.scheduledTimer(withTimeInterval: 20, repeats: false) { [weak self] _ in
            MainActor.assumeIsolated { self?.expireError(at: Date()) }
        }
        if announce { UIAccessibility.post(notification: .announcement, argument: message) }
    }

    private func expireError(at date: Date) {
        guard let deadline = errorDeadline, date >= deadline else { return }
        hideError()
    }

    @objc private func hideError() {
        errorTimer?.invalidate()
        errorTimer = nil
        errorDeadline = nil
        errorBanner.isHidden = true
    }

    @objc private func dismissAccessibleError() -> Bool {
        hideError()
        return true
    }

    private func showImportError(_ message: String) {
        importError = message
        showError(message)
        if presentedViewController == nil {
            let alert = UIAlertController(title: "Could Not Open Model", message: message, preferredStyle: .alert)
            alert.addAction(UIAlertAction(title: "OK", style: .default))
            present(alert, animated: true)
        }
    }

    private func resetTreeExpansion() {
        displayedTreeState = nil
        collapsedNodes.removeAll()
        func collapse(_ node: RobotModelSceneView.ModelNode) {
            collapsedNodes.insert("model/\(node.id)")
            node.children.forEach(collapse)
        }
        viewport.modelHierarchy?.children.forEach(collapse)
    }

    private func reloadTree() {
        let state = TreeState(viewportID: ObjectIdentifier(viewport), document: document,
                              query: search.text ?? "", collapsed: collapsedNodes, selected: selected,
                              activeGroupID: activeGroupID, editable: canEditTree)
        guard state != displayedTreeState else { return }
        displayedTreeState = state
        let parts = Dictionary(uniqueKeysWithValues: viewport.parts.map { ($0.id, $0.name) })
        func leaves(_ ids: Set<String>, prefix: String) -> [TreeNode] {
            ids.sorted().map { TreeNode(id: "\(prefix)/\($0)", name: parts[$0] ?? $0, parts: [$0]) }
        }
        func modelNode(_ node: RobotModelSceneView.ModelNode) -> TreeNode {
            TreeNode(id: "model/\(node.id)", name: node.name, parts: node.allParts,
                     children: node.children.map(modelNode) + leaves(node.parts, prefix: "source"))
        }
        func groupNode(_ group: RobotRigGroup) -> TreeNode {
            let children = document.groups.filter { $0.parentID == group.id }.map(groupNode)
            return TreeNode(id: "group/\(group.id)", name: group.name,
                            parts: children.reduce(into: group.parts) { $0.formUnion($1.parts) }, groupID: group.id,
                            children: children + leaves(group.parts, prefix: "rig"))
        }
        let assigned = document.groups.reduce(into: Set<String>()) { $0.formUnion($1.parts) }
        let groups = TreeNode(id: "groups", name: "Rig Groups", parts: assigned, children:
            document.groups.filter { $0.parentID == nil }.map(groupNode))
        let roots = [groups] + (viewport.modelHierarchy.map { [modelNode($0)] } ?? [])
        let query = (search.text ?? "").trimmingCharacters(in: .whitespacesAndNewlines)
        func matches(_ node: TreeNode) -> Bool {
            node.name.localizedCaseInsensitiveContains(query) || node.children.contains(where: matches)
        }
        treeRows.removeAll(keepingCapacity: true)
        func append(_ node: TreeNode, depth: Int, ancestorMatches: Bool) {
            let ownMatch = !query.isEmpty && node.name.localizedCaseInsensitiveContains(query)
            guard query.isEmpty || ancestorMatches || matches(node) else { return }
            treeRows.append(TreeRow(node: node, depth: depth))
            if !query.isEmpty || !collapsedNodes.contains(node.id) {
                for child in node.children { append(child, depth: depth + 1, ancestorMatches: ancestorMatches || ownMatch) }
            }
        }
        for root in roots { append(root, depth: 0, ancestorMatches: false) }
        table.reloadData()
    }

    private func toggleTreeNode(_ id: String) {
        if !collapsedNodes.insert(id).inserted { collapsedNodes.remove(id) }
        reloadTree()
    }

    private func selectTreeNode(_ node: TreeNode) {
        guard canEditTree else { return }
        if let groupID = node.groupID { activeGroupID = groupID }
        if !node.parts.isEmpty && node.parts.isSubset(of: selected) { selected.subtract(node.parts) }
        else { selected.formUnion(node.parts) }
        refresh()
    }

    func tableView(_ tableView: UITableView, numberOfRowsInSection section: Int) -> Int { treeRows.count }

    func tableView(_ tableView: UITableView, cellForRowAt indexPath: IndexPath) -> UITableViewCell {
        let row = treeRows[indexPath.row]
        let node = row.node
        let cell = tableView.dequeueReusableCell(withIdentifier: "tree") ?? UITableViewCell(style: .subtitle, reuseIdentifier: "tree")
        cell.textLabel?.text = node.name
        cell.textLabel?.font = .systemFont(ofSize: 14, weight: node.groupID == activeGroupID && node.groupID != nil ? .semibold : .regular)
        cell.textLabel?.lineBreakMode = .byTruncatingMiddle
        cell.indentationLevel = min(row.depth, 6)
        cell.indentationWidth = 12
        let selectedCount = node.parts.intersection(selected).count
        cell.detailTextLabel?.text = node.children.isEmpty && node.parts.count == 1
            ? (document.groups.first { $0.parts.contains(node.parts.first!) }?.name ?? "Unassigned")
            : "\(node.parts.count) parts" + (selectedCount > 0 ? " / \(selectedCount) selected" : "")
        cell.imageView?.image = UIImage(systemName: node.children.isEmpty ? "cube" : "folder")
        let controls = UIStackView()
        controls.spacing = 0
        let selection = UIButton(type: .system)
        let symbol = selectedCount == 0 ? "square" : (selectedCount == node.parts.count ? "checkmark.square.fill" : "minus.square.fill")
        selection.setImage(UIImage(systemName: symbol), for: .normal)
        selection.accessibilityLabel = "Select \(node.name)"
        selection.accessibilityValue = selectedCount == 0 ? "Not selected" : (selectedCount == node.parts.count ? "Selected" : "Partially selected")
        selection.toolTip = "Toggle selection: \(node.name)"
        selection.isEnabled = canEditTree && !node.parts.isEmpty
        selection.addAction(UIAction { [weak self] _ in self?.selectTreeNode(node) }, for: .touchUpInside)
        controls.addArrangedSubview(selection)
        selection.widthAnchor.constraint(equalToConstant: 36).isActive = true
        if !node.children.isEmpty {
            let disclosure = UIButton(type: .system)
            let expanded = !(search.text ?? "").isEmpty || !collapsedNodes.contains(node.id)
            disclosure.setImage(UIImage(systemName: expanded ? "chevron.down" : "chevron.right"), for: .normal)
            disclosure.accessibilityLabel = "\(expanded ? "Collapse" : "Expand") \(node.name)"
            disclosure.toolTip = disclosure.accessibilityLabel
            disclosure.isEnabled = (search.text ?? "").trimmingCharacters(in: .whitespacesAndNewlines).isEmpty
            disclosure.addAction(UIAction { [weak self] _ in self?.toggleTreeNode(node.id) }, for: .touchUpInside)
            controls.addArrangedSubview(disclosure)
            disclosure.widthAnchor.constraint(equalToConstant: 36).isActive = true
        }
        controls.frame = CGRect(x: 0, y: 0, width: node.children.isEmpty ? 36 : 72, height: 44)
        cell.accessoryView = controls
        cell.accessibilityValue = selection.accessibilityValue
        return cell
    }

    func tableView(_ tableView: UITableView, didSelectRowAt indexPath: IndexPath) {
        tableView.deselectRow(at: indexPath, animated: false)
        selectTreeNode(treeRows[indexPath.row].node)
    }

    func searchBar(_ searchBar: UISearchBar, textDidChange searchText: String) { reloadTree() }

    func tableView(_ tableView: UITableView, itemsForBeginning session: UIDragSession, at indexPath: IndexPath) -> [UIDragItem] {
        guard canEditTree else { return [] }
        let node = treeRows[indexPath.row].node
        guard node.id != "groups", node.groupID != nil || !node.parts.isEmpty else { return [] }
        let chosen = node.parts.isSubset(of: selected) ? selected : node.parts
        let item = UIDragItem(itemProvider: NSItemProvider(object: node.name as NSString))
        item.localObject = TreeDrag(groupID: node.groupID, parts: chosen)
        session.localContext = self
        return [item]
    }

    private func dropping(_ items: [UIDragItem], onto destination: TreeNode) throws -> RobotRigDocument {
        guard canEditTree, destination.groupID != nil || destination.id == "groups" else {
            throw RobotRigError.invalid("Drop onto a rig group")
        }
        var draft = document
        for item in items {
            guard let payload = item.localObject as? TreeDrag else { throw RobotRigError.invalid("Only local model items can be moved") }
            if let id = payload.groupID {
                guard let index = draft.groups.firstIndex(where: { $0.id == id }) else {
                    throw RobotRigError.invalid("Drop groups onto another group or Rig Groups")
                }
                draft.groups[index].parentID = destination.groupID
            } else {
                guard destination.id != "groups" else { throw RobotRigError.invalid("Choose a group for these parts") }
                for index in draft.groups.indices {
                    if draft.groups[index].id == destination.groupID { draft.groups[index].parts.formUnion(payload.parts) }
                    else { draft.groups[index].parts.subtract(payload.parts) }
                }
            }
        }
        try draft.validate(partIDs: partIDs)
        return draft
    }

    func tableView(_ tableView: UITableView, canHandle session: UIDropSession) -> Bool {
        (session.localDragSession?.localContext as? RobotModelViewController) === self && canEditTree
    }

    func tableView(_ tableView: UITableView, dropSessionDidUpdate session: UIDropSession, withDestinationIndexPath destinationIndexPath: IndexPath?) -> UITableViewDropProposal {
        guard self.tableView(tableView, canHandle: session), let destinationIndexPath,
              treeRows.indices.contains(destinationIndexPath.row),
              (try? dropping(session.items, onto: treeRows[destinationIndexPath.row].node)) != nil else {
            return UITableViewDropProposal(operation: .forbidden)
        }
        return UITableViewDropProposal(operation: .move, intent: .insertIntoDestinationIndexPath)
    }

    func tableView(_ tableView: UITableView, performDropWith coordinator: UITableViewDropCoordinator) {
        guard self.tableView(tableView, canHandle: coordinator.session), let indexPath = coordinator.destinationIndexPath,
              treeRows.indices.contains(indexPath.row) else { return }
        let destination = treeRows[indexPath.row].node
        do {
            let draft = try dropping(coordinator.items.map(\.dragItem), onto: destination)
            guard draft != document else { return }
            for item in coordinator.items { coordinator.drop(item.dragItem, toRowAt: indexPath) }
            collapsedNodes.remove(destination.id)
            edit { $0 = draft }
        } catch { showError(error.localizedDescription) }
    }

    #if DEBUG
    static func makeRuntimeChecks() -> RobotModelViewController {
        let commander = RobotRigCheckCommander()
        var connected = true
        var session = "test-session"
        let motors = RobotRigMotorController(commander: commander, connectionAvailable: { connected }, connectionIdentity: { session })
        let editor = RobotModelViewController(motors: motors)
        editor.runtimeCheck = { [weak editor] in
            guard let editor else { return }
            func awaitCollision() async {
                for _ in 0..<200 {
                    editor.processIKFrame()
                    let worker = editor.collisionCheck
                    _ = await worker?.value
                    await Task.yield()
                    if editor.collisionCheck == nil && editor.pendingIKMove == nil { return }
                }
                preconditionFailure("Collision completion was not applied")
            }
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
                let collection = SCNNode()
                collection.name = "Assembly"
                collection.addChildNode(node)
                scene.rootNode.addChildNode(collection)
                try editor.viewport.install(scene)
                precondition(editor.viewport.parts.count == 2)
                precondition(editor.viewport.modelHierarchy?.children.first?.name == "Assembly")
                precondition(editor.viewport.modelHierarchy?.children.first?.children.first?.name == "Fixture")
                precondition(editor.viewport.modelHierarchy?.allParts == Set(editor.viewport.parts.map(\.id)))
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
                print("PASS: No Active Group clears selection, Deselect All button and Command-Shift-A action, axis cancellation and document preservation")
                let treeUndo = editor.undoStack
                let treeRedo = editor.redoStack
                editor.resetTreeExpansion()
                editor.refresh()
                let assembly = editor.treeRows.first { $0.node.name == "Assembly" }!.node
                precondition(!editor.treeRows.contains { $0.node.name == "Fixture" })
                editor.toggleTreeNode(assembly.id)
                precondition(editor.treeRows.contains { $0.node.name == "Fixture" })
                editor.toggleTreeNode(assembly.id)
                editor.search.text = "Fixture"
                editor.reloadTree()
                precondition(editor.treeRows.contains { $0.node.name == "Assembly" })
                precondition(editor.treeRows.contains { $0.node.name == "Fixture" })
                editor.search.text = ""
                editor.reloadTree()
                precondition(!editor.treeRows.contains { $0.node.name == "Fixture" })
                editor.selected.removeAll()
                editor.selectTreeNode(assembly)
                precondition(editor.selected == editor.partIDs)
                editor.selectTreeNode(assembly)
                precondition(editor.selected.isEmpty)
                editor.selected = [part.id]
                editor.selectTreeNode(assembly)
                precondition(editor.selected == editor.partIDs)
                for title in editor.sidebarSections.keys {
                    editor.sidebarSections[title]?.0.sendActions(for: .touchUpInside)
                    precondition(editor.sidebarSections[title]?.1.isHidden == true)
                    editor.view.layoutIfNeeded()
                    editor.sidebarSections[title]?.0.sendActions(for: .touchUpInside)
                    precondition(editor.sidebarSections[title]?.1.isHidden == false)
                }
                let targetGroup = editor.treeRows.first { $0.node.groupID == savedGroupID }!.node
                let partDrag = UIDragItem(itemProvider: NSItemProvider(object: "Parts" as NSString))
                partDrag.localObject = TreeDrag(groupID: nil, parts: editor.partIDs)
                let moved = try editor.dropping([partDrag], onto: targetGroup)
                editor.edit { $0 = moved }
                precondition(editor.activeGroup?.parts == editor.partIDs)
                let persistedMove = try RobotModelStorage.loadDocument(at: url, hash: "fixture")
                precondition(persistedMove == moved)
                editor.undoTapped()
                precondition(editor.document == savedDocument)
                editor.redoTapped()
                precondition(editor.document == moved)
                editor.removeSelection()
                precondition(editor.document.groups.allSatisfy { $0.parts.isEmpty })
                let savedCollapsedNodes = editor.collapsedNodes
                editor.collapsedNodes.removeAll()
                editor.reloadTree()
                precondition(!editor.treeRows.contains { $0.node.id == "unassigned" })
                for partID in editor.partIDs {
                    let leaves = editor.treeRows.filter { $0.node.children.isEmpty && $0.node.parts.contains(partID) }
                    precondition(leaves.count == 1 && leaves[0].node.id.hasPrefix("source/"))
                }
                editor.collapsedNodes = savedCollapsedNodes
                editor.undoTapped()
                precondition(editor.document == moved)
                print("PASS: ungrouped parts appear only in the source hierarchy; Remove Parts and undo preserve the model")
                let childGroup = RobotRigGroup(name: "Child", parts: [])
                editor.document.groups.append(childGroup)
                editor.refresh()
                let groupDrag = UIDragItem(itemProvider: NSItemProvider(object: "Child" as NSString))
                groupDrag.localObject = TreeDrag(groupID: childGroup.id, parts: [])
                editor.document = try editor.dropping([groupDrag], onto: targetGroup)
                precondition(editor.document.groups.last?.parentID == savedGroupID)
                editor.refresh()
                let childRow = editor.treeRows.first { $0.node.groupID == childGroup.id }!
                let parentRow = editor.treeRows.first { $0.node.groupID == savedGroupID }!
                precondition(childRow.depth == parentRow.depth + 1)
                groupDrag.localObject = TreeDrag(groupID: savedGroupID, parts: [])
                do { _ = try editor.dropping([groupDrag], onto: childRow.node); preconditionFailure("Tree accepted a cycle") } catch {}
                do { _ = try editor.dropping([groupDrag], onto: targetGroup); preconditionFailure("Tree accepted self-parenting") } catch {}
                groupDrag.localObject = TreeDrag(groupID: childGroup.id, parts: [])
                let rootRow = editor.treeRows.first { $0.node.id == "groups" }!.node
                let rooted = try editor.dropping([groupDrag], onto: rootRow)
                precondition(rooted.groups.last?.parentID == nil)
                editor.importing = true
                do { _ = try editor.dropping([partDrag], onto: targetGroup); preconditionFailure("Tree accepted edit during import") } catch {}
                editor.importing = false
                editor.document = savedDocument
                editor.undoStack = treeUndo
                editor.redoStack = treeRedo
                editor.selectGroup(savedGroupID)
                try RobotModelStorage.save(savedDocument, at: url)
                print("PASS: retained GLB collections, tree collapse/search, batch selection, collapsible sections, part transfer, group nesting, cycle rejection, persistence and undo/redo")
                for index in 0..<30 {
                    editor.document.groups.append(RobotRigGroup(name: "Scroll fixture \(index)", parts: []))
                }
                editor.refresh()
                editor.table.layoutIfNeeded()
                editor.table.setContentOffset(CGPoint(x: 0, y: 440), animated: false)
                editor.table.layoutIfNeeded()
                let scrollOffset = editor.table.contentOffset
                let visiblePaths = editor.table.indexPathsForVisibleRows!
                let visibleCells = visiblePaths.map { editor.table.cellForRow(at: $0)! }
                let visibleAccessories = visibleCells.map { $0.accessoryView! }
                let rowIDs = editor.treeRows.map { $0.node.id }
                for _ in 0..<20 {
                    editor.motorUpdate()
                    editor.table.layoutIfNeeded()
                    precondition(editor.table.contentOffset == scrollOffset)
                    precondition(editor.treeRows.map { $0.node.id } == rowIDs)
                    for (index, path) in visiblePaths.enumerated() {
                        precondition(editor.table.cellForRow(at: path) === visibleCells[index])
                        precondition(editor.table.cellForRow(at: path)?.accessoryView === visibleAccessories[index])
                    }
                }
                editor.document = savedDocument
                editor.refresh()
                editor.table.setContentOffset(.zero, animated: false)
                print("PASS: telemetry refresh preserves hierarchy row order, visible cells and scroll offset")
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
                editor.moveIK(SIMD3(1.5, 0.5, 0), state: .changed)
                let firstGeneration = editor.collisionGeneration
                editor.moveIK(SIMD3(0.5, 1.5, 0), state: .ended)
                precondition(editor.collisionGeneration == firstGeneration && editor.pendingIKMove != nil)
                precondition(editor.collisionCheck == nil, "Drag events must wait for the next frame")
                editor.processIKFrame()
                let batchGeneration = editor.collisionGeneration
                await awaitCollision()
                precondition(editor.collisionGeneration == batchGeneration, "One target must use one solve/check batch")
                precondition(editor.pendingIKMove == nil && editor.collisionCheck == nil)
                precondition(simd_distance(editor.document.ikEndpoint(for: savedGroupID!, angles: editor.angles)!, SIMD3(0.5, 1.5, 0)) < 0.003)
                precondition(abs(editor.angles[root.id] ?? 0) > 1)
                precondition(editor.document == ikDocument && commander.motion.isEmpty)
                editor.moveIK(SIMD3(1.5, 0.5, 0), state: .changed)
                editor.processIKFrame()
                let runningGeneration = editor.collisionGeneration
                editor.moveIK(SIMD3(1, 1, 0), state: .changed)
                editor.moveIK(SIMD3(0.5, 1.5, 0), state: .ended)
                editor.processIKFrame()
                precondition(editor.collisionGeneration == runningGeneration, "A frame must not start a second worker")
                precondition(editor.pendingIKMove?.target == SIMD3(0.5, 1.5, 0), "Only the newest pending target should survive")
                await awaitCollision()
                precondition(editor.pendingIKMove == nil && editor.collisionCheck == nil && editor.ikDragAngles == nil)
                precondition(simd_distance(editor.document.ikEndpoint(for: savedGroupID!, angles: editor.angles)!, SIMD3(0.5, 1.5, 0)) < 0.003)
                SCNTransaction.flush()
                _ = editor.viewport.snapshot()
                let targetScreen = editor.viewport.projectPoint(SCNVector3(editor.ikTarget!))
                let dragStart = CGPoint(x: CGFloat(targetScreen.x), y: CGFloat(targetScreen.y))
                let dragEnd = CGPoint(x: dragStart.x + 25, y: dragStart.y - 10)
                let poseBeforeDrag = editor.angles
                precondition(editor.viewport.beginAxisDrag(at: dragStart))
                editor.viewport.moveAxisDrag(to: dragEnd, state: .changed)
                await awaitCollision()
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
                await awaitCollision()
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
                await awaitCollision()
                precondition(editor.ikMessage!.contains("not reached") && commander.motion.isEmpty)
                editor.moveIK(SIMD3(1, 1, 0), state: .ended)
                await awaitCollision()
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
                editor.showError("Enter three finite coordinates")
                let firstDeadline = editor.errorDeadline!
                editor.motorUpdate()
                precondition(!editor.errorBanner.isHidden && editor.viewportModeLabel.isHidden)
                editor.expireError(at: firstDeadline.addingTimeInterval(-1))
                precondition(!editor.errorBanner.isHidden)
                editor.showError("Replacement error")
                editor.expireError(at: firstDeadline.addingTimeInterval(-1))
                precondition(!editor.errorBanner.isHidden && editor.errorLabel.text == "Replacement error")
                editor.expireError(at: editor.errorDeadline!)
                precondition(editor.errorBanner.isHidden)
                editor.showError("Dismissable error")
                editor.view.layoutIfNeeded()
                precondition(editor.errorBanner.backgroundColor == UIColor.clear && editor.errorLabel.textColor == UIColor.systemRed)
                precondition(editor.errorBanner.arrangedSubviews.count == 1 && editor.errorBanner.arrangedSubviews[0] === editor.errorLabel)
                precondition(editor.errorBanner.gestureRecognizers?.contains(where: { $0 is UITapGestureRecognizer }) == true)
                precondition(abs(editor.errorBanner.frame.maxY - (editor.viewportHost.safeAreaLayoutGuide.layoutFrame.maxY - 8)) < 1)
                precondition(editor.dismissAccessibleError())
                precondition(editor.errorBanner.isHidden && editor.errorDeadline == nil)
                let coverage = await Task.detached {
                    RigCollisionWorld(parts: [.init(id: "empty", name: "empty", owner: "arm", vertices: [])], joints: [], moving: ["arm"])
                        .check(poses: [[:], [:]], timeLimit: 1)
                }.value
                precondition(coverage.message?.contains("unsupported") == true)
                precondition(collisionError(coverage, proposedSamples: 1) == nil, "Coverage notices must not appear as errors")
                precondition(collisionError(coverage, proposedSamples: 2) != nil, "Incomplete motion must still report its message")
                print("PASS: viewport errors survive telemetry, remain for 20 seconds, renew and dismiss; preview badge hidden")
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
                        editor.resetTreeExpansion()
                        editor.refresh()
                        SCNTransaction.flush()
                        print("PASS: sample GLB decoded into \(editor.viewport.parts.count) parts")
                        let meshParts = editor.viewport.parts.map { part in
                            RigCollisionWorld.Part(id: part.id, name: part.name, owner: nil,
                                                   vertices: part.mesh.triangles.flatMap { [$0.first, $0.second, $0.third] })
                        }
                        let preparationStart = Date()
                        let world = await Task.detached { RigCollisionWorld(parts: meshParts, joints: [], moving: []) }.value
                        print("MESH COVERAGE: \(meshParts.count - world.unsupported.count)/\(meshParts.count) components, \(world.surfaceCount) surface-only; preparation \(Date().timeIntervalSince(preparationStart))s")
                        report += "Mesh coverage: \(meshParts.count - world.unsupported.count)/\(meshParts.count) components; \(world.surfaceCount) surface-only\n"
                        if let rigIndex = args.firstIndex(of: "--rig-document"), args.indices.contains(rigIndex + 1) {
                            let saved = try JSONDecoder().decode(RobotRigDocument.self, from: Data(contentsOf: URL(fileURLWithPath: args[rigIndex + 1])))
                            let owned = meshParts.map { part in
                                RigCollisionWorld.Part(id: part.id, name: part.name,
                                                       owner: saved.groups.first { $0.parts.contains(part.id) }?.id.uuidString, vertices: part.vertices)
                            }
                            let joints = saved.groups.compactMap { group in
                                group.axis.map { RigCollisionWorld.Joint(id: group.id.uuidString, parent: group.parentID?.uuidString, pivot: $0.origin, direction: $0.direction) }
                            }
                            let moving = Set(saved.groups.map { $0.id.uuidString })
                            let poses = (0...2).map { step in
                                let angles = Dictionary(uniqueKeysWithValues: saved.groups.map { ($0.id, Float(step) * 0.1) })
                                return Dictionary(uniqueKeysWithValues: saved.transforms(angles: angles).map { ($0.key.uuidString, $0.value) })
                            }
                            let dragPoses = (0...60).map { step in
                                let angle = Float(step <= 30 ? step : 60 - step) / 150
                                let angles = Dictionary(uniqueKeysWithValues: saved.groups.map { ($0.id, angle) })
                                return Dictionary(uniqueKeysWithValues: saved.transforms(angles: angles).map { ($0.key.uuidString, $0.value) })
                            }
                            await Task.detached {
                                let probe = RigCollisionWorld(parts: owned, joints: joints, moving: moving)
                                for budget in [10.0, 0.25] {
                                    let start = Date()
                                    let outcome = probe.check(poses: poses, timeLimit: budget, measureOverlap: false)
                                    print("FULL RIG: budget \(budget); elapsed \(Date().timeIntervalSince(start)); accepted \(outcome.accepted); \(outcome.message ?? "clear")")
                                    for part in owned where outcome.blockedParts.contains(part.id) {
                                        print("BLOCKED OWNER: \(part.name), \(part.id), group \(part.owner ?? "ungrouped")")
                                    }
                                    precondition(outcome.message?.contains("timed out") != true, "Saved-rig collision query exhausted its budget")
                                    precondition(outcome.accepted == poses.count - 1, "Saved-rig small motion was blocked")
                                }
                                var durations: [Double] = []
                                for step in 1..<dragPoses.count {
                                    let start = ContinuousClock.now
                                    let outcome = probe.check(poses: [dragPoses[step - 1], dragPoses[step]], measureOverlap: false)
                                    let elapsed = start.duration(to: .now).components
                                    durations.append(Double(elapsed.seconds) + Double(elapsed.attoseconds) / 1e18)
                                    precondition(outcome.accepted == 1, "Saved-rig incremental motion was blocked: \(outcome.message ?? "unknown")")
                                }
                                let sorted = durations.sorted()
                                print("FULL RIG INCREMENTAL: \(durations.count) checks; mean \(durations.reduce(0, +) / Double(durations.count) * 1000)ms; p95 \(sorted[Int(Double(sorted.count - 1) * 0.95)] * 1000)ms; max \(sorted.last! * 1000)ms")
                            }.value
                        }
                        let wrists = meshParts.filter { $0.name == "Wrist_001 [1]" }
                        let servos = meshParts.filter { $0.name == "Bend Servo_001" }
                        for wrist in wrists {
                            for servo in servos {
                                let pair = [RigCollisionWorld.Part(id: wrist.id, name: wrist.name, owner: "wrist", vertices: wrist.vertices), servo]
                                await Task.detached {
                                    let probe = RigCollisionWorld(parts: pair, joints: [], moving: ["wrist"])
                                    print("COLLISION PROXY: \(wrist.name) \(wrist.vertices.count / 3) -> \(probe.triangleCounts[wrist.id] ?? 0) triangles; relative error \(probe.simplificationErrors[wrist.id] ?? 0)")
                                    let rotation = simd_float4x4(simd_quatf(angle: 0.01, axis: SIMD3<Float>(0, 1, 0)))
                                    for budget in [0.25, 5.0] {
                                        let start = Date()
                                        let outcome = probe.check(poses: [[:], ["wrist": rotation]], timeLimit: budget)
                                        print("MESH PAIR: \(wrist.name) (\(wrist.vertices.count / 3)) / \(servo.name) (\(servo.vertices.count / 3)); surfaces \(probe.surfaceCount); budget \(budget); elapsed \(Date().timeIntervalSince(start)); accepted \(outcome.accepted); \(outcome.message ?? "clear")")
                                    }
                                }.value
                            }
                        }
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