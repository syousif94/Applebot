import UIKit

class PanelViewController: UIViewController {
    var onPanelVisibilityChanged: ((Bool) -> Void)?
    var onPanelResignKey: (() -> Void)?
    #if targetEnvironment(macCatalyst)
    private var layoutRefreshPending = false
    #endif

    override var canBecomeFirstResponder: Bool { true }

    override var keyCommands: [UIKeyCommand]? {
        [
            UIKeyCommand(input: UIKeyCommand.inputEscape, modifierFlags: [], action: #selector(closePanel)),
            UIKeyCommand(input: "w", modifierFlags: .command, action: #selector(closePanel))
        ]
    }

    override func viewDidLoad() {
        super.viewDidLoad()
        NotificationCenter.default.addObserver(self, selector: #selector(windowResignedKey(_:)), name: UIWindow.didResignKeyNotification, object: nil)
    }

    override func viewDidAppear(_ animated: Bool) {
        super.viewDidAppear(animated)
        becomeFirstResponder()
        panelVisibilityChanged(true)
        #if targetEnvironment(macCatalyst)
        scheduleLayoutRefresh()
        #endif
    }

    #if targetEnvironment(macCatalyst)
    override func viewSafeAreaInsetsDidChange() {
        super.viewSafeAreaInsetsDidChange()
        scheduleLayoutRefresh()
    }

    private func scheduleLayoutRefresh() {
        guard !layoutRefreshPending, viewIfLoaded?.window != nil else { return }
        layoutRefreshPending = true
        DispatchQueue.main.async { [weak self] in
            guard let self else { return }
            self.layoutRefreshPending = false
            guard let root = self.viewIfLoaded, root.window != nil else { return }
            self.invalidateLayout(in: root)
            root.layoutIfNeeded()
        }
    }

    private func invalidateLayout(in view: UIView) {
        guard !view.isHidden else { return }
        view.setNeedsLayout()
        for subview in view.subviews {
            invalidateLayout(in: subview)
        }
    }
    #endif

    override func viewWillDisappear(_ animated: Bool) {
        super.viewWillDisappear(animated)
        panelResignedKey()
        panelVisibilityChanged(false)
    }

    func panelVisibilityChanged(_ visible: Bool) {
        onPanelVisibilityChanged?(visible)
    }

    func panelResignedKey() {
        onPanelResignKey?()
    }

    @objc private func windowResignedKey(_ notification: Notification) {
        guard let window = notification.object as? UIWindow, window === viewIfLoaded?.window else { return }
        panelResignedKey()
    }

    @objc private func closePanel() {
        PanelPresentation.close(self)
    }
}

enum PanelPresentation {
    static func prepare(_ controller: UIViewController) {
        #if !targetEnvironment(macCatalyst)
        if UIDevice.current.userInterfaceIdiom == .pad {
            controller.modalPresentationStyle = .custom
            controller.transitioningDelegate = InsetPanelTransition.shared
        }
        #endif
    }

    static func close(_ controller: UIViewController, completion: (() -> Void)? = nil) {
        controller.view.endEditing(true)
        #if targetEnvironment(macCatalyst)
        if let scene = controller.view.window?.windowScene,
           scene.delegate is PanelSceneDelegate {
            PanelWindows.shared.close(scene)
            return
        }
        #endif
        controller.dismiss(animated: true, completion: completion)
    }
}

private final class InsetPanelTransition: NSObject, UIViewControllerTransitioningDelegate {
    static let shared = InsetPanelTransition()

    func presentationController(forPresented presented: UIViewController, presenting: UIViewController?, source: UIViewController) -> UIPresentationController? {
        InsetPanelPresentationController(presentedViewController: presented, presenting: presenting)
    }

    func animationController(forPresented presented: UIViewController, presenting: UIViewController, source: UIViewController) -> UIViewControllerAnimatedTransitioning? {
        InsetPanelAnimator(presenting: true)
    }

    func animationController(forDismissed dismissed: UIViewController) -> UIViewControllerAnimatedTransitioning? {
        InsetPanelAnimator(presenting: false)
    }
}

private final class InsetPanelAnimator: NSObject, UIViewControllerAnimatedTransitioning {
    let presenting: Bool

    init(presenting: Bool) { self.presenting = presenting }

    func transitionDuration(using transitionContext: UIViewControllerContextTransitioning?) -> TimeInterval { 0.2 }

    func animateTransition(using transitionContext: UIViewControllerContextTransitioning) {
        let key: UITransitionContextViewControllerKey = presenting ? .to : .from
        guard let controller = transitionContext.viewController(forKey: key) else {
            transitionContext.completeTransition(false)
            return
        }
        let panel = controller.view!
        if presenting {
            transitionContext.containerView.addSubview(panel)
            panel.frame = transitionContext.finalFrame(for: controller)
            panel.alpha = 0
        }
        UIView.animate(withDuration: transitionDuration(using: transitionContext), animations: {
            panel.alpha = self.presenting ? 1 : 0
        }, completion: { _ in
            panel.alpha = 1
            transitionContext.completeTransition(!transitionContext.transitionWasCancelled)
        })
    }
}

private final class InsetPanelPresentationController: UIPresentationController {
    private let dimmingView = UIControl()

    override var shouldRemovePresentersView: Bool { false }

    override var frameOfPresentedViewInContainerView: CGRect {
        guard let containerView else { return .zero }
        let available = containerView.bounds.inset(by: containerView.safeAreaInsets)
        let inset: CGFloat = available.width < 600 || available.height < 500 ? 8 : 20
        return available.insetBy(dx: inset, dy: inset)
    }

    override func presentationTransitionWillBegin() {
        guard let containerView else { return }
        dimmingView.backgroundColor = UIColor.black.withAlphaComponent(0.35)
        dimmingView.addTarget(self, action: #selector(dismissPanel), for: .touchUpInside)
        containerView.insertSubview(dimmingView, at: 0)
        presentedView?.layer.cornerRadius = 16
        presentedView?.clipsToBounds = true
        presentedView?.accessibilityViewIsModal = true
    }

    override func containerViewWillLayoutSubviews() {
        super.containerViewWillLayoutSubviews()
        dimmingView.frame = containerView?.bounds ?? .zero
        presentedView?.frame = frameOfPresentedViewInContainerView
    }

    override func dismissalTransitionDidEnd(_ completed: Bool) {
        if completed { dimmingView.removeFromSuperview() }
    }

    @objc private func dismissPanel() {
        PanelPresentation.close(presentedViewController)
    }
}

#if targetEnvironment(macCatalyst)
@MainActor
final class PanelWindows {
    static let shared = PanelWindows()
    static let activityType = "com.robocar.utility-panel"

    enum Kind: String { case settings, servos }

    private final class Entry {
        weak var owner: UIWindowScene?
        let controller: PanelViewController
        let kind: Kind
        let onClose: (() -> Void)?
        var session: UISceneSession?
        var pending = false

        init(owner: UIWindowScene, kind: Kind, controller: PanelViewController, onClose: (() -> Void)?) {
            self.owner = owner
            self.kind = kind
            self.controller = controller
            self.onClose = onClose
        }
    }

    private var entries: [String: Entry] = [:]

    func owner(of scene: UIWindowScene) -> UIWindowScene? {
        if let delegate = scene.delegate as? PanelSceneDelegate {
            return delegate.panelKey.flatMap { entries[$0]?.owner }
        }
        return scene
    }

    @discardableResult
    func focus(_ kind: Kind, from scene: UIWindowScene) -> Bool {
        guard let owner = owner(of: scene),
              let entry = entries[key(owner, kind)], entry.session != nil || entry.pending else { return false }
        activate(entry, key: key(owner, kind), from: scene)
        return true
    }

    func open(_ kind: Kind, from scene: UIWindowScene, onClose: (() -> Void)? = nil, makeController: () -> PanelViewController) {
        guard let owner = owner(of: scene) else { return }
        let panelKey = key(owner, kind)
        let entry: Entry
        if let existing = entries[panelKey] {
            entry = existing
        } else {
            entry = Entry(owner: owner, kind: kind, controller: makeController(), onClose: onClose)
            entries[panelKey] = entry
        }
        activate(entry, key: panelKey, from: scene)
    }

    private func key(_ owner: UIWindowScene, _ kind: Kind) -> String {
        "\(owner.session.persistentIdentifier):\(kind.rawValue)"
    }

    private func activate(_ entry: Entry, key: String, from scene: UIWindowScene) {
        guard !entry.pending else { return }
        entry.pending = true
        let activity = NSUserActivity(activityType: Self.activityType)
        activity.userInfo = ["panelKey": key]
        let options = UIScene.ActivationRequestOptions()
        options.requestingScene = scene
        options.collectionJoinBehavior = .disallowed
        UIApplication.shared.requestSceneSessionActivation(entry.session, userActivity: activity, options: options) { [weak self, weak scene] error in
            entry.pending = false
            if entry.session == nil, entry.kind == .servos {
                entry.onClose?()
                self?.entries.removeValue(forKey: key)
            }
            let alert = UIAlertController(title: "Unable to Open Window", message: error.localizedDescription, preferredStyle: .alert)
            alert.addAction(UIAlertAction(title: "OK", style: .default))
            let root = scene?.windows.first(where: \.isKeyWindow)?.rootViewController
            (root?.presentedViewController ?? root)?.present(alert, animated: true)
        }
    }

    func connect(key: String, scene: UIWindowScene) -> PanelViewController? {
        guard let entry = entries[key], entry.owner != nil,
              entry.session == nil || entry.session?.persistentIdentifier == scene.session.persistentIdentifier else { return nil }
        entry.session = scene.session
        entry.pending = false
        scene.title = entry.kind == .settings ? "Settings" : "Servos"
        scene.titlebar?.titleVisibility = .visible
        scene.sizeRestrictions?.minimumSize = CGSize(width: 440, height: 420)
        entry.controller.preferredContentSize = entry.kind == .settings ? CGSize(width: 620, height: 760) : CGSize(width: 720, height: 800)
        return entry.controller
    }

    func activated(key: String) { entries[key]?.pending = false }

    func hasForegroundPanels(ownerID: String) -> Bool {
        UIApplication.shared.connectedScenes.contains { scene in
            guard let delegate = scene.delegate as? PanelSceneDelegate,
                  delegate.panelKey?.hasPrefix(ownerID + ":") == true else { return false }
            return scene.activationState == .foregroundActive || scene.activationState == .foregroundInactive
        }
    }

    func close(_ scene: UIWindowScene) {
        (scene.delegate as? PanelSceneDelegate)?.prepareToClose()
        UIApplication.shared.requestSceneSessionDestruction(scene.session, options: nil) { error in
            (scene.delegate as? PanelSceneDelegate)?.restoreAfterFailedClose()
            print("[Panels] Could not close window: \(error)")
        }
    }

    func disconnected(key: String, session: UISceneSession) {
        guard let entry = entries[key], entry.session?.persistentIdentifier == session.persistentIdentifier else { return }
        entry.session = nil
        entry.pending = false
        entry.onClose?()
        if entry.kind == .servos { entries.removeValue(forKey: key) }
    }

    func closeAll(ownerID: String) {
        let keys = entries.keys.filter { $0.hasPrefix(ownerID + ":") }
        for key in keys {
            guard let entry = entries.removeValue(forKey: key) else { continue }
            entry.controller.panelResignedKey()
            entry.controller.panelVisibilityChanged(false)
            entry.onClose?()
            if let session = entry.session {
                UIApplication.shared.requestSceneSessionDestruction(session, options: nil)
            }
        }
    }
}

final class PanelSceneDelegate: UIResponder, UIWindowSceneDelegate {
    var window: UIWindow?
    private(set) var panelKey: String?

    func scene(_ scene: UIScene, willConnectTo session: UISceneSession, options connectionOptions: UIScene.ConnectionOptions) {
        guard let windowScene = scene as? UIWindowScene,
              let activity = connectionOptions.userActivities.first(where: { $0.activityType == PanelWindows.activityType }),
              let key = activity.userInfo?["panelKey"] as? String,
              let controller = PanelWindows.shared.connect(key: key, scene: windowScene) else {
            UIApplication.shared.requestSceneSessionDestruction(session, options: nil)
            return
        }
        panelKey = key
        window = UIWindow(windowScene: windowScene)
        window?.rootViewController = controller
        window?.makeKeyAndVisible()
    }

    func scene(_ scene: UIScene, continue userActivity: NSUserActivity) {
        if let panelKey { PanelWindows.shared.activated(key: panelKey) }
        window?.makeKeyAndVisible()
        if let windowScene = scene as? UIWindowScene { updateLayout(in: windowScene) }
    }

    func sceneDidBecomeActive(_ scene: UIScene) {
        if let panelKey { PanelWindows.shared.activated(key: panelKey) }
        if let windowScene = scene as? UIWindowScene { updateLayout(in: windowScene) }
        (window?.rootViewController as? PanelViewController)?.panelVisibilityChanged(true)
    }

    func windowScene(_ windowScene: UIWindowScene, didUpdateEffectiveGeometry previousEffectiveGeometry: UIWindowScene.Geometry) {
        updateLayout(in: windowScene)
    }

    private func updateLayout(in scene: UIWindowScene) {
        guard let window else { return }
        logLayout(in: scene, phase: "before")
        window.setNeedsLayout()
        window.layoutIfNeeded()
        window.rootViewController?.view.setNeedsLayout()
        window.rootViewController?.view.layoutIfNeeded()
        logLayout(in: scene, phase: "after")
    }

    private func logLayout(in scene: UIWindowScene, phase: String) {
        guard let window, let root = window.rootViewController?.viewIfLoaded else { return }
        print("[PanelLayout] \(scene.title ?? "Panel") \(phase) scene=\(scene.effectiveGeometry.coordinateSpace.bounds) window=\(window.frame) root=\(root.frame) bounds=\(root.bounds) safe=\(root.safeAreaInsets) keyboard=\(root.keyboardLayoutGuide.layoutFrame) ambiguous=\(root.hasAmbiguousLayout)")
        logScrollLayout(in: root)
    }

    private func logScrollLayout(in view: UIView) {
        guard !view.isHidden else { return }
        if let scroll = view as? UIScrollView {
            print("[PanelLayout] scroll frame=\(scroll.frame) bounds=\(scroll.bounds) offset=\(scroll.contentOffset) size=\(scroll.contentSize) inset=\(scroll.adjustedContentInset) content=\(scroll.contentLayoutGuide.layoutFrame) ambiguous=\(scroll.hasAmbiguousLayout)")
        }
        for subview in view.subviews {
            logScrollLayout(in: subview)
        }
    }

    func sceneWillResignActive(_ scene: UIScene) { prepareToClose() }

    func sceneDidEnterBackground(_ scene: UIScene) { prepareToClose() }

    func prepareToClose() {
        let panel = window?.rootViewController as? PanelViewController
        panel?.panelResignedKey()
        panel?.panelVisibilityChanged(false)
    }

    func restoreAfterFailedClose() {
        (window?.rootViewController as? PanelViewController)?.panelVisibilityChanged(true)
    }

    func sceneDidDisconnect(_ scene: UIScene) {
        prepareToClose()
        window?.rootViewController = nil
        window = nil
        if let panelKey { PanelWindows.shared.disconnected(key: panelKey, session: scene.session) }
    }
}
#endif