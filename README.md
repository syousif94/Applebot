# RoboCar — Path Navigation

RoboCar is an iOS app that turns an iPhone with LiDAR into the brain of a small robot car. The phone mounts on the car facing backward, connects to an ESP32 motor controller over BLE, and uses ARKit scene reconstruction to build a real-time occupancy grid of the environment. Users can tap a point on the map to navigate there autonomously, or let the car explore on its own.

## Native Controller Connections

Scanning an active pairing QR remembers the peer by its Iroh public identity, not its name. The controller connects after pairing and saves the last successfully connected robot in device-local Keychain storage. Startup and foreground entry resume that connection; offline and busy robots are retried indefinitely with backoff capped at 15 seconds and bounded dial attempts. iOS suspends networking in the background. Local disconnect, deletion, and rejected/revoked pairing stop retries.

Open Paired Devices and use the trash button or swipe Remove Pairing to delete a remembered device. Deleting the reconnect target also cancels its pending connection attempts and clears the saved target.

Device labels use `UIDevice.current.name` and refresh on authenticated reconnection. On iOS 16 and later, obtaining the system-given name requires Apple's `com.apple.developer.device-information.user-assigned-device-name` entitlement. Both entitlement files request it. Request approval at https://developer.apple.com/contact/request/user-assigned-device-name/ and regenerate the provisioning profiles before signing. Without approval, signing can fail or the system can return a generic name. RoboCar does not substitute a custom app nickname.

## Robot Model Editor

Open **Settings > Model Editor** on the robot host or remote controller. The editor is available even when Bluetooth or the remote robot is disconnected. Use Import to choose a GLB from Files. Orbit, pan and pinch in the viewport; tap parts or rows to toggle selection. The eye button isolates the selection. Disconnected mesh components become individually selectable parts, with original materials retained.

The **Hierarchy** sidebar shows nested **Rig Groups** and the original GLB model nodes, including exported collections and each mesh's disconnected components. Ungrouped parts appear only in the model hierarchy, not in a synthetic group. Expand/collapse branches with their chevrons. Tap a row or checkbox to toggle all its descendant parts; a minus indicates partial selection. Selection is shared between both trees and the viewport. Search includes group, collection and part names and reveals matching branches without losing the previous collapse state. Collection names and nesting must be exported as GLB nodes; collections omitted by the exporter cannot be recovered. Each sidebar section, including the hierarchy and grouping tools, collapses independently.

Select parts and choose **Create Group**. Drag a part or model branch onto a rig group to transfer its parts; dragging a fully selected part/branch carries the current part selection. Select a rig group and use **Remove Parts** to remove the selected parts from that group. Drag a rig group onto another group to reparent it, or onto **Rig Groups** to make it a root. Invalid cycles and edits that invalidate an IK chain are rejected. Source GLB nodes remain unchanged. Dragging is available only during ordinary Preview selection. **Add Parts** also changes membership and asks for confirmation when transferring between groups. **Group Settings** provides rename, parent selection and deletion. The group picker displays the active group's name, or **No Active Group** when none is active. A parent moves all its descendant groups. Undo/redo applies to document edits, including drops, but not selection or collapse state. Changes are saved atomically beside a copied GLB under Application Support/RobotModels, keyed by SHA-256. The last imported model reopens automatically; selecting an earlier GLB restores that model's saved rig.

Creating a group automatically enters **Select Axis Face** mode. For an existing group, choose it in the group picker and press **Select Axis Face**. Click/tap a flat surface on one of that group's parts in the 3D viewport. The highlighted region consists of connected coplanar triangles, including triangles separated by material/UV seams. Press **Use Selected Face** to save the axis: the pivot is the region's area-weighted center and the direction is its geometric normal. **Cancel Face Selection** returns to part selection without changing the saved axis. **Flip Axis** reverses direction; **Edit Axis** edits origin/direction in original model-rest coordinates. Curved surfaces are not treated as a single flat face. Once the axis is confirmed, the angle slider previews hierarchy motion; tap its numeric label to enter an exact preview angle. The inspector scrolls on smaller screens, and all actions have visible labels.

The link button configures an ST3215 binding: servo ID, direction (+1/-1), motor-to-joint ratio, motor angle at the model's rest pose, and joint limits. Multi-turn angles are signed hardware-center degrees, not modulo encoder position. Position-mode mapping uses raw encoder degrees (0 through 4095 * 360 / 4096). Bindings saved while connected are scoped to that robot's BLE or Iroh identity; changing transport requires rebinding. Preview never sends motion, torque, calibration, or tracking commands.

**Live control currently supports tracked multi-turn axes only.** Configure tracking, zero, both travel marks and torque in the existing servo controls first. Live requires fresh healthy telemetry and explicit confirmation of calibration. The play button sends a target; the displayed actual angle comes from telemetry. The stop button stops editor-owned motion and returns to Preview. Backgrounding, hiding the editor, stale/error telemetry, and connection changes disarm control. Multi-turn target speed uses the existing firmware default. Physical hardware validation remains required before normal use.

Ordinary position-mode Live moves are intentionally disabled: the current firmware's multi-turn stop does not abort native position moves. A lost transport cannot guarantee a physical stop. This editor does not provide collision avoidance or an emergency-stop guarantee. Never rely on the model preview to establish mechanically safe travel.

Import limits: embedded GLB 2.0 resources, rigid triangle meshes, 100 MB file, 1,000,000 triangles after mesh instances are expanded, 2,000 components, and bounded decoded geometry buffers. Meshopt compression, quantized geometry and WebP textures have been exercised with the repository's MangoBot asset. Draco, KTX2, skinning/morph targets, external resources and other required extensions are rejected. Source animations are not played. Fused surfaces cannot be manually cut; re-export separate parts for those models. The importer is asynchronous, but document copying and geometry/component preparation currently run on the UI thread and can pause interaction for large assets. Models and calibration are local to each app installation; peer synchronization and a model-library browser are not included. Import failures remain visible rather than being replaced by motor telemetry status.

Choose **Move Axis** after confirming an axis to reposition its pivot with red **X**, green **Y**, and blue **Z** cone-tipped handles. Drag a handle in the viewport; movement is constrained to that model-rest coordinate and leaves the axis direction unchanged. Each released drag is saved as one undoable edit. **Done Moving Axis** returns to normal selection and rotation preview. Moving the pivot resets preview rotations to the rest pose and is unavailable in Live mode. **Edit Axis** remains available for exact origin and direction values.

### Collision Exclusions

Select parts in the viewport or hierarchy, then press **Exclude from Collisions** beside **Deselect All** in the sidebar. When all selected parts are excluded, the button becomes **Include in Collisions**. For a mixed selection, it excludes all selected parts. The button is disabled when nothing is selected. You can also open **Collision Checks** and turn off **Check Selected Parts**. Selecting a group or model branch applies the change to all selected descendant parts. Excluded parts remain visible, selectable and move with their groups, but are omitted entirely from IK collision geometry, including collisions against other groups or ungrouped geometry. Exclusions affect whole selectable parts, not individual faces or specific contact pairs.

The section shows the selection's included/excluded/mixed state and the total excluded count. For a mixed selection, switching off excludes the entire selection; switching on includes it. **Select Excluded** selects all excluded parts so you can inspect or re-enable them. **Include All Parts** clears all exclusions. IK status also reports the excluded count.

Exclusions are saved with the model and support undo/redo. Changes cancel pending IK checks and reset preview; restart IK to prepare the updated collision world. Editing is disabled during Live control. Existing rigs default to checking every part. Excluding all parts removes model collision obstacles from IK; no clearance or physical safety is implied. Manual previews and Live commands remain outside collision checking.

### Lead Screws and Parallel Grippers

1. Group the translating parts and set their axis using **Select Axis Face** or **Edit Axis**. The axis direction is the travel direction; its origin does not change linear displacement. Keep the stationary motor housing in its parent group.
2. In **Motion Axis**, select **Linear**. Confirm **Model Units**: meters (the GLB convention), millimeters, or custom millimeters per model unit. This converts travel without rescaling the imported geometry or pivots. Enter minimum/maximum travel in millimeters, including zero at the modeled rest pose. New setups propose 0...10 mm for confirmation.
3. For a lead screw or one moving jaw, leave **Opposing Jaw** unset. For two symmetric jaws, choose the other group there. Eligible groups share the same parent and have parallel or antiparallel axes. Both jaws move equally in opposite physical directions; each jaw's descendants follow it. Linking removes the opposing group's separate motor binding. **None** unlinks it to independent linear motion without a motor.
4. In **Motor Binding**, enter millimeters of travel per actual motor revolution, including any transmission gearing. This is each jaw's travel, not the combined opening. Set the motor degrees at modeled rest, reversal, and limits. At 2 mm/revolution, 10 mm travel maps to 1,800 motor degrees before offset/reversal.
5. Preview with the position slider or tap the numeric value. Selecting either jaw controls the source's displacement. A 3 mm outward move of each jaw increases opening by 6 mm; the editor does not infer absolute jaw aperture. Live uses the same calibration and displays source travel in millimeters from motor telemetry, issuing one motor command for the pair.

**Travel Limits** remain available without a motor and constrain both preview and IK. Changing motion type clears that group's motor binding rather than reusing degree calibration as distance calibration. Switching a source to Rotation or deleting it unlinks its opposing jaw. Reparenting or axis edits that invalidate a pair are rejected; unlink before restructuring. Configuration changes reset preview and support save/load and undo/redo. Editing is disabled while Live is armed.

Legacy rigs retain rotational behavior. Saving linear motion or model scale uses rig format 2, which older app versions reject. This models constant-ratio translation, not thread geometry, nonlinear linkage motion, force control, or automatic homing. Verify actual pitch, zero, travel marks and physical travel under supervision before using connected motors.

### Inverse Kinematics

IK is a position-only, preview-only tool for rigid rotational and linear joint hierarchies. It never sends motor commands.

1. Create groups, set their motion axes, and connect them using **Group Settings > Parent Group**. Configure linear units and limits before using linear IK.
2. Select the final child group, press **Place IK Helper**, and click a point on that child's mesh. This is the point the solver will move toward the target. **Edit IK Helper** also accepts exact model-rest XYZ coordinates, including points outside the mesh.
3. Choose **IK Chain Root** to select the highest ancestor allowed to move. The default is the topmost ancestor. Groups without axes transmit their parent's transform but do not add a joint.
4. Press **Drag IK Target**. Drag the child or the target marker in the camera plane, or use the XYZ cone handles for constrained target movement. Pink marks the requested target; cyan marks the achieved helper position. The inspector reports target distance and whether it was reached.
5. Press **Done With IK** to keep the preview pose, or **Reset Preview** to return to the rest pose.

Helper points and chain roots are saved with the rig and support document undo/redo. Solved poses are temporary, like slider previews. The solver respects motor-binding rotational limits (unbound rotational joints use -180 to 180 degrees) and configured linear travel limits in millimeters. Ancestors above the selected root retain their positions. Branches attached to a moving ancestor follow it but their own independent joints are not solved. Linked jaws are one degree of freedom: targeting either jaw can solve the source travel, even when the source is a sibling, and both branches are collision checked. Chains are limited to 32 groups. Unreachable or constrained targets retain the best pose found; convergence is not guaranteed. There is no orientation target, pole vector, or simultaneous multi-target constraint. Existing rigs without IK helpers still load unchanged.

IK preview checks proposed motion using [CollisionQueries](CollisionQueries/README.md). Collision-only meshes are simplified once per IK preparation with meshoptimizer, targeting 256 triangles per part at a 0.1% relative error budget with locked open borders; topology/error constraints can leave more triangles. Visual meshes and picking are unchanged. Simplified triangles and packed BVHs stay in Metal buffers. CPU bounds find candidate pairs, then one batched GPU dispatch per pose checks triangle crossings and closed-solid containment. Preparation and queries run in background tasks. Drag events coalesce behind one running worker; cancellation and edits discard outstanding results. Interactive checks share a 250 ms budget across preparation and GPU waiting; CPU preparation is checked at phase boundaries. GPU failures and timeouts hold motion. Blocking parts are highlighted red and named in the floating banner. Manual angle previews and Live commands are not collision checked.

Different groups are never excluded as whole pairs merely because they are connected. Direct parent/child pairs whose rest bounds overlap near the hinge axis receive a parent-relative contact sphere sized to enclose that rest overlap. The center lies along the hinge line at the contact, rather than necessarily at the selected pivot point, allowing shaft contacts beyond an outer-face pivot. Its size is fixed from rest geometry, not enlarged by motion. Only collision points within it are ignored. This bounds-derived tolerance can overestimate contact on concave parts; it is not an exact joint boundary. Impacts elsewhere on adjacent links remain checked, as do non-adjacent links, siblings, other branches and ungrouped parts. Parts in the same rigid group are not self-tested. Ungrouped parts have no contact exemption. IK checks proposed samples without measuring the starting pose's overlap; existing intersections outside the joint allowance can block motion. Legacy manual contact exemptions are ignored. Simplification and partial geometry coverage are not physical safety guarantees.

Linear joints receive no hinge contact exemption. Intentional rail or screw intersections can conservatively block IK; there are no automatic sliding-contact allowances. Both linked jaws and their descendants remain checked. Manual previews and Live moves remain outside collision checking.

Drag targets coalesce to display frames (up to 60 Hz), with only the newest target retained while a collision worker runs. Each update solves once, then checks 1...8 interpolated poses. The sample budget sums each independent rotational joint's travel divided by five degrees and each linear joint's travel divided by one millimeter, without double-counting linked jaws. Large jumps have wider spacing because of the eight-sample cap. The preview immediately displays the furthest accepted sample instead of animating one-degree steps. This can miss thin collisions between samples and is not continuous collision detection or obstacle routing. A saved-rig 60-query small-motion test averaged 4.55 ms per GPU-backed query in the Debug simulator (p95 4.94 ms), excluding solving and rendering and not measuring a full multi-sample update. Preparation took 13.8 seconds and remains CPU work. Physical-device performance remains unverified. Empty/degenerate components and components over 20,000 triangles are skipped with an incomplete-check warning; the saved rig covered 193/1,024 components. Surface-only coverage is reported. This does not certify all saved-rig target paths or physical hardware safety.

Errors appear as red text on a transparent background at the bottom of the 3D view for 20 seconds after the latest error, independently of telemetry status updates. Click or tap the message to dismiss it; there is no close button. Successful collision checks do not display incomplete-coverage or surface-only notices as errors; coverage limits still apply. Blocking collisions and query failures remain visible. Ordinary preview mode text is hidden from the viewport; the Preview/Live selector and Live warning remain.

### Editor Checks

From the workspace root:

```sh
DEVELOPER_DIR=/Applications/Xcode-beta.app/Contents/Developer xcrun swiftc \
  RoboCar/RoboCar/Models/RobotRigDocument.swift \
  RoboCar/RoboCar/Models/RobotRigGeometry.swift \
  RoboCar/Tests/RobotRigChecks.swift -o /tmp/robocar-rig-checks
/tmp/robocar-rig-checks
```

Debug builds support the launch argument `--robot-rig-checks` to open isolated runtime checks with synthetic geometry and a mock motor transport. Optional `--rig-model /absolute/path/to/model.glb` also tests the real GLTFKit2 bridge and rendering. On Simulator, provide a host-readable file path. This checks multi-material face picking, persistence, Preview command isolation, motor conversion, pending-command rejection, wrong-robot/session rejection, invalid telemetry and disconnect behavior. Results and a viewport snapshot are written to the app's temporary `RigChecks` directory. The harness is excluded from Release builds.

## Architecture Overview

```
┌─────────────┐      BLE       ┌──────────────┐
│   iPhone     │ ◄────────────► │   ESP32       │
│  (LiDAR +    │   motor cmds   │  Motor Ctrl   │
│   ARKit)     │                │  (4 motors)   │
└──────┬───────┘                └──────────────┘
       │
       ▼
┌──────────────────────────────────────────────┐
│              App Components                   │
│                                               │
│  MeshProcessor ──► OccupancyGrid ◄── ObstacleDetector
│                        │                      │
│                   findPath (A*)               │
│                        │                      │
│                   PathNavigator               │
│                  (pure pursuit)               │
│                        │                      │
│                   ESP32BLEManager             │
│                  (motor commands)             │
└──────────────────────────────────────────────┘
```

## Key Components

| File                              | Role                                                                                                                                                |
| --------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------- |
| `Models/OccupancyGrid.swift`      | 2D grid map — stores cell states (unknown/free/occupied), coordinate conversion, A\* pathfinding, path smoothing, frontier detection                |
| `AR/MeshProcessor.swift`          | Converts ARKit mesh anchors into occupancy grid updates; estimates floor height; classifies surfaces (wall, table, door, etc.)                      |
| `AR/ObstacleDetector.swift`       | Real-time collision avoidance using both the grid and raw LiDAR depth; maintains a protective radius around the device                              |
| `AR/PathNavigator.swift`          | Drives the car along a planned path using pure-pursuit steering; handles obstacle pausing, reverse maneuvers, stuck detection, and live re-planning |
| `AR/ExplorationController.swift`  | Autonomous frontier-based exploration — finds unmapped areas and drives toward them until the full reachable space is mapped                        |
| `BLE/ESP32BLEManager.swift`       | BLE connection to the ESP32; sends 4-motor power commands with a heartbeat watchdog                                                                 |
| `Views/LiDARViewController.swift` | Coordinates everything — runs the ARSession, processes frames, triggers pathfinding on tap, manages navigation re-planning during the render loop   |

## How Path Navigation Works

### 1. Building the Map

Every frame, `LiDARViewController` feeds AR mesh anchors to `MeshProcessor`, which:

1. Extracts vertices and per-face surface classifications from each `ARMeshAnchor`.
2. Filters by height — points between the floor and ~1.8 m are potential obstacles; floor-level points are marked free.
3. Writes results into the `OccupancyGrid` using `markOccupied` / `markFree` / `markRayAsFree` (Bresenham ray-casting clears cells between the device and an obstacle).

The grid is a 1000×1000 array of 5 cm cells (50 m × 50 m coverage) holding three values per cell:

- **State**: `unknown`, `free`, or `occupied`
- **Classification**: wall, table, seat, door, etc.
- **Height range**: min/max observed height

### 2. Requesting a Path

When the user taps the 2D grid map, `LiDARViewController.planPath(toX:toY:)` runs on a background thread:

1. **A\* search** (`OccupancyGrid.findPath`) — traverses only `free` cells with 8-directional movement and an octile distance heuristic. An obstacle proximity penalty (checked within a 3-cell / 15 cm clearance ring) biases the path away from walls.
2. **Greedy fallback** (`OccupancyGrid.findPathGreedy`) — if strict A\* fails (e.g., the target is in unexplored territory), a second A\* run allows traversal through `unknown` cells at a higher cost.
3. **Path smoothing** — a line-of-sight sweep removes unnecessary intermediate waypoints from the raw grid path.
4. **Densification** — waypoints are interpolated at 5 cm intervals to produce a smooth ribbon for the AR overlay and fine-grained pursuit.

The resulting path is displayed as both a 2D line on the `GridMapView` and a 3D green ribbon in the AR scene.

### 3. Following the Path (Pure Pursuit)

`PathNavigator` executes the path with a 10 Hz control loop (`tick()`):

```
every 100 ms:
  1. Check arrival (< 12 cm from target) → stop
  2. Check ObstacleDetector → pause + reverse if blocked
  3. Check stuck detection → reverse if no movement for 1 s
  4. Advance past completed waypoints
  5. Find lookahead point (30 cm ahead on path)
  6. Compute heading error = atan2(dx, dy) − current heading
  7. Differential steering: left/right power = base ± turn·base
  8. Send motor command via ESP32BLEManager
```

**Steering model**: Proportional differential drive. A turn gain of 1.5 maps heading error to a `[-1, 1]` turn value. Each side gets `cruiseSpeed ± turn × cruiseSpeed`, clamped so values between ±50 % are snapped to ±50 % (motor dead-zone avoidance). Motors A & C are the left side, B & D are the right side.

**Waypoint advancement**: The navigator skips past any waypoint closer than 60 % of the lookahead distance, preventing the car from circling back to already-passed points.

### 4. Obstacle Handling

`ObstacleDetector` runs every frame using two data sources:

- **Grid-based**: scans cells within a configurable `stopRadius` (18 cm) around the device for occupied cells.
- **Depth-based**: samples the raw LiDAR depth map on a 16×16 grid to catch dynamic objects (hands, people) that scene reconstruction hasn't meshed yet. Depth points are projected to world space and filtered by floor/ceiling height.

The detector maintains a **360° awareness bubble** but `PathNavigator` uses **bearing-aware** logic to decide how to react. The obstacle detector computes a `blockedWorldDirection` — a weighted centroid of all nearby obstacles. `PathNavigator` compares this to the current travel direction (toward the next waypoint) using a configurable forward cone (`forwardConeHalfAngle`, default 60° → 120° cone).

**Obstacle in the forward travel cone** (blocking the path):

1. `PathNavigator` enters the **paused** state and starts a **reverse maneuver**:
   - Phase 0: reverse at 60 % power for 0.6 s.
   - Phase 1: turn away from the blocked direction for 0.5 s (direction chosen from `ObstacleDetector.blockedLocalDirection`).
2. After reversing, a **re-plan** is forced — `LiDARViewController` re-runs A\* from the new position.
3. If the obstacle clears on its own, navigation resumes without reversing.
4. If still blocked after the `obstaclePauseTimeout` (5 s), another reverse is attempted.

**Obstacle to the side** (not blocking the path):

- The car continues driving. No pause, no reverse.
- An early re-plan is triggered so A\* can re-route around the nearby obstacle if needed.
- If the car was previously paused due to this obstacle being ahead but has since steered to a new heading where it's no longer in the cone, navigation resumes.

This avoids the costly and unnecessary reverse maneuver when, for example, a wall is within 18 cm to the right but the car is driving straight forward or turning left.

**Stuck detection**: If the car's position changes by less than 3 cm over 1 second while power is applied, it's considered stuck and a reverse maneuver is triggered.

### 5. Live Re-Planning

During active navigation, `LiDARViewController` checks `pathNavigator.needsReplan` every frame. When the re-plan interval (2 s) expires:

1. A new A\* path is computed from the current device position to the original target.
2. `PathNavigator.updatePath()` replaces the waypoint list and resets the waypoint index.
3. The AR and 2D overlays are rebuilt with the new path.

This keeps the route up-to-date as the map changes — newly discovered obstacles or cleared space are automatically incorporated.

### 6. Autonomous Exploration

`ExplorationController` provides a higher-level "explore everything" mode:

1. Performs an initial 360° scan (slow rotation) to seed the grid.
2. Finds **frontier clusters** — groups of unknown cells adjacent to free cells — using BFS flood-fill in `OccupancyGrid.findFrontierClusters`.
3. Picks the nearest reachable frontier (checked via `isPathClear` ray-cast).
4. Turns to face the target, then drives toward it with collision checks each 50 ms.
5. Repeats until no frontiers remain or a 5-minute timeout is reached.

Exploration uses simpler bang-bang steering (full-power turn-in-place → straight drive) rather than pure pursuit, since it doesn't need to follow a precise multi-waypoint path.

## Configuration

Key tuning parameters in `PathNavigator`:

| Parameter              | Default | Description                              |
| ---------------------- | ------- | ---------------------------------------- |
| `cruiseSpeed`          | 0.55    | Forward drive power (0–1 scale)          |
| `lookaheadDistance`    | 0.30 m  | Pure pursuit lookahead                   |
| `arrivalThreshold`     | 0.12 m  | Distance to consider "arrived"           |
| `forwardConeHalfAngle` | 60°     | Half-angle of the "blocking" travel cone |
| `replanInterval`       | 2.0 s   | How often to re-run A\*                  |
| `obstaclePauseTimeout` | 5.0 s   | Max wait before re-reversing             |
| `reverseDuration`      | 0.6 s   | How long to reverse                      |
| `reverseTurnDuration`  | 0.5 s   | How long to turn after reversing         |

Key tuning parameters in `ObstacleDetector`:

| Parameter             | Default | Description                                |
| --------------------- | ------- | ------------------------------------------ |
| `stopRadius`          | 0.18 m  | Protective bubble around the device        |
| `floorFilterMargin`   | 0.08 m  | Ignore depth hits within 8 cm of the floor |
| `ceilingFilterHeight` | 1.8 m   | Ignore depth hits above this height        |

Key tuning parameters in `OccupancyGrid`:

| Parameter              | Default   | Description                             |
| ---------------------- | --------- | --------------------------------------- |
| `cellSize`             | 0.05 m    | Grid resolution (5 cm)                  |
| `gridRadius`           | 500 cells | 25 m radius from origin                 |
| A\* clearance          | 3 cells   | 15 cm obstacle avoidance buffer         |
| Greedy unknown penalty | 3.0×      | Extra cost for traversing unknown cells |
