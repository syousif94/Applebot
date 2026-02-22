# RoboCar — Path Navigation

RoboCar is an iOS app that turns an iPhone with LiDAR into the brain of a small robot car. The phone mounts on the car facing backward, connects to an ESP32 motor controller over BLE, and uses ARKit scene reconstruction to build a real-time occupancy grid of the environment. Users can tap a point on the map to navigate there autonomously, or let the car explore on its own.

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
