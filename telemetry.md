# RoboCar Telemetry Spec

Wire format for streaming navigation session telemetry from an iOS app to a Rust server over WebSocket. The Rust server processes incoming data and relays it to a Three.js web client for real-time 3D replay.

## System Overview

```
iPhone (WS client) ──ws──► Rust Server ──ws──► Three.js Web Client
                   JSON lines         processed frames
                      │
                      ▼
               local .jsonl file
            (crash-safe persistence)
```

- The **iPhone** writes every event as a JSON line to a local `.jsonl` file AND streams it over WebSocket when connected.
- The **Rust server** ingests the stream, processes mesh geometry, and fans out to web clients.
- The **Three.js client** renders a 3D replay of the navigation session.

---

## Wire Format

Every message is a single JSON object with a `type` field used as the serde tag. On disk, each message is one line in a `.jsonl` file.

All messages share these top-level fields:

```rust
struct Envelope {
    #[serde(rename = "type")]
    msg_type: String,    // discriminator
    ts: f64,             // unix epoch seconds, ms precision
    seq: u64,            // monotonic sequence number (survives crash)
    session_id: String,  // UUID, new per app launch or grid reset
    // ... event-specific fields are flattened into the same object
}
```

In the JSON wire format, the envelope fields and event fields are **merged at the top level** (not nested). Example:

```json
{"type":"nav_frame","ts":1740441600.123,"seq":42,"session_id":"abc-123","pose":{...},"motor_cmd":{...},...}
```

---

## Coordinate System

The app uses ARKit with a LiDAR-equipped iPhone. The coordinate system for the 2D ground plane:

- **X** = ARKit world X (meters, rightward)
- **Y** = ARKit world Z (meters, forward on the ground plane)
- **Z** = ARKit world Y (meters, height/up)
- **Heading** = `atan2(forward.x, forward.z)` in ARKit space, radians. 0 = facing +Z (forward on the ground plane).

The phone is mounted **facing backward** on the car. Motor commands are inverted in software so positive power = forward physical motion.

---

## Shared Types

These appear in multiple event types. All field names use `snake_case` in JSON.

### `Vec2`

```rust
struct Vec2 {
    x: f32,
    y: f32,
}
```

### `Vec3`

```rust
struct Vec3 {
    x: f32,
    y: f32,
    z: f32,
}
```

### `Pose`

Full 6-DOF device pose at the instant of the event. Included in every event so any single record can be placed in 3D space without context.

```rust
struct Pose {
    x: f32,             // world X (meters)
    y: f32,             // world Y (meters) — ARKit Z
    z: f32,             // height (meters) — ARKit Y
    heading: f32,       // radians
    heading_deg: f32,   // degrees (derived, for readability)
    transform: [f32; 16], // raw ARKit 4×4 column-major transform
}
```

### `Motor`

Four motor power values, range -100 to 100 each. A & C = left side, B & D = right side.

```rust
struct Motor {
    a: i8,  // front-left
    b: i8,  // front-right
    c: i8,  // rear-left
    d: i8,  // rear-right
}
```

### `ObstacleRecord`

A single detected obstacle near the device.

```rust
struct ObstacleRecord {
    distance: f32,          // meters from device
    classification: String, // "wall" | "table" | "seat" | "door" | "window" | "floor" | "ceiling" | ""
    angle_deg: i32,         // horizontal angle from forward, signed (neg=left, pos=right)
    direction: String,      // "Ahead" | "Left" | "Right" | "Front-Left" | "Front-Right" | "Behind"
    elevation_deg: i32,     // vertical angle (0=level, neg=below, pos=above)
    is_depth_based: bool,   // true = LiDAR depth map, false = reconstructed mesh grid
    world_pos: Option<Vec2>, // world position if known
}
```

---

## Event Types

### 1. `nav_frame` (~10 Hz)

The core replay record. Contains the **complete algorithm state** at each tick so the Rust server can reconstruct every decision. **Emitted continuously at ~10 Hz whenever the telemetry service is connected**, regardless of whether navigation is active. When idle, the steering/waypoint/pursuit fields will contain zeroed or stale values, but `pose`, `obstacle`, and `motor_cmd` are always live.

```rust
struct NavFrame {
    pose: Pose,
    motor_cmd: Motor,       // AFTER obstacle filtering (what actually got sent)
    motor_cmd_raw: Motor,   // BEFORE obstacle filtering (algorithm's intent)
    steering: SteeringState,
    waypoint: WaypointState,
    obstacle: ObstacleState,
    pursuit: PurePursuitState,
    ble_connected: bool,
    motor_write_in_flight: bool, // true if BLE write hasn't completed (command may not have reached ESP32)
}
```

#### `SteeringState`

The steering algorithm's computed values for this tick.

```rust
struct SteeringState {
    mode: String,           // "point_turn" | "arc" | "reverse_phase0" | "reverse_phase1" | "stopped"
    heading_error: f32,     // radians, signed (>0 = need to turn right)
    heading_error_deg: f32,
    speed_factor: f32,      // effective multiplier (approach slowdown × depth evasion)
    depth_evasion_bias: f32, // radians added to heading by depth-based evasion
    spin_power: f32,        // point-turn spin power (0 if not in point-turn mode)
    arc_turn: f32,          // proportional arc steering value (-1..1)
}
```

#### `WaypointState`

The current waypoint tracking state.

```rust
struct WaypointState {
    index: u32,             // index into current path
    count: u32,             // total waypoints in current path
    current: Option<Vec2>,  // world position of current waypoint
    lookahead: Option<Vec2>, // pure pursuit lookahead point
    target: Option<Vec2>,   // final destination
    dist_to_target: Option<f32>,
    dist_to_waypoint: Option<f32>,
}
```

#### `ObstacleState`

Snapshot of the obstacle detector at this tick.

```rust
struct ObstacleState {
    detected: bool,
    nearest_dist: Option<f32>,
    nearest_classification: Option<String>,
    blocked_local_dir: Option<Vec2>,  // device-local (x>0=right, y>0=ahead)
    blocked_world_dir: Option<Vec2>,  // world-space unit vector toward obstacle centroid
    blocking_travel: bool,            // is obstacle within the forward travel cone?
    depth_dist: Option<f32>,          // LiDAR depth-map obstacle distance
    depth_world_pos: Option<Vec3>,    // LiDAR depth-map obstacle world position
    nearby_count: u32,
    nearby: Vec<ObstacleRecord>,      // all obstacles within stop radius, sorted by distance
}
```

#### `PurePursuitState`

Pure pursuit algorithm internals.

```rust
struct PurePursuitState {
    lookahead_dist: f32,          // effective lookahead distance used this tick (shrinks near target)
    lookahead_point: Vec2,        // the point on the path being pursued
    target_heading: f32,          // heading toward lookahead (radians)
    stuck_detected: bool,         // true if stuck detection fired this tick
    dist_since_stuck_check: Option<f32>, // distance moved since last stuck check
}
```

---

### 2. `obstacle_map` (~1 Hz)

World-space positions of all occupancy grid cells classified as obstacles within a radius around the device. Sent at the same rate as `mesh_snapshot`. Excludes floor and ceiling cells. Allows the web client to render obstacle locations as a point cloud overlaid on the 3D mesh.

```rust
struct ObstacleMap {
    pose: Pose,
    radius: f32,                         // query radius in meters (default 3.0)
    cell_count: u32,                     // number of obstacle cells returned
    cells: Vec<ObstacleCell>,            // all obstacle cells within radius
    depth_obstacle: Option<DepthObstaclePoint>, // current LiDAR depth-map obstacle
}
```

#### `ObstacleCell`

A single occupied grid cell that is not floor or ceiling.

```rust
struct ObstacleCell {
    x: f32,               // world X (meters)
    y: f32,               // world Y (meters) — ARKit Z
    height: f32,          // max vertex height above floor (meters)
    classification: String, // "wall" | "table" | "seat" | "door" | "window" | ""
    distance: f32,        // distance from device (meters)
}
```

#### `DepthObstaclePoint`

The nearest obstacle detected by the raw LiDAR depth map (catches dynamic objects not yet in the reconstructed mesh).

```rust
struct DepthObstaclePoint {
    x: f32,       // world X
    y: f32,       // world Y (ARKit Z)
    z: f32,       // height (ARKit Y)
    distance: f32, // distance from device (meters)
}
```

---

### 3. `mesh_snapshot` (~0.5–1 Hz)

Raw mesh geometry for Three.js 3D reconstruction plus a 2D occupancy grid summary. Sent at a lower rate than nav frames.

```rust
struct MeshSnapshot {
    pose: Pose,
    anchors: Vec<MeshAnchorData>,  // only anchors changed since last snapshot
    grid: GridSnapshot,
}
```

#### `MeshAnchorData`

A single ARKit mesh anchor's geometry. Binary data is base64-encoded for JSON transport.

```rust
struct MeshAnchorData {
    anchor_id: String,           // UUID
    transform: [f32; 16],        // 4×4 column-major
    vertices_b64: String,        // base64(f32[]), flattened [x,y,z, x,y,z, ...] in anchor-local space
    vertex_count: u32,
    indices_b64: String,         // base64(u32[]), flattened triangle indices [i0,i1,i2, ...]
    triangle_count: u32,
    classifications_b64: String, // base64(u8[]), one per face — see MeshClassification enum below
    generation: u32,             // discard if stale after grid reset
}
```

**MeshClassification values** (matches ARMeshClassification):

| Value | Label   |
| ----- | ------- |
| 0     | none    |
| 1     | wall    |
| 2     | floor   |
| 3     | ceiling |
| 4     | table   |
| 5     | seat    |
| 6     | window  |
| 7     | door    |

#### `GridSnapshot`

Run-length-encoded 2D occupancy grid for a region around the device. Cell states: 0=unknown, 1=free, 2=occupied.

```rust
struct GridSnapshot {
    origin_x: f32,              // world X of cell [0][0]
    origin_y: f32,              // world Y of cell [0][0]
    cell_size: f32,             // meters per cell (default 0.05 = 5cm)
    width: u32,                 // cells across
    height: u32,                // cells tall
    cells_rle_b64: String,      // base64 of RLE-encoded cell states (see below)
    classifications_rle_b64: String, // base64 of RLE-encoded classification values
    occupied_count: u32,
    free_count: u32,
    floor_height: f32,          // estimated floor plane Y in world coords
}
```

**RLE encoding**: sequence of `(value: u8, count: u16_le)` pairs = 3 bytes each. Cells are serialized row-major (all columns of row 0, then row 1, etc.).

---

### 4. `route_planned`

Emitted every time a path is computed — initial plan or replan. Contains the path, the algorithm used, timing, and optionally a grid snapshot showing what A\* was working with.

```rust
struct RoutePlanned {
    route_id: String,                // UUID
    pose: Pose,                      // device pose at plan time
    target: Vec2,                    // destination
    origin: Vec2,                    // start position fed to pathfinder
    waypoints: Vec<Vec2>,            // the computed path
    waypoint_count: u32,
    path_length_meters: f32,         // sum of segment distances
    plan_duration_ms: f32,           // wall time to compute the path
    algorithm: String,               // "astar" | "greedy"
    reason: String,                  // see RoutePlanReason below
    replaces_route_id: Option<String>, // previous route this replaces (for replan chains)
    grid_at_plan_time: Option<GridSnapshot>, // optional: the grid A* used for planning
}
```

**`reason` values:**

| Value                   | Description                                       |
| ----------------------- | ------------------------------------------------- |
| `user_tap`              | User tapped a point on the grid map               |
| `replan_periodic`       | Periodic replan during navigation (~2s interval)  |
| `replan_obstacle`       | Obstacle detected, replanning around it           |
| `replan_stuck`          | Stuck detection fired, replanning                 |
| `replan_path_exhausted` | Ran out of waypoints before reaching target       |
| `exploration`           | Autonomous exploration selected a frontier target |
| `voice`                 | Voice assistant commanded navigation              |

---

### 5. `nav_state_change`

Emitted on every navigation state machine transition. States: `idle`, `navigating`, `paused`, `arrived`.

```rust
struct NavStateChange {
    from: String,
    to: String,
    reason: String,           // see NavStateChangeReason below
    pose: Pose,
    route_id: Option<String>, // active route at time of change
    dist_to_target: Option<f32>,
    motor: Motor,             // motor state at transition
}
```

**`reason` values:**

| Value               | Description                                      |
| ------------------- | ------------------------------------------------ |
| `started`           | Navigation started by user or voice              |
| `arrived`           | Reached target within arrival threshold (0.12m)  |
| `user_stop`         | User pressed stop button                         |
| `obstacle_blocking` | Obstacle in forward travel cone, paused          |
| `obstacle_cleared`  | Obstacle no longer blocking, resumed             |
| `replan_resume`     | New path found, resuming from paused             |
| `stuck`             | Stuck detection (not moving despite motor power) |
| `reverse_start`     | Initiating reverse maneuver                      |
| `reverse_complete`  | Reverse maneuver finished                        |
| `path_exhausted`    | Ran out of waypoints                             |
| `cancelled`         | Exploration or task cancelled                    |

---

### 6. `obstacle_event`

Emitted when obstacle state **changes** (new detection, cleared, action taken). More detailed than the per-frame `ObstacleState`.

```rust
struct ObstacleEvent {
    pose: Pose,
    action: String,           // see ObstacleAction below
    obstacles: Vec<ObstacleRecord>,
    motor_before: Motor,      // what the algorithm wanted to send
    motor_after: Motor,       // what actually got sent after filtering
    trigger_source: String,   // "mesh_grid" | "depth_map" | "both"
    replan_attempt: Option<u32>, // which replan attempt number (1, 2, 3...)
    route_id: Option<String>,
}
```

**`action` values:**

| Value               | Description                                   |
| ------------------- | --------------------------------------------- |
| `detected`          | Obstacle newly detected within stop radius    |
| `cleared`           | All obstacles cleared                         |
| `replan_requested`  | Requesting path replan around obstacle        |
| `reverse_initiated` | Starting reverse maneuver (replans exhausted) |
| `evasion_steering`  | Depth-based steering bias being applied       |
| `motor_filtered`    | Motor command was blocked/modified by filter  |

---

### 7. `exploration_event`

Emitted during autonomous exploration (frontier-based mapping).

```rust
struct ExplorationEvent {
    status: String,              // see below
    pose: Pose,
    frontier_count: u32,
    nearest_frontier_dist: Option<f32>,
    selected_target: Option<Vec2>,
    total_turns: u32,
    total_drive_segments: u32,
    elapsed_sec: f32,
    message: String,             // human-readable status for logging
}
```

**`status` values:** `started`, `scanning`, `target_selected`, `turning`, `driving`, `stuck`, `completed`, `stopped`, `time_limit`

---

### 8. `calibration_event`

Emitted during motor calibration (measures motor power → velocity relationship).

```rust
struct CalibrationEvent {
    phase: String,                   // "sample" | "result"
    // Sample fields (present when phase == "sample")
    test_index: Option<u32>,
    total_tests: Option<u32>,
    left_power: Option<i8>,
    right_power: Option<i8>,
    linear_speed: Option<f32>,       // m/s, from ARKit position tracking
    angular_velocity: Option<f32>,   // rad/s
    duration: Option<f32>,           // seconds
    distance: Option<f32>,           // meters traveled
    angle_turned: Option<f32>,       // radians
    // Result fields (present when phase == "result")
    speed_per_power_unit: Option<f32>,
    max_turn_rate: Option<f32>,      // rad/s at power 100
    left_right_bias: Option<f32>,    // >1.0 means left side faster
}
```

---

### 9. `session_start`

Emitted once when the telemetry service initializes.

```rust
struct SessionStart {
    session_id: String,
    grid_cell_size: f32,       // 0.05 (5cm)
    grid_radius: u32,          // 500 (= 25m radius)
    ar_tracking_state: String, // "normal" | "limited" | "not_available"
    ble_state: String,         // "connected" | "disconnected" | "scanning" | "connecting"
    app_version: String,
    device_model: String,
    os_version: String,
}
```

---

### 10. `session_end`

Emitted when the session ends cleanly. If absent in a `.jsonl` file, the session crashed.

```rust
struct SessionEnd {
    session_id: String,
    duration_sec: f32,
    total_frames: u64,         // total nav_frame count
    total_routes: u32,
    total_obstacle_events: u32,
    total_replans: u32,
    final_grid: GridSnapshot,
    end_reason: String,        // "clean" | "crash" | "background" | "user_stop"
}
```

---

### 11. `crash_breadcrumb`

Written to a file on the iPhone every tick (overwritten atomically). If the app crashes, this file survives and contains the last known state. **On next launch**, the app detects the leftover breadcrumb (it is deleted on clean shutdown), and **sends it as a `crash_breadcrumb` event over WebSocket** at the beginning of the new session. The Rust server should use it to annotate the previous session's crash point.

```rust
struct CrashBreadcrumb {
    seq: u64,
    ts: f64,
    session_id: String,
    pose: Pose,
    motor: Motor,
    nav_state: String,
    waypoint_index: u32,
    heading_error: f32,
    obstacle_detected: bool,
    nearest_obstacle_dist: Option<f32>,
    steering_mode: String,
    recent_logs: Vec<String>,  // last 5 log messages from PathNavigator/ObstacleDetector
}
```

When the server receives a `session_start` and the previous session has no `session_end`, it should treat the previous session as crashed. The breadcrumb (delivered as a `crash_breadcrumb` event at the start of the new session) provides the final state.

---

## Algorithm Reference

This section describes the navigation algorithms so the Rust server and Three.js client can visualize decisions meaningfully.

### Navigation State Machine

```
idle ──► navigating ──► arrived
              │  ▲
              ▼  │
           paused
```

- **idle**: No active navigation
- **navigating**: Following a path using pure pursuit at ~10 Hz
- **paused**: Obstacle blocking forward travel cone; waiting for replan or reverse
- **arrived**: Within 0.12m of target

### Pure Pursuit Steering

Each tick at 10 Hz:

1. Read device position from ARKit via `OccupancyGrid.devicePosition`
2. Walk along the path from current waypoint index to find a **lookahead point** at `lookahead_dist` arc-length ahead on the path
3. Compute `heading_error` = angle from current heading to the lookahead point
4. Apply **depth-based evasion**: if LiDAR depth map detects an obstacle within 30cm ahead, bias the heading error away from it and reduce speed
5. Choose steering mode:
   - If `|heading_error| > 50°`: **point turn** (spin in place, proportional power)
   - Else: **arc steering** (differential drive, proportional to heading error)
6. Apply **approach slowdown** when within 0.40m of target
7. Send motor command through obstacle filter

### Obstacle Detection

Two sources run every frame (~20-30 Hz):

1. **Mesh grid**: queries `OccupancyGrid` for occupied cells within `stop_radius` (70mm). Ignores floor/ceiling classifications.
2. **LiDAR depth map**: samples a 16×16 grid of the raw depth buffer, projects to world space, filters floor/ceiling by height. Detection radius: 300mm.

When obstacles are detected:

- Compute weighted direction toward obstacle centroid (closer = more weight)
- Motor commands heading toward obstacle are blocked
- Navigation pauses and requests replan (up to 3 attempts)
- If replans fail, initiates reverse maneuver (0.6s back + 0.5s turn)

### Pathfinding

Two algorithms, tried in order:

1. **A\*** (strict): only traverses `free` cells. 8-directional movement with octile distance heuristic. Clearance buffer: cells within 10cm of obstacles are near-impassable (cost +1000), cells within 20cm get graduated penalty. Path is smoothed by removing waypoints where line-of-sight exists (with a 15cm buffer for car width).

2. **Greedy A\*** (fallback): same as above but `unknown` cells are traversable with a 3× cost penalty. Used when strict A\* finds no path (common when navigating toward unexplored areas).

### Occupancy Grid

- Cell size: 0.05m (5cm)
- Grid radius: 500 cells (25m from center)
- Total: 1000×1000 cells, 50m × 50m
- Cell states: `unknown` (0), `free` (1), `occupied` (2)
- Each cell also stores: min/max height, mesh classification, update ID
- Coordinate mapping: `worldToGrid(x, y)` → `gridX = (x / cellSize) + gridRadius`, same for Y

### Motor Control

- 4 motors: A (front-left), B (front-right), C (rear-left), D (rear-right)
- Power range: -100 to +100 per motor
- Differential drive: left side = A,C; right side = B,D
- Commands sent over BLE to ESP32 as 4 bytes
- Heartbeat at 200ms to prevent ESP32 watchdog timeout (500ms)
- **Wiring is inverted**: positive software value = backward physical; the code negates before sending

---

## Persistence & Crash Recovery

### Local Storage (iPhone)

```
Documents/telemetry/
├── {session_id}.jsonl     # one JSON line per event, flushed every 1s or 100 records
├── breadcrumb.bin         # mmap'd CrashBreadcrumb, overwritten every nav tick
└── upload_cursor.json     # tracks last seq ACK'd by server per session
```

### Recovery Flow

1. On app launch, check `breadcrumb.bin`
2. If `breadcrumb.session_id` differs from any active session → previous session crashed
3. Read the orphaned `.jsonl` file, append synthetic `session_end` with `end_reason: "crash"`
4. Send `crash_breadcrumb` event at the start of the new session
5. Background-upload completed `.jsonl` files to the server

### WebSocket Protocol

- iPhone connects to `ws://{server}/ingest/{session_id}`
- Each JSON line is sent as a text frame
- Server ACKs with `{"ack": seq}` so the client tracks what's been received
- On disconnect, events accumulate in the `.jsonl` file
- On reconnect, unsent events (seq > last ACK'd) are replayed from the file
- Server can request a full grid snapshot: `{"cmd": "request_mesh_snapshot"}`

---

## Bandwidth Estimates

| Event            | Size/msg  | Frequency                      | Bandwidth    |
| ---------------- | --------- | ------------------------------ | ------------ |
| `nav_frame`      | ~800 B    | 10 Hz                          | ~8 KB/s      |
| `obstacle_map`   | ~2-10 KB  | 1 Hz                           | ~5 KB/s      |
| `mesh_snapshot`  | ~20-50 KB | 0.5 Hz                         | ~15 KB/s     |
| `route_planned`  | ~2 KB     | on replan (~0.5 Hz during nav) | ~1 KB/s      |
| `obstacle_event` | ~1 KB     | on change                      | <1 KB/s      |
| Other events     | ~200 B    | rare                           | <0.1 KB/s    |
| **Total**        |           |                                | **~30 KB/s** |

Per-message deflate on the WebSocket would roughly halve this. The base64-encoded mesh/grid data is the dominant cost and compresses well.

---

## Rust Server Responsibilities

1. **Ingest**: accept WebSocket connections from iPhones, parse JSON lines, persist to storage
2. **Session management**: track active sessions, detect crashes (missing `session_end`), accept `.jsonl` uploads for completed sessions
3. **Mesh processing**: decode base64 vertex/index buffers from `MeshAnchorData`, transform to world space, build Three.js-compatible geometry (positions, indices, colors by classification)
4. **Grid processing**: decode RLE grid snapshots, maintain current grid state per session
5. **Relay to web clients**: fan out processed frames to connected Three.js clients via WebSocket
6. **Replay**: serve historical sessions with original timing for playback
7. **ACK protocol**: send `{"ack": seq}` back to the iPhone so it can trim its upload cursor

### Suggested Rust Crates

- `tokio` + `tokio-tungstenite` for async WebSocket
- `serde` + `serde_json` for deserialization (use `#[serde(tag = "type")]` on the message enum)
- `base64` for decoding mesh/grid binary data
- `glam` for f32 vector/matrix math (matches the SIMD types)
