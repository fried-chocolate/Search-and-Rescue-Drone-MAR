# Search-and-Rescue Drone Simulation — Progress Track

> Last updated: **2026-03-09** (bugfix run)
> Stack: ROS2 Jazzy · Gazebo Harmonic · Python · SDF

---

## Bugfix Session — Rendering & Shutdown Issues ✅ FIXED

### Root cause diagnosis
`glxinfo -B` showed the VirtualBox SVGA3D driver (Mesa 25.2.8) advertises
OpenGL 4.1 but its **GLSL shader pipeline is incomplete** — Ogre2 (Gazebo
Harmonic's default renderer) fails silently and produces a fully black
render window. This affected both the Gazebo GUI and the camera sensor (which
renders the scene with the same Ogre2 backend).

### Fixes applied

| File | Fix |
|---|---|
| `launch/drone_sim.launch.py` | Added `LIBGL_ALWAYS_SOFTWARE=1` + `MESA_GL_VERSION_OVERRIDE=3.3` to Gazebo `ExecuteProcess`. Forces Mesa **LLVMpipe** (CPU software renderer), which is fully conformant and correctly runs Ogre2 GLSL shaders. |
| `worlds/rescue_world.sdf` | Added `<scene>` block with ambient light, sky-blue background, and clouds so Ogre2 always has a valid scene to initialize against. |
| `drone_sim/takeoff.py` | Wrapped `rclpy.shutdown()` in `try/except` — SIGINT already calls shutdown; the `finally` block was raising `RCLError` (harmless but noisy). |
| `drone_sim/camera_viewer.py` | Same SIGINT double-shutdown fix as `takeoff.py`. |

### Performance note
LLVMpipe is CPU-rendered — expect ~5–15 fps in the Gazebo GUI on a 4-core VM.
Simulation physics and sensor data are unaffected by rendering frame rate.

---

## Phase 1 — Basic Drone Movement ✅ COMPLETE

**What was done**
- Created ROS2 Python package `drone_sim`
- Drone SDF model (`drone.sdf`) — box body, VelocityControl plugin
- Gazebo world (`rescue_world.sdf`) — ground + sun
- Control node `takeoff.py` — fake counter-based altitude, random movement

**Limitations at completion**
- Altitude was a Python counter, not a real measurement
- Movement was random with no real boundary logic
- No sensors, no pose feedback

---

## Phase 2 — Real Pose Feedback ✅ COMPLETE

**What was done**
- Added `gz-sim-pose-publisher-system` plugin to `drone.sdf`
  - Publishes `/model/quadrotor/pose` as `gz.msgs.Pose` at 30 Hz
- `takeoff.py` now subscribes to `/model/quadrotor/pose` (`geometry_msgs/msg/Pose`)
- Real X, Y, Z position and full quaternion orientation available to the controller
- Yaw extracted from quaternion using `atan2` formula

**Bridge command added**
```
/model/quadrotor/pose@geometry_msgs/msg/Pose[gz.msgs.Pose
```

**Files changed**
- `models/drone.sdf` — added PosePublisher plugin
- `drone_sim/takeoff.py` — full rewrite; added `_pose_callback`, removed fake counter

---

## Phase 3 — Altitude PID Controller ✅ COMPLETE

**What was done**
- Implemented `PID` class in `takeoff.py`
  - Kp = 1.5, Ki = 0.05, Kd = 0.8
  - Output clamped to ±3 m/s vertical velocity
  - Integral wind-up guard
- Drone targets `CRUISE_ALT = 8.0 m` using real Z from pose feedback
- `cmd.linear.z` is now PID output, not a hard-coded ramp

**Files changed**
- `drone_sim/takeoff.py` — added `PID` class, replaced altitude logic

---

## Phase 4 — Lawnmower Search Pattern ✅ COMPLETE

**What was done**
- Implemented `generate_lawnmower()` in `takeoff.py`
  - Grid: X ∈ [−20, 20], Y ∈ [−20, 20], lane spacing 5 m
  - Produces 18 waypoints covering the entire search area
- Waypoint navigator:
  - Computes heading error to next waypoint
  - Proportional yaw-rate control (turns to face waypoint)
  - Forward speed scales with alignment (`cos(yaw_err)`)
  - Small lateral correction to cancel drift
  - Waypoint accepted within 1.5 m radius
- Phase state machine: `wait_pose → takeoff → search → hover`

**Rescue world updated** (`rescue_world.sdf`)
- Added `gz-sim-sensors-system` and `gz-sim-imu-system` world plugins
- Environment objects added:
  | Object | Position |
  |---|---|
  | Collapsed building 1 | (10, 5, 1) |
  | Collapsed building 2 | (−8, 12, 0.5) |
  | Rubble pile 1 | (5, −8, 0.3) |
  | Rubble pile 2 | (−12, −6, 0.25) |
  | Car wreck | (−15, 8, 0.4) |
  | Victim 1 (red sphere) | (12, 3, 0.3) |
  | Victim 2 (red sphere) | (−6, 14, 0.3) |

**Files changed**
- `drone_sim/takeoff.py` — added `generate_lawnmower()`, navigator logic
- `worlds/rescue_world.sdf` — full rewrite with rescue environment

---

## Phase 5 — Camera Sensor Integration ✅ COMPLETE

**What was done**
- Added downward-facing camera sensor to `drone.sdf`
  - Resolution: 640×480, 30 Hz, 60° horizontal FOV
  - Mounted at bottom of drone, pitched −90° (looks at ground)
  - Gaussian noise added for realism
  - Topic: `/drone/camera`
- Added IMU sensor to `drone.sdf`
  - Update rate: 100 Hz
  - Gaussian noise on angular velocity and linear acceleration
  - Topic: `/drone/imu`
- Created `camera_viewer.py` node
  - Subscribes to `/drone/camera` and `/drone/camera_info`
  - Uses `cv_bridge` + OpenCV to decode and display frames
  - Falls back to log-only mode if no `DISPLAY` variable set
- Created unified launch file (`launch/drone_sim.launch.py`)
  - Single `ros2 launch` command replaces 5-terminal manual setup

**Bridges added**
```
/drone/camera@sensor_msgs/msg/Image[gz.msgs.Image
/drone/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo
/drone/imu@sensor_msgs/msg/Imu[gz.msgs.IMU
```

**`setup.py` updated**
- `models/`, `worlds/`, `launch/` installed to package share directory
- New entry point: `camera_viewer = drone_sim.camera_viewer:main`

**`package.xml` updated**
- Added: `sensor_msgs`, `cv_bridge`, `launch`, `launch_ros`, `ros_gz_bridge`

**Files created/changed**
- `drone_sim/camera_viewer.py` — new Phase 5 node
- `models/drone.sdf` — camera + IMU sensors added
- `launch/drone_sim.launch.py` — new unified launch file
- `setup.py` — data_files + entry points
- `package.xml` — dependencies

---

## How to Build and Run

### Build
```bash
cd ~/drone_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select drone_sim
source install/setup.bash
```

### Launch everything (single command)
```bash
ros2 launch drone_sim drone_sim.launch.py
```

### Or manually (5 terminals — legacy method)
```bash
# T1 — Gazebo
gz sim worlds/rescue_world.sdf -r

# T2 — Clock bridge
ros2 run ros_gz_bridge parameter_bridge \
  /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock

# T3 — cmd_vel bridge
ros2 run ros_gz_bridge parameter_bridge \
  /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist

# T4 — Pose bridge (Phase 2)
ros2 run ros_gz_bridge parameter_bridge \
  /model/quadrotor/pose@geometry_msgs/msg/Pose[gz.msgs.Pose

# T5 — Camera bridges (Phase 5)
ros2 run ros_gz_bridge parameter_bridge \
  /drone/camera@sensor_msgs/msg/Image[gz.msgs.Image \
  /drone/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo \
  /drone/imu@sensor_msgs/msg/Imu[gz.msgs.IMU

# T6 — Spawn drone (edit path as needed)
gz service -s /world/rescue_world/create \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 3000 \
  --req 'sdf_filename: "/path/to/drone.sdf", name: "quadrotor"'

# T7 — Controller
ros2 run drone_sim takeoff

# T8 — Camera viewer
ros2 run drone_sim camera_viewer
```

---

## Known Issues / TODOs

- [ ] **VirtualBox camera**: ogre2 renderer requires 3D acceleration.
  Enable in VM Settings → Display → Enable 3D Acceleration.
  Without it, camera sensor will silently produce no frames.
- [ ] **PID tuning**: Gains (Kp/Ki/Kd) may need adjustment after real
  simulation testing — depends on physics step size and model mass.
- [ ] **Drone model**: Still a simple box. Realistic quadrotor mesh and
  4-rotor physics are deferred to a future phase.
- [ ] **Body-frame vs world-frame velocities**: VelocityControl applies
  velocities in the body frame. If the drone tilts significantly, the
  altitude PID output (`linear.z`) will not be purely vertical.

---

## Phase 6 — Victim Detection (PLANNED)

Planned approach:
- OpenCV HSV colour-blob detection on `/drone/camera` to find the red victim spheres
- Publish detected victim world coordinates to `/drone/victims`
- Optional: upgrade to YOLO-based detection

---

## Phase 7 — Obstacle Avoidance (PLANNED)

Planned approach:
- Add Lidar sensor (`type="gpu_lidar"`) to `drone.sdf`
- Bridge `/drone/lidar` via `ros_gz_bridge`
- Implement reactive avoidance layer in the control loop

---

## Phase 8 — Multi-Drone Coordination (OPTIONAL)

Planned approach:
- Namespaced model spawning (`drone_1`, `drone_2`, …)
- Divide lawnmower grid among drones
- Shared victim map via a ROS2 topic
