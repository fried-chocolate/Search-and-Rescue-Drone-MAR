# Search-and-Rescue Drone Simulation — Progress Track

> Last updated: **2026-04-12** (repository-wide audit)
> Stack: ROS2 Jazzy · Gazebo Harmonic · Python · SDF

## Completion Snapshot (codebase audit)

- ✅ **Completed**: Model aesthetics, launch orchestration, pose feedback, altitude PID,
  lawnmower navigation, camera + IMU integration, camera HUD/mini-map viewer
- 🟡 **In progress / tuning**: PID gains, long-run flight robustness
- ⏳ **Not started**: Victim detection (Phase 6), obstacle avoidance (Phase 7),
  multi-drone coordination (Phase 8)

Approx. project completion against current roadmap:
- Core simulation + autonomy baseline (Phases 1–5): **100%**
- End-to-end SAR features (Phases 1–8): **~62%**

---

## Model Aesthetics Upgrade ✅ COMPLETE

### Problem
All scene objects rendered as plain coloured boxes: the drone looked like a floating
blue rectangle; the car wreck was a dark-red slab; the victims were featureless
spheres. This is unrealistic for a rescue simulation.

### Solution — Real 3D mesh assets

**Drone (quadrotor mesh)**
- Source: [`osrf/gazebo_models`](https://github.com/osrf/gazebo_models/tree/master/quadrotor)
  — `quadrotor_base.dae` (Stefan Kohlbrecher, COLLADA format)
- Placed in `models/quadrotor/meshes/quadrotor_base.dae`
- `drone.sdf` visual replaced: `<uri>model://quadrotor/meshes/quadrotor_base.dae</uri>`
- Inertia updated to osrf values: mass=1.316 kg, ixx=iyy=0.0128, izz=0.0218
- Box collision **kept** (mesh collision is expensive; physics is identical)

**Car wreck (Hatchback Red)**
- Source: Gazebo Fuel — `OpenRobotics/Hatchback Red` (OBJ + MTL + PNG texture)
- Scale: 0.0254 (inches → metres), 90° yaw rotation applied in model.sdf
- `rescue_world.sdf` car_wreck replaced with:
  `<include><uri>model://hatchback_red</uri><pose>-15 8 0.0 0 0 0.8</pose></include>`

**Victims (Standing person)**
- Source: Gazebo Fuel — `OpenRobotics/Standing person` (DAE + 11 skin/clothing PNGs)
- Fixed model.sdf to use local `model://` URI instead of Fuel HTTPS download URL
- `rescue_world.sdf` victim_1/2 red spheres replaced with `<include>` tags, each
  given a different yaw angle so they appear to face different directions

### New model directory structure
```
src/drone_sim/models/
  quadrotor/
    model.config
    meshes/quadrotor_base.dae
  hatchback_red/
    model.config  model.sdf
    meshes/hatchback.obj  hatchback.mtl
    materials/textures/hatchback.png
  person_standing/
    model.config  model.sdf
    meshes/standing.dae
    materials/textures/*.png  (11 files: skin, jeans, shirt, eyes, teeth, …)
```

### GZ_SIM_RESOURCE_PATH updated
Previously only pointed to `worlds/`. Now set to `worlds_dir:models_dir` so Gazebo
can resolve all `model://` URIs at runtime:
- `model://quadrotor` → drone COLLADA mesh
- `model://hatchback_red` → car OBJ
- `model://person_standing` → victim DAE

### setup.py
Added explicit `data_files` entries for every mesh and texture subfolder so
`colcon build` installs everything into `share/drone_sim/models/`.

**Files changed**
- `models/drone.sdf` — mesh visual, updated inertia
- `worlds/rescue_world.sdf` — 3× `<include>` tags replace box models
- `launch/drone_sim.launch.py` — `GZ_SIM_RESOURCE_PATH` extended
- `setup.py` — 6 new `data_files` entries for mesh model dirs
- **New directories**: `models/quadrotor/`, `models/hatchback_red/`, `models/person_standing/`

**Git commit**: `feat: replace box geometry with real 3D meshes (quadrotor DAE, hatchback OBJ, standing person DAE)`

---

## Bugfix Session 5 — Black camera on VirtualBox EGL path ✅ IMPLEMENTED IN LAUNCH

### Problem
Even after forcing software GL for the GUI, the **camera sensor** could still go black
because it renders through EGL, not GLX.

### Current launch-side fix
`launch/drone_sim.launch.py` now exports:

```python
'EGL_PLATFORM': 'x11'
```

with:

```python
'LIBGL_ALWAYS_SOFTWARE': '1'
'MESA_GL_VERSION_OVERRIDE': '3.3'
```

This keeps both GUI and sensor rendering on a software-compatible path in this setup.

### Files changed
- `launch/drone_sim.launch.py` — added `EGL_PLATFORM=x11`

---

## Bugfix Session 4 — gzserver SIGKILL / "taking too long to respond" ✅ FIXED

### Problem
After Bugfix Session 2, Gazebo was being killed by the OS with SIGKILL approximately
8 seconds after launch: `gzserver: taking too long to respond`. This made the world
impossible to load.

### Root cause
Two features added in Session 2 triggered **Ogre2 compute-shader compilation** at
startup, which hangs indefinitely on software GL (LLVMpipe has no GPU compute):
1. `<sky><clouds>` SDF tag — uses an Ogre2 procedural cloud compute shader
2. PBR `<albedo_map>` texture on the ground plane — triggers the PBR shader pipeline

The kernel-level OOM killer / Gazebo's own watchdog sent SIGKILL after the 5-second
(later 8-second) startup timeout.

### Fix applied

| File | Change |
|---|---|
| `worlds/rescue_world.sdf` | Removed `<sky><clouds>` block entirely |
| `worlds/rescue_world.sdf` | Removed `<pbr><metal><albedo_map>` from ground plane; reverted to flat RGBA `<material>` |
| `launch/drone_sim.launch.py` | Increased spawn delay 5s→8s, Gazebo timeout 5000ms→8000ms |

Flat `<ambient>/<diffuse>` colours use the **Legacy (Phong)** shader path — no
compute-shader compilation, safe on software GL.

**Files changed**
- `worlds/rescue_world.sdf` — removed sky/clouds, reverted to flat material
- `launch/drone_sim.launch.py` — longer timeouts

---

## Bugfix Session 3 — Segfault in driCreateNewScreen3 ✅ FIXED

### Problem
After Bugfix Session 2 added `MESA_LOADER_DRIVER_OVERRIDE=llvmpipe` and
`GALLIUM_DRIVER=llvmpipe`, Gazebo crashed immediately on startup with a segfault
inside `driCreateNewScreen3` in the Mesa EGL stack.

### Root cause
Ogre2-Next (Gazebo Harmonic's renderer) uses **EGL_PLATFORM_DEVICE_EXT** to
explicitly enumerate and select the EGL device. This device-selection path
**bypasses** the DRI loader-level override (`MESA_LOADER_DRIVER_OVERRIDE`).
When the loader-level override is applied after EGL has already locked onto a
hardware device, the two driver paths conflict and segfault.

### Fix applied
Reverted to the Session-1 env var set — only two variables:

```python
'LIBGL_ALWAYS_SOFTWARE': '1',       # forces llvmpipe for GLX (GUI window)
'MESA_GL_VERSION_OVERRIDE': '3.3',  # tells Ogre2 that OpenGL 3.3 is available
```

`MESA_LOADER_DRIVER_OVERRIDE`, `GALLIUM_DRIVER`, and `MESA_GLSL_VERSION_OVERRIDE`
**removed** — they cause the crash.

**Status after fix**: Gazebo GUI renders correctly via LLVMpipe. Camera EGL path
still uses hardware (VirtualBox VMSVGA3D) and produces black frames — unavoidable
without a real GPU.

**Files changed**
- `launch/drone_sim.launch.py` — removed 3 broken env vars, kept 2 stable ones

---



### Camera black-frame follow-up
Earlier loader-level overrides (`MESA_LOADER_DRIVER_OVERRIDE`, `GALLIUM_DRIVER`)
were unstable in this environment and caused EGL/Mesa crashes. The current launch
configuration instead uses `EGL_PLATFORM=x11` + software GL vars.

### Textures
No textures ship with Gazebo Harmonic. Generated procedurally with **Python PIL**:
- `worlds/textures/ground.png` (1024×1024) — sandy disaster-zone terrain with
  dark ash patches and crack lines. *(Asset exists; not currently bound in world SDF,
  which uses flat legacy material for compatibility.)*
- `worlds/textures/concrete.png` (512×512) — gray concrete tiles for buildings.
- `GZ_SIM_RESOURCE_PATH` set to the installed `worlds/` directory in the launch file
  so Ogre2 can find the textures.

### Navigation clarity
| Parameter | Before | After |
|---|---|---|
| Grid size | ±20 m | ±13 m (matches obstacle area) |
| Lane step | 5 m | 3.5 m (denser coverage) |
| Max speed | 3.0 m/s | 2.5 m/s |
| WP radius | 1.5 m | 1.2 m |
| Speed near WP | constant | ramps down linearly in final 4 m |
| Position log | none | every 2 s: x/y/z/yaw/phase/wp/dist |

The `[NAV]` log line now printed every 2 seconds shows exactly where the drone is,
which direction it's heading, and how far it is from the next waypoint.

---


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
| `worlds/rescue_world.sdf` | Added `<scene>` block with ambient light and sky-blue background so Ogre2 always has a valid scene to initialize against. |
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
  - Grid: X ∈ [−13, 13], Y ∈ [−13, 13], lane spacing 3.5 m
  - Produces 18 waypoints covering the entire search area
- Waypoint navigator:
  - Computes heading error to next waypoint
  - Proportional yaw-rate control (turns to face waypoint)
  - Forward speed scales with alignment (`cos(yaw_err)`)
  - Small lateral correction to cancel drift
  - Waypoint accepted within 1.2 m radius
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
cd ~/mar-mini-project
source /opt/ros/jazzy/setup.bash
colcon build --packages-select drone_sim
source install/setup.bash
```

### Launch everything (single command)
```bash
ros2 launch drone_sim drone_sim.launch.py
```

### Or manually (8 terminals — legacy method)
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

- [ ] **Rendering portability**: camera output depends on host graphics stack.
  Current launch defaults (`EGL_PLATFORM=x11`, software GL env) are tuned for this
  VM setup and may require adjustment on other machines.
- [ ] **PID tuning**: Gains (Kp/Ki/Kd) may need adjustment after real
  simulation testing — depends on physics step size and model mass.
- [ ] **Body-frame vs world-frame velocities**: VelocityControl applies
  velocities in the body frame. If the drone tilts significantly, the
  altitude PID output (`linear.z`) will not be purely vertical.
- [ ] **Metadata cleanup**: `package.xml` still contains placeholder
  description/license fields (`TODO`).

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
