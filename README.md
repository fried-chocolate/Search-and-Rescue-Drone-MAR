# Autonomous Search and Rescue Drone Simulation

A ROS 2 + Gazebo Harmonic mini project for Mobile and Autonomous Robotics (MAR). A quadrotor drone autonomously searches a simulated disaster zone, detects survivors, avoids obstacles, and logs mission events.

## Features

- Altitude hold via PID controller targeting 5.2 m cruise altitude
- Lawnmower (boustrophedon) search pattern over a 26x26 m grid
- Downward camera with colour segmentation based victim detection
- Forward LiDAR (720 beam, 180 FOV) with reactive obstacle avoidance
- Spatial deduplication and hover hold on victim confirmation
- Realistic 3D scene: quadrotor mesh, hatchback car wreck, textured standing person models
- Single launch command starts Gazebo, all bridges, controller and camera viewer

## Project Structure

```
Search-and-Rescue-Drone-MAR/
├── full_run.sh                     # Build + launch (first run)
├── quick_run.sh                    # Launch only (after first build)
└── src/drone_sim/
    ├── drone_sim/
    │   ├── takeoff.py              # Main controller node
    │   └── camera_viewer.py        # Camera HUD and victim detection node
    ├── launch/
    │   └── drone_sim.launch.py
    ├── models/
    │   ├── drone.sdf               # Quadrotor SDF with camera, IMU and LiDAR
    │   ├── quadrotor/              # Quadrotor mesh (OSRF)
    │   ├── hatchback_red/          # Car wreck model (Fuel)
    │   └── person_standing/        # Victim model (Fuel)
    └── worlds/
        └── rescue_world.sdf
```

## Prerequisites

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic

```bash
sudo apt install -y \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-sim \
  python3-colcon-common-extensions \
  python3-opencv \
  ros-jazzy-cv-bridge
```

## Running

First run (builds and launches):
```bash
chmod +x full_run.sh
./full_run.sh
```

Subsequent runs:
```bash
./quick_run.sh
```

Manual:
```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select drone_sim
source install/setup.bash
ros2 launch drone_sim drone_sim.launch.py
```

## ROS 2 Topics

| Topic | Type | Direction |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | ROS to Gazebo |
| `/model/quadrotor/pose` | `geometry_msgs/Pose` | Gazebo to ROS |
| `/drone/camera` | `sensor_msgs/Image` | Gazebo to ROS |
| `/drone/imu` | `sensor_msgs/Imu` | Gazebo to ROS |
| `/drone/lidar` | `sensor_msgs/LaserScan` | Gazebo to ROS |
| `/drone/victims` | `geometry_msgs/PoseArray` | camera_viewer to takeoff |

## Flight Phases

`wait_pose` → `takeoff` → `search` → `hover`

1. Waits for first valid pose from Gazebo
2. PID climbs to cruise altitude
3. Executes lawnmower waypoints with live obstacle avoidance; hovers briefly on each victim confirmation
4. Holds position after both victims are confirmed

## Configuration

Key constants in `takeoff.py`:

```python
CRUISE_ALT              = 5.2    # metres
GRID_LANE_STEP          = 3.5    # metres
MAX_SPEED               = 2.5    # m/s
WP_ACCEPT_RADIUS        = 1.2    # metres
VICTIM_CONFIRM_RADIUS   = 3.0    # metres
TARGET_VICTIMS_FOR_DEMO = 2
PRESENTATION_MODE       = True   # short direct route for demos
```

## Software Rendering Note

Developed on VirtualBox with Mesa LLVMpipe. The launch file sets these automatically:

```python
LIBGL_ALWAYS_SOFTWARE    = '1'
MESA_GL_VERSION_OVERRIDE = '3.3'
EGL_PLATFORM             = 'x11'
```

On a machine with a real GPU, these can be removed from `launch/drone_sim.launch.py`.

## Progress

| Phase | Feature | Status |
|---|---|---|
| 1 | Basic drone movement | Done |
| 2 | Real pose feedback | Done |
| 3 | Altitude PID | Done |
| 4 | Lawnmower search pattern | Done |
| 5 | Camera and IMU integration | Done |
| 6 | Victim detection | Done |
| 7 | LiDAR obstacle avoidance | Done |
| 8 | Multi-drone coordination | Not started (optional) |
| 9 | Mesh-aware victim detection tuning | Done |
| 10 | Victim-found hover behaviour | Done |

## Credits

3D model assets: quadrotor mesh by Stefan Kohlbrecher / OSRF, Hatchback Red and Standing Person by OpenRobotics via Gazebo Fuel.