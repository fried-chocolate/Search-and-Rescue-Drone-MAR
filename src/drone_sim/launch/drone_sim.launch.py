#!/usr/bin/env python3
"""
Launch file for the search-and-rescue drone simulation.

Starts (in order):
  1. Gazebo Harmonic with rescue_world.sdf
    2. All ros_gz_bridge bridges (clock, cmd_vel, pose, camera, IMU, lidar)
        3. Drone model spawn (after 3 s to let Gazebo load)
        4. Drone controller node (after 5 s to allow spawning)
        5. Camera viewer node (after 6 s, once simulation is fully running)

Usage after colcon build:
  ros2 launch drone_sim drone_sim.launch.py
"""

import os
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable, LogInfo
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory('drone_sim')
    world_file = os.path.join(pkg, 'worlds', 'rescue_world.sdf')
    model_file = os.path.join(pkg, 'models', 'drone.sdf')
    worlds_dir = os.path.join(pkg, 'worlds')
    # models_dir is where quadrotor/, hatchback_red/, person_standing/ live
    # Gazebo resolves model:// URIs by searching each dir in GZ_SIM_RESOURCE_PATH
    models_dir = os.path.join(pkg, 'models')

    # Prevent cross-talk with stale Gazebo servers by using a launch-unique partition.
    gz_partition = f'drone_sim_{os.getpid()}_{int(time.time())}'

    # ── 1. Gazebo ─────────────────────────────────────────────────────────────
    # -r = run immediately (no pause at startup)
    #
    # VirtualBox's VMSVGA3D driver exposes OpenGL 4.1 via Mesa but its GLSL
    # implementation is incomplete, causing Ogre2 to render a black window.
    #
    # LIBGL_ALWAYS_SOFTWARE=1      → Mesa uses llvmpipe for GLX (GUI window)
    # MESA_GL_VERSION_OVERRIDE=3.3 → tells Ogre2 OpenGL 3.3 is available
    #
    # NOTE: MESA_LOADER_DRIVER_OVERRIDE=llvmpipe and GALLIUM_DRIVER=llvmpipe
    # were tried but cause a segfault in driCreateNewScreen3 when Ogre2-Next
    # uses EGL_PLATFORM_DEVICE_EXT (explicit EGL device selection bypasses
    # the loader-level override).  The session-1 pair below is the only
    # combination that doesn't crash on this VirtualBox + Mesa 25.2.8 setup.
    #
    # GZ_SIM_RESOURCE_PATH lets Ogre2 find our procedural texture PNGs.
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        additional_env={
            'LIBGL_ALWAYS_SOFTWARE': '1',
            'MESA_GL_VERSION_OVERRIDE': '3.3',
            # EGL_PLATFORM=x11 forces Mesa's EGL to use the X11 display path
            # instead of EGL_PLATFORM_DEVICE_EXT (explicit GPU device).
            # On the X11 EGL path, LIBGL_ALWAYS_SOFTWARE=1 IS respected, so
            # llvmpipe handles sensor (EGL) rendering the same way it handles
            # the GUI (GLX) — fixing the all-black camera image on VirtualBox.
            'EGL_PLATFORM': 'x11',
            # Colon-separated: Gazebo searches each dir for model:// URIs and
            # resource files (textures, etc.)
            'GZ_SIM_RESOURCE_PATH': f'{worlds_dir}:{models_dir}',
            'GZ_PARTITION': gz_partition,
        },
        output='screen',
    )

    # ── 2. Topic bridges ──────────────────────────────────────────────────────
    # Clock — needed so ROS2 nodes can use sim time
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    # cmd_vel — bidirectional: ROS2 → Gazebo
    cmd_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='cmd_vel_bridge',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
        output='screen',
    )

    # Drone pose — Gazebo → ROS2 (Phase 2)
    pose_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='pose_bridge',
        arguments=[
            '/model/quadrotor/pose@geometry_msgs/msg/Pose[gz.msgs.Pose',
        ],
        output='screen',
    )

    # Camera image — Gazebo → ROS2 (Phase 5)
    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_bridge',
        arguments=[
            '/drone/camera@sensor_msgs/msg/Image[gz.msgs.Image',
        ],
        output='screen',
    )

    # Camera info — Gazebo → ROS2 (Phase 5)
    camera_info_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_info_bridge',
        arguments=[
            '/drone/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        output='screen',
    )

    # IMU — Gazebo → ROS2 (Phase 5)
    imu_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='imu_bridge',
        arguments=[
            '/drone/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
        output='screen',
    )

    # Lidar — Gazebo → ROS2 (Phase 7)
    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='lidar_bridge',
        arguments=[
            '/drone/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        output='screen',
    )

    # ── 3. Spawn drone (delayed 3 s) ──────────────────────────────────────────
    spawn_drone = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'gz', 'service',
                    '-s', '/world/rescue_world/create',
                    '--reqtype', 'gz.msgs.EntityFactory',
                    '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '8000',
                    '--req',
                    f'sdf_filename: "{model_file}", name: "quadrotor"',
                ],
                output='screen',
            ),
        ],
    )

    # ── 4. Drone controller (delayed 5 s) ─────────────────────────────────────
    drone_controller = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='drone_sim',
                executable='takeoff',
                name='drone_control',
                output='screen',
            ),
        ],
    )

    # ── 5. Camera viewer (delayed 6 s) ────────────────────────────────────────
    camera_viewer = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='drone_sim',
                executable='camera_viewer',
                name='camera_viewer',
                output='screen',
            ),
        ],
    )

    return LaunchDescription([
        SetEnvironmentVariable('GZ_PARTITION', gz_partition),
        LogInfo(msg=f'Using GZ_PARTITION={gz_partition}'),
        gazebo,
        clock_bridge,
        cmd_vel_bridge,
        pose_bridge,
        camera_bridge,
        camera_info_bridge,
        imu_bridge,
        lidar_bridge,
        spawn_drone,
        drone_controller,
        camera_viewer,
    ])
