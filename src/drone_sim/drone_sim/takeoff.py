#!/usr/bin/env python3
"""
Drone autonomous control node.

Phase 2 — Real pose feedback  : subscribes to /model/quadrotor/pose
Phase 3 — Altitude PID        : holds cruise altitude with a PID controller
Phase 4 — Lawnmower search    : navigates a boustrophedon (zigzag) grid pattern
"""

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose


# ── PID Controller ────────────────────────────────────────────────────────────

class PID:
    """Simple PID controller with output clamping and integral wind-up guard."""

    def __init__(self, kp: float, ki: float, kd: float,
                 output_min: float = -5.0, output_max: float = 5.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.output_min = output_min
        self.output_max = output_max
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time: float | None = None

    def compute(self, setpoint: float, measurement: float) -> float:
        now = time.monotonic()
        dt = (now - self._prev_time) if self._prev_time is not None else 0.05
        self._prev_time = now
        dt = max(dt, 1e-6)

        error = setpoint - measurement
        self._integral = max(self.output_min,
                             min(self.output_max,
                                 self._integral + error * dt))
        derivative = (error - self._prev_error) / dt
        self._prev_error = error

        output = (self.kp * error
                  + self.ki * self._integral
                  + self.kd * derivative)
        return max(self.output_min, min(self.output_max, output))

    def reset(self) -> None:
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = None


# ── Helpers ───────────────────────────────────────────────────────────────────

def quaternion_to_yaw(q) -> float:
    """Extract yaw angle (radians) from a geometry_msgs quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalise_angle(angle: float) -> float:
    """Clamp an angle to [-π, π]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def generate_lawnmower(x_min: float, x_max: float,
                       y_min: float, y_max: float,
                       lane_step: float, altitude: float) -> list:
    """
    Generate a boustrophedon (lawnmower) coverage path.

    Sweeps parallel east-west lanes spaced `lane_step` metres apart.
    Returns a list of (x, y, z) tuples.
    """
    waypoints: list[tuple[float, float, float]] = []
    y = y_min
    left_to_right = True
    while y <= y_max + 1e-3:
        if left_to_right:
            waypoints.append((x_min, y, altitude))
            waypoints.append((x_max, y, altitude))
        else:
            waypoints.append((x_max, y, altitude))
            waypoints.append((x_min, y, altitude))
        y += lane_step
        left_to_right = not left_to_right
    return waypoints


# ── Parameters ────────────────────────────────────────────────────────────────

CRUISE_ALT     = 8.0    # metres — target cruising altitude
TAKEOFF_TOL    = 0.5    # metres — altitude tolerance to leave takeoff phase
WP_RADIUS      = 1.5    # metres — "waypoint reached" acceptance radius
MAX_XY_SPEED   = 3.0    # m/s    — horizontal speed cap
MAX_YAW_RATE   = 1.2    # rad/s  — yaw rate cap
CTRL_HZ        = 10.0   # Hz     — control-loop frequency

# Lawnmower search grid (metres, world frame)
GRID_X_MIN,  GRID_X_MAX  = -20.0, 20.0
GRID_Y_MIN,  GRID_Y_MAX  = -20.0, 20.0
GRID_LANE_STEP            =   5.0


# ── Main Control Node ─────────────────────────────────────────────────────────

class DroneControl(Node):

    def __init__(self):
        super().__init__('drone_control')

        # ── Publishers / subscribers ─────────────────────────────────────────
        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Phase 2: real pose feedback from Gazebo via ros_gz_bridge
        self._pose_sub = self.create_subscription(
            Pose,
            '/model/quadrotor/pose',
            self._pose_callback,
            10,
        )

        # ── State ────────────────────────────────────────────────────────────
        self._pose: Pose | None = None
        self._phase = 'wait_pose'   # wait_pose → takeoff → search → hover

        # Phase 4: pre-generate the full lawnmower waypoint list
        self._waypoints = generate_lawnmower(
            GRID_X_MIN, GRID_X_MAX,
            GRID_Y_MIN, GRID_Y_MAX,
            GRID_LANE_STEP, CRUISE_ALT,
        )
        self._wp_index = 0

        # Phase 3: altitude PID (Kp=1.5, Ki=0.05, Kd=0.8)
        self._alt_pid = PID(kp=1.5, ki=0.05, kd=0.8,
                            output_min=-3.0, output_max=3.0)

        # ── Timer ────────────────────────────────────────────────────────────
        self.create_timer(1.0 / CTRL_HZ, self._control_loop)

        self.get_logger().info(
            f'DroneControl started — {len(self._waypoints)} waypoints queued.'
            ' Waiting for first pose message…'
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _pose_callback(self, msg: Pose) -> None:
        """Phase 2: store latest real pose from Gazebo."""
        self._pose = msg
        if self._phase == 'wait_pose':
            self._phase = 'takeoff'
            self.get_logger().info('Pose received — beginning takeoff.')

    # ── Control loop ──────────────────────────────────────────────────────────

    def _control_loop(self) -> None:
        cmd = Twist()

        # ----- WAIT FOR POSE --------------------------------------------------
        if self._phase == 'wait_pose':
            self._pub.publish(cmd)
            return

        # Extract current position / heading from real pose (Phase 2)
        x   = self._pose.position.x
        y   = self._pose.position.y
        z   = self._pose.position.z
        yaw = quaternion_to_yaw(self._pose.orientation)

        # Phase 3: altitude PID is active during every flying phase
        cmd.linear.z = self._alt_pid.compute(CRUISE_ALT, z)

        # ----- TAKEOFF --------------------------------------------------------
        if self._phase == 'takeoff':
            if abs(z - CRUISE_ALT) < TAKEOFF_TOL:
                self._phase = 'search'
                self.get_logger().info(
                    f'Takeoff complete (z = {z:.2f} m). Starting search pattern.'
                )
            # Climb in place — no horizontal movement yet
            self._pub.publish(cmd)
            return

        # ----- SEARCH (lawnmower) ---------------------------------------------
        if self._phase == 'search':
            if self._wp_index >= len(self._waypoints):
                self._phase = 'hover'
                self.get_logger().info(
                    'Search pattern complete — switching to hover.'
                )
                self._pub.publish(cmd)
                return

            wx, wy, _ = self._waypoints[self._wp_index]
            dx = wx - x
            dy = wy - y
            dist_xy = math.hypot(dx, dy)

            # Waypoint reached?
            if dist_xy < WP_RADIUS:
                self._wp_index += 1
                self.get_logger().info(
                    f'Waypoint {self._wp_index}/{len(self._waypoints)} reached '
                    f'(x={wx:.1f}, y={wy:.1f}).'
                )
                self._pub.publish(cmd)
                return

            # Heading error toward waypoint
            desired_yaw = math.atan2(dy, dx)
            yaw_err = normalise_angle(desired_yaw - yaw)

            # Yaw-rate command (proportional)
            cmd.angular.z = max(-MAX_YAW_RATE,
                                min(MAX_YAW_RATE, 1.5 * yaw_err))

            # Forward speed — scale down when misaligned to avoid overshoot
            alignment = math.cos(yaw_err)           # 1 = perfect, -1 = opposite
            forward   = max(0.0, alignment) * min(MAX_XY_SPEED, dist_xy)
            cmd.linear.x = forward

            # Small lateral correction to cancel crosswind / drift
            cmd.linear.y = -math.sin(yaw_err) * 0.5

            self._pub.publish(cmd)
            return

        # ----- HOVER ----------------------------------------------------------
        if self._phase == 'hover':
            # Maintain altitude only; no horizontal movement
            self._pub.publish(cmd)
            return

        self._pub.publish(cmd)


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = DroneControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
