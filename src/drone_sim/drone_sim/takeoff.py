#!/usr/bin/env python3
"""
Drone autonomous control node.

Phase 2 — Real pose feedback  : subscribes to /model/quadrotor/pose
Phase 3 — Altitude PID        : holds cruise altitude with a PID controller
Phase 4 — Lawnmower search    : navigates a boustrophedon (zigzag) grid pattern
Phase 7 — Obstacle avoidance  : reactive lidar-based avoidance layer
Phase 10 — Victim event mode  : hover-hold + mission logs on victim confirmation
Presentation mode             : short, victim-focused route for fast demos
"""

import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, Pose, PoseArray
from sensor_msgs.msg import LaserScan


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


def generate_presentation_route(altitude: float) -> list[tuple[float, float, float]]:
    """
    Short route that visits both victim zones quickly for presentation demos.

    The route starts near spawn and reaches V1 then V2 with minimal detours.
    """
    points = [
        (12.0, 1.0),   # victim zone V1
        (-4.0, 14.0),  # victim zone V2
    ]
    return [(x, y, altitude) for x, y in points]


# ── Parameters ────────────────────────────────────────────────────────────────

CRUISE_ALT     = 5.2    # metres — lower cruise altitude for faster demo startup
TAKEOFF_TOL    = 0.4    # metres — altitude tolerance to leave takeoff phase
WP_RADIUS      = 2.2    # metres — "waypoint reached" acceptance radius
MAX_XY_SPEED   = 4.3    # m/s    — horizontal speed cap
MAX_YAW_RATE   = 1.9    # rad/s  — yaw rate cap
CTRL_HZ        = 10.0   # Hz     — control-loop frequency
DEFAULT_RUN_MODE = 'quick'
QUICK_MODE_ALIASES = {'quick', 'demo', 'presentation', 'fast'}
FULL_MODE_ALIASES = {'full', 'coverage', 'lawnmower'}
TARGET_VICTIMS_FOR_DEMO = 2
QUICK_STOP_ON_DEMO_COMPLETE = False

# Lawnmower search grid (metres, world frame)
# Kept to ±13 m to match the obstacle/victim placement area.
GRID_X_MIN,  GRID_X_MAX  = -13.0, 13.0
GRID_Y_MIN,  GRID_Y_MAX  = -13.0, 13.0
GRID_LANE_STEP            =   3.5

# Phase 7: reactive obstacle avoidance thresholds (metres)
LIDAR_STOP_DIST   = 2.2
LIDAR_CAUTION_DIST = 4.5
LIDAR_SIDE_DIST   = 1.5
AVOID_LOG_PERIOD  = 1.5   # seconds

# Phase 10: victim confirmation and event-hold behavior
VICTIM_CONFIRM_RADIUS = 3.0   # metres: merge duplicate detections nearby
VICTIM_HOLD_SEC = 1.0         # seconds: brief hold for presentation cue
VICTIM_HOLD_LOG_PERIOD = 1.0  # seconds
VICTIM_HINTS = [(12.0, 1.0), (-4.0, 14.0)]
VICTIM_HINT_ACCEPT_RADIUS = 4.5


# ── Main Control Node ─────────────────────────────────────────────────────────

class DroneControl(Node):

    def __init__(self):
        super().__init__('drone_control')

        raw_run_mode = os.environ.get('SAR_RUN_MODE', DEFAULT_RUN_MODE).strip().lower()
        if raw_run_mode in FULL_MODE_ALIASES:
            self._run_mode = 'full'
            self._presentation_mode = False
        elif raw_run_mode in QUICK_MODE_ALIASES or raw_run_mode == '':
            self._run_mode = 'quick'
            self._presentation_mode = True
        else:
            self._run_mode = 'quick'
            self._presentation_mode = True
            self.get_logger().warn(
                f'Unknown SAR_RUN_MODE="{raw_run_mode}"; defaulting to quick mode.'
            )

        # ── Publishers / subscribers ─────────────────────────────────────────
        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Phase 2: real pose feedback from Gazebo via ros_gz_bridge
        self._pose_sub = self.create_subscription(
            Pose,
            '/model/quadrotor/pose',
            self._pose_callback,
            10,
        )

        # Phase 7: forward lidar feed for reactive obstacle avoidance
        self._lidar_sub = self.create_subscription(
            LaserScan,
            '/drone/lidar',
            self._lidar_callback,
            qos_profile_sensor_data,
        )

        # Phase 10: victim detections from camera node.
        self._victim_sub = self.create_subscription(
            PoseArray,
            '/drone/victims',
            self._victim_callback,
            10,
        )

        # ── State ────────────────────────────────────────────────────────────
        self._pose: Pose | None = None
        self._phase = 'wait_pose'   # wait_pose → takeoff → search → hover

        # Latest lidar scan cache (Phase 7)
        self._scan_ranges: list[float] = []
        self._scan_angle_min = 0.0
        self._scan_angle_increment = 0.0
        self._scan_range_min = 0.0
        self._scan_range_max = 0.0
        self._lidar_online = False
        self._last_avoid_log = 0.0

        # Phase 10 state
        self._confirmed_victims: list[tuple[float, float]] = []
        self._victim_hold_until = 0.0
        self._last_hold_log = 0.0
        self._mission_complete_announced = False

        # Phase 4: pre-generate the full lawnmower waypoint list
        if self._presentation_mode:
            self._waypoints = generate_presentation_route(CRUISE_ALT)
        else:
            self._waypoints = generate_lawnmower(
                GRID_X_MIN, GRID_X_MAX,
                GRID_Y_MIN, GRID_Y_MAX,
                GRID_LANE_STEP, CRUISE_ALT,
            )
        self._wp_index = 0

        # Phase 3: altitude PID (Kp=1.5, Ki=0.05, Kd=0.8)
        self._alt_pid = PID(kp=1.5, ki=0.05, kd=0.8,
                            output_min=-3.0, output_max=3.0)

        # Loop counter for periodic position logging (every 2 s at 10 Hz)
        self._log_counter = 0

        # ── Timer ────────────────────────────────────────────────────────────
        self.create_timer(1.0 / CTRL_HZ, self._control_loop)

        mode = 'presentation-fast' if self._presentation_mode else 'coverage-lawnmower'
        self.get_logger().info(
            f'DroneControl started ({mode}, run_mode={self._run_mode}) '
            f'— {len(self._waypoints)} waypoints queued.'
            ' Waiting for first pose message…'
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _pose_callback(self, msg: Pose) -> None:
        """Phase 2: store latest real pose from Gazebo."""
        self._pose = msg
        if self._phase == 'wait_pose':
            self._phase = 'takeoff'
            self.get_logger().info('Pose received — beginning takeoff.')

    def _lidar_callback(self, msg: LaserScan) -> None:
        """Phase 7: cache latest lidar scan for obstacle-avoidance checks."""
        self._scan_ranges = list(msg.ranges)
        self._scan_angle_min = msg.angle_min
        self._scan_angle_increment = msg.angle_increment
        self._scan_range_min = msg.range_min
        self._scan_range_max = msg.range_max

        if not self._lidar_online:
            self._lidar_online = True
            fov = math.degrees(msg.angle_max - msg.angle_min)
            self.get_logger().info(
                f'Lidar online — {len(msg.ranges)} beams, FOV={fov:.0f}°.'
            )

    def _victim_callback(self, msg: PoseArray) -> None:
        """Phase 10: confirm new victims and trigger event-hold behavior."""
        if self._phase != 'search':
            return

        if not msg.poses:
            return

        new_hits: list[tuple[float, float]] = []
        for pose in msg.poses:
            vx = pose.position.x
            vy = pose.position.y
            if not (math.isfinite(vx) and math.isfinite(vy)):
                continue

            # Demo scenario gating: confirm only victims within expected zones.
            in_expected_zone = any(
                math.hypot(vx - hx, vy - hy) <= VICTIM_HINT_ACCEPT_RADIUS
                for hx, hy in VICTIM_HINTS
            )
            if not in_expected_zone:
                continue

            duplicate = any(
                math.hypot(vx - cx, vy - cy) < VICTIM_CONFIRM_RADIUS
                for cx, cy in self._confirmed_victims
            )
            if duplicate:
                continue

            self._confirmed_victims.append((vx, vy))
            new_hits.append((vx, vy))

        if not new_hits:
            return

        self._victim_hold_until = max(
            self._victim_hold_until,
            time.monotonic() + VICTIM_HOLD_SEC,
        )

        start_idx = len(self._confirmed_victims) - len(new_hits) + 1
        for idx, (vx, vy) in enumerate(new_hits, start=start_idx):
            self.get_logger().warn(
                f'[MISSION] VICTIM FOUND #{idx} at ({vx:+.1f}, {vy:+.1f}) m '
                f'— hold for {VICTIM_HOLD_SEC:.0f}s.'
            )

    def _sector_min(self, start_deg: float, end_deg: float) -> float:
        """Return minimum valid range in the requested angular sector."""
        if not self._scan_ranges or abs(self._scan_angle_increment) < 1e-9:
            return float('inf')

        start = math.radians(start_deg)
        end = math.radians(end_deg)
        angle = self._scan_angle_min
        min_dist = float('inf')

        for rng in self._scan_ranges:
            if start <= angle <= end:
                if math.isfinite(rng) and self._scan_range_min < rng < self._scan_range_max:
                    min_dist = min(min_dist, rng)
            angle += self._scan_angle_increment

        return min_dist

    def _log_avoid(self, mode: str,
                   front: float, left: float, right: float) -> None:
        """Rate-limited obstacle-avoidance telemetry."""
        now = time.monotonic()
        if now - self._last_avoid_log < AVOID_LOG_PERIOD:
            return
        self._last_avoid_log = now

        def fmt(val: float) -> str:
            return f'{val:.2f}' if math.isfinite(val) else 'inf'

        self.get_logger().info(
            f'[AVOID:{mode}] front={fmt(front)}m '
            f'left={fmt(left)}m right={fmt(right)}m'
        )

    def _apply_obstacle_avoidance(self, cmd: Twist) -> None:
        """Phase 7: blend reactive avoidance into navigation commands."""
        if not self._scan_ranges:
            return

        front_min = self._sector_min(-20.0, 20.0)
        left_min = self._sector_min(20.0, 100.0)
        right_min = self._sector_min(-100.0, -20.0)

        # Hard block: stop forward motion and strafe/turn toward clearer side.
        if front_min < LIDAR_STOP_DIST:
            turn_sign = 1.0 if left_min >= right_min else -1.0
            cmd.linear.x = 0.0
            cmd.linear.y = 0.9 * turn_sign
            cmd.angular.z = MAX_YAW_RATE * turn_sign
            self._log_avoid('hard', front_min, left_min, right_min)
            return

        # Caution zone: reduce forward speed and bias heading away from clutter.
        if front_min < LIDAR_CAUTION_DIST:
            span = max(0.1, LIDAR_CAUTION_DIST - LIDAR_STOP_DIST)
            scale = max(0.15, min(1.0, (front_min - LIDAR_STOP_DIST) / span))
            cmd.linear.x *= scale

            if left_min < right_min:
                cmd.angular.z -= 0.6
            elif right_min < left_min:
                cmd.angular.z += 0.6

        # Side clearance nudges.
        if left_min < LIDAR_SIDE_DIST and right_min >= left_min:
            cmd.linear.y -= 0.5
        if right_min < LIDAR_SIDE_DIST and left_min > right_min:
            cmd.linear.y += 0.5

        cmd.linear.y = max(-1.2, min(1.2, cmd.linear.y))
        cmd.angular.z = max(-MAX_YAW_RATE, min(MAX_YAW_RATE, cmd.angular.z))

        if (front_min < LIDAR_CAUTION_DIST
                or left_min < LIDAR_SIDE_DIST
                or right_min < LIDAR_SIDE_DIST):
            self._log_avoid('caution', front_min, left_min, right_min)

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

        # Periodic position log (every 20 loops = 2 s)
        self._log_counter += 1
        if self._log_counter % 20 == 0:
            wp_str = '—'
            if self._phase == 'search' and self._wp_index < len(self._waypoints):
                wx, wy, _ = self._waypoints[self._wp_index]
                dist_to_wp = math.hypot(wx - x, wy - y)
                wp_str = (f'wp {self._wp_index+1}/{len(self._waypoints)} '
                          f'→ ({wx:.0f},{wy:.0f}) dist={dist_to_wp:.1f}m')
            self.get_logger().info(
                f'[NAV] x={x:+.1f}m y={y:+.1f}m z={z:.1f}m '
                f'yaw={math.degrees(yaw):+.0f}° | {self._phase} '
                f'| victims={len(self._confirmed_victims)} | {wp_str}'
            )

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
            now = time.monotonic()

            # Phase 10: pause and hover briefly after new victim confirmation.
            if now < self._victim_hold_until:
                cmd.linear.x = 0.0
                cmd.linear.y = 0.0
                cmd.angular.z = 0.0

                if now - self._last_hold_log >= VICTIM_HOLD_LOG_PERIOD:
                    self._last_hold_log = now
                    remain = self._victim_hold_until - now
                    self.get_logger().info(
                        f'[MISSION] Holding for victim assessment '
                        f'({remain:.1f}s remaining).'
                    )

                self._pub.publish(cmd)
                return

            if (self._presentation_mode
                    and QUICK_STOP_ON_DEMO_COMPLETE
                    and len(self._confirmed_victims) >= TARGET_VICTIMS_FOR_DEMO):
                self._phase = 'hover'
                if not self._mission_complete_announced:
                    self._mission_complete_announced = True
                    self.get_logger().warn(
                        '[MISSION] Demo target reached: required victims confirmed. '
                        'Switching to hover.'
                    )
                self._pub.publish(cmd)
                return

            if self._wp_index >= len(self._waypoints):
                if self._presentation_mode:
                    self._wp_index = 0
                    self.get_logger().info(
                        'Quick route loop complete — restarting from waypoint 1.'
                    )
                    self._pub.publish(cmd)
                    return

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

            # Forward speed — scale down when misaligned OR when close to wp
            alignment  = math.cos(yaw_err)           # 1 = perfect, -1 = opposite
            ramp       = min(1.0, dist_xy / 4.0)     # slow down in last 4 m
            forward    = max(0.0, alignment) * MAX_XY_SPEED * ramp
            cmd.linear.x = forward

            # Small lateral correction to cancel crosswind / drift
            cmd.linear.y = -math.sin(yaw_err) * 0.5

            # Phase 7: obstacle avoidance adjusts cmd in-place when needed.
            self._apply_obstacle_avoidance(cmd)

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
        try:
            rclpy.shutdown()
        except Exception:  # already shut down by SIGINT handler
            pass


if __name__ == '__main__':
    main()
