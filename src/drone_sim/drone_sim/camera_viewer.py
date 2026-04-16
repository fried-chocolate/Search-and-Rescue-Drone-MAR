#!/usr/bin/env python3
"""
Camera viewer node — Phase 5.

Architecture
------------
ROS2 callbacks run in a **background daemon thread** via `rclpy.spin`.
The OpenCV display loop (`cv2.waitKey`) runs on the **main thread**.

This separation is mandatory on Linux: OpenCV's window event loop (backed by
Qt or GTK) must be driven on the main thread, or the window appears but is
completely frozen — it cannot be resized, moved, or closed from the UI.

Subscribes
----------
  /drone/camera           sensor_msgs/msg/Image      (camera frames)
  /drone/camera_info      sensor_msgs/msg/CameraInfo (lens info, logged once)
  /model/quadrotor/pose   geometry_msgs/msg/Pose     (for HUD telemetry)

Features
--------
  * Live 640x480 downward camera feed, resizable window (WINDOW_NORMAL)
  * HUD overlay: drone x/y/z/yaw + frame counter
  * Mini top-down map (bottom-right): scene objects + drone position/heading
    * Mesh-aware victim detection tuned for person_standing model textures
  * "Waiting for camera feed..." placeholder before first frame arrives
  * q / Esc to close the viewer without killing the whole simulation
"""

import math
import os
import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

_HAS_DISPLAY = bool(os.environ.get('DISPLAY'))

# ── Scene objects for the mini-map ────────────────────────────────────────────
# (world_x, world_y, label, colour_BGR)
_SCENE = [
    (10,   5,  'B1',  (110, 110, 110)),   # building 1
    (-8,  12,  'B2',  (110, 110, 110)),   # building 2
    (5,   -8,  'Rb',  ( 70,  50,  30)),   # rubble 1
    (-12,  -6, 'Rb',  ( 70,  50,  30)),   # rubble 2
    (-15,  8,  'Car', ( 40,  40, 200)),   # hatchback (blue)
    (12,   1,  'V1',  (  0,   0, 255)),   # victim 1
    (-4,  14,  'V2',  (  0,   0, 255)),   # victim 2
]

_VICTIM_HINTS = [(12.0, 1.0), (-4.0, 14.0)]
_HINT_ACCEPT_RADIUS_M = 4.0


# ── ROS2 node ─────────────────────────────────────────────────────────────────

class CameraViewer(Node):
    """Receives camera + pose data; prepares annotated frames for display."""

    def __init__(self):
        super().__init__('camera_viewer')

        self._bridge = CvBridge()
        self._lock = threading.Lock()

        # Shared state (written in callbacks, read in main-thread display loop)
        self._latest_frame: np.ndarray | None = None
        self._frame_count = 0
        self._drone_x = 0.0
        self._drone_y = 0.0
        self._drone_z = 0.0
        self._drone_yaw = 0.0
        self._info_logged = False
        self._camera_info: CameraInfo | None = None
        self._detected_victims: list[tuple[int, int]] = []
        self._victim_pub = self.create_publisher(PoseArray, '/drone/victims', 10)
        self._last_victim_log_sig = ''
        self._last_victim_log_time = 0.0

        self.create_subscription(
            Image, '/drone/camera',
            self._image_cb, rclpy.qos.qos_profile_sensor_data)

        self.create_subscription(
            CameraInfo, '/drone/camera_info',
            self._info_cb, rclpy.qos.qos_profile_sensor_data)

        self.create_subscription(
            Pose, '/model/quadrotor/pose',
            self._pose_cb, 10)

        self.get_logger().info(
            f'Camera viewer ready. DISPLAY={os.environ.get("DISPLAY", "(none)")}')

    # ── Callbacks (run in background spin thread) ─────────────────────────────

    def _pose_cb(self, msg: Pose) -> None:
        q = msg.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        with self._lock:
            self._drone_x = msg.position.x
            self._drone_y = msg.position.y
            self._drone_z = msg.position.z
            self._drone_yaw = math.atan2(siny, cosy)

    def _image_cb(self, msg: Image) -> None:
        try:
            raw = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'imgmsg_to_cv2 failed: {exc}')
            return

        self._detected_victims = self._find_mesh_victims(raw)
        annotated = self._annotate(raw)

        with self._lock:
            self._latest_frame = annotated
            self._frame_count += 1
            fc = self._frame_count

        if fc % 30 == 0:
            h, w = annotated.shape[:2]
            with self._lock:
                xv, yv, zv = self._drone_x, self._drone_y, self._drone_z
                victim_count = len(self._detected_victims)
            self.get_logger().info(
                f'Camera: frame {fc}, {w}x{h}, '
                f'drone ({xv:+.1f}, {yv:+.1f}, {zv:.1f}) m, '
                f'victims={victim_count}')
        self._publish_victim_points()

    def _info_cb(self, msg: CameraInfo) -> None:
        if not self._info_logged:
            self.get_logger().info(
                f'Camera info: {msg.width}x{msg.height}, '
                f'distortion={msg.distortion_model}')
            self._info_logged = True
        self._camera_info = msg

    def _dedupe_points(self, points: list[tuple[int, int]],
                       min_dist_px: float = 22.0) -> list[tuple[int, int]]:
        """Remove near-duplicate pixel centers produced by overlapping masks."""
        unique: list[tuple[int, int]] = []
        min_d2 = min_dist_px * min_dist_px
        for px, py in points:
            if any((px - ux) ** 2 + (py - uy) ** 2 < min_d2 for ux, uy in unique):
                continue
            unique.append((px, py))
        return unique

    def _world_to_pixel(self, wx: float, wy: float) -> tuple[float, float] | None:
        """Project a ground-plane world point to image pixel coordinates."""
        if self._camera_info is None or self._drone_z <= 0.01:
            return None

        fx = self._camera_info.k[0]
        fy = self._camera_info.k[4]
        if fx <= 1e-6 or fy <= 1e-6:
            return None
        cx = self._camera_info.k[2]
        cy = self._camera_info.k[5]

        dx = wx - self._drone_x
        dy = wy - self._drone_y
        local_forward = dx * math.cos(self._drone_yaw) + dy * math.sin(self._drone_yaw)
        local_right = -dx * math.sin(self._drone_yaw) + dy * math.cos(self._drone_yaw)

        u = cx + (local_right / self._drone_z) * fx
        v = cy - (local_forward / self._drone_z) * fy
        return u, v

    def _find_mesh_victims(self, frame: np.ndarray) -> list[tuple[int, int]]:
        """
        Phase 9: detect victims using person mesh color signatures.

        Uses skin/denim/light-cloth ranges tuned from person_standing textures,
        then applies hint-assisted filtering near known victim locations.
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Skin and clothing color bands from person_standing textures.
        skin = cv2.inRange(hsv, np.array([5, 40, 80]), np.array([25, 190, 255]))
        denim = cv2.inRange(hsv, np.array([95, 40, 50]), np.array([125, 255, 255]))
        shirt = cv2.inRange(hsv, np.array([0, 0, 160]), np.array([179, 70, 255]))
        hair = cv2.inRange(hsv, np.array([5, 20, 20]), np.array([30, 180, 120]))

        # Keep legacy red range as a fallback marker detector.
        red = cv2.inRange(hsv, np.array([0, 120, 80]), np.array([10, 255, 255]))
        red |= cv2.inRange(hsv, np.array([160, 120, 80]), np.array([180, 255, 255]))

        mesh_mask = skin | denim | shirt | hair | red

        kernel = np.ones((5, 5), np.uint8)
        mesh_mask = cv2.morphologyEx(mesh_mask, cv2.MORPH_OPEN, kernel)
        mesh_mask = cv2.morphologyEx(mesh_mask, cv2.MORPH_CLOSE, kernel)

        # Presentation profile: restrict detection to projected victim hint windows.
        centers: list[tuple[int, int]] = []
        if self._camera_info is None:
            return centers

        h, w = frame.shape[:2]
        for hx, hy in _VICTIM_HINTS:
            px = self._world_to_pixel(hx, hy)
            if px is None:
                continue
            u, v = int(px[0]), int(px[1])
            if not (0 <= u < w and 0 <= v < h):
                continue

            r = 30
            x0, x1 = max(0, u - r), min(w, u + r)
            y0, y1 = max(0, v - r), min(h, v + r)
            patch_px = max(1, (x1 - x0) * (y1 - y0))

            patch_mesh = mesh_mask[y0:y1, x0:x1]
            total_ratio = cv2.countNonZero(patch_mesh) / patch_px
            skin_ratio = cv2.countNonZero(skin[y0:y1, x0:x1]) / patch_px
            denim_ratio = cv2.countNonZero(denim[y0:y1, x0:x1]) / patch_px
            shirt_ratio = cv2.countNonZero(shirt[y0:y1, x0:x1]) / patch_px
            hair_ratio = cv2.countNonZero(hair[y0:y1, x0:x1]) / patch_px
            red_ratio = cv2.countNonZero(red[y0:y1, x0:x1]) / patch_px

            if total_ratio < 0.018:
                continue
            if skin_ratio < 0.004:
                continue
            if (denim_ratio < 0.003
                    and shirt_ratio < 0.028
                    and hair_ratio < 0.008
                    and red_ratio < 0.006):
                continue

            moments = cv2.moments(patch_mesh, binaryImage=True)
            if moments['m00'] > 1.0:
                cx = int(x0 + moments['m10'] / moments['m00'])
                cy = int(y0 + moments['m01'] / moments['m00'])
            else:
                cx, cy = u, v
            centers.append((cx, cy))

        return self._dedupe_points(centers, min_dist_px=28.0)

    def _pixel_to_world(self, u: int, v: int) -> tuple[float, float, float]:
        if self._camera_info is None or self._drone_z <= 0.01:
            return (float('nan'), float('nan'), 0.0)

        fx = self._camera_info.k[0]
        fy = self._camera_info.k[4]
        if fx <= 1e-6 or fy <= 1e-6:
            return (float('nan'), float('nan'), 0.0)
        cx = self._camera_info.k[2]
        cy = self._camera_info.k[5]
        z = self._drone_z

        u_norm = (u - cx) / fx
        v_norm = (v - cy) / fy
        local_forward = -v_norm * z
        local_right = u_norm * z

        world_x = (self._drone_x
                   + local_forward * math.cos(self._drone_yaw)
                   - local_right * math.sin(self._drone_yaw))
        world_y = (self._drone_y
                   + local_forward * math.sin(self._drone_yaw)
                   + local_right * math.cos(self._drone_yaw))
        return world_x, world_y, 0.0

    def _publish_victim_points(self) -> None:
        if not self._detected_victims or self._camera_info is None:
            return

        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'

        coords = []
        for u, v in self._detected_victims:
            wx, wy, wz = self._pixel_to_world(u, v)
            if not (math.isfinite(wx) and math.isfinite(wy) and math.isfinite(wz)):
                continue

            pose = Pose()
            pose.position.x = wx
            pose.position.y = wy
            pose.position.z = wz
            pose.orientation.w = 1.0
            msg.poses.append(pose)
            coords.append((wx, wy, wz))

        if not msg.poses:
            return

        self._victim_pub.publish(msg)
        if coords:
            sig = ';'.join(f'{wx:.1f},{wy:.1f}' for wx, wy, _ in coords)
            now = time.monotonic()
            if sig != self._last_victim_log_sig or now - self._last_victim_log_time > 2.0:
                self._last_victim_log_sig = sig
                self._last_victim_log_time = now
                coord_str = ', '.join(
                    f'({wx:.2f}, {wy:.2f}, {wz:.2f})' for wx, wy, wz in coords)
                self.get_logger().info(
                    f'Victim detected: {len(coords)} target(s) at {coord_str}')

    # ── Frame annotation ──────────────────────────────────────────────────────

    def _annotate(self, frame: np.ndarray) -> np.ndarray:
        """Stamp HUD telemetry + mini top-down map onto a copy of frame."""
        out = frame.copy()
        h, w = out.shape[:2]

        with self._lock:
            x, y, z = self._drone_x, self._drone_y, self._drone_z
            yaw = self._drone_yaw
            fc = self._frame_count

        FONT = cv2.FONT_HERSHEY_SIMPLEX
        GREEN = (0, 255, 80)
        LBLUE = (180, 200, 255)

        # Semi-transparent dark banner at top
        banner = out.copy()
        cv2.rectangle(banner, (0, 0), (w, 56), (0, 0, 0), -1)
        cv2.addWeighted(banner, 0.6, out, 0.4, 0, out)

        cv2.putText(out,
                    f'DRONE CAM  |  x:{x:+.1f}m  y:{y:+.1f}m  z:{z:.1f}m  '
                    f'yaw:{math.degrees(yaw):+.0f}deg  frame:{fc}  '
                    f'victims:{len(self._detected_victims)}',
                    (8, 22), FONT, 0.52, GREEN, 1, cv2.LINE_AA)
        cv2.putText(out,
                    'Phase 9 - mesh-aware victim detection  640x480  30Hz  FOV=60deg',
                    (8, 46), FONT, 0.44, LBLUE, 1, cv2.LINE_AA)

        # Mini top-down map ─ bottom-right corner
        MAP_SZ  = 150    # pixel width/height of map box
        MAP_PAD = 8      # gap from window edge
        SCALE   = 4.5    # pixels per metre (covers ~+/-16 m)
        mx0 = w - MAP_SZ - MAP_PAD
        my0 = h - MAP_SZ - MAP_PAD
        cx  = mx0 + MAP_SZ // 2
        cy  = my0 + MAP_SZ // 2

        # Background + border
        cv2.rectangle(out, (mx0, my0), (mx0 + MAP_SZ, my0 + MAP_SZ),
                      (15, 15, 15), -1)
        cv2.rectangle(out, (mx0, my0), (mx0 + MAP_SZ, my0 + MAP_SZ),
                      (70, 70, 70), 1)

        def to_px(wx, wy):
            return (int(cx + wx * SCALE), int(cy - wy * SCALE))

        # Faint grid lines
        for g in range(-12, 13, 4):
            gx = to_px(g, 0)[0]
            gy = to_px(0, g)[1]
            cv2.line(out, (gx, my0), (gx, my0 + MAP_SZ), (35, 35, 35), 1)
            cv2.line(out, (mx0, gy), (mx0 + MAP_SZ, gy), (35, 35, 35), 1)

        # Scene objects
        for ox, oy, label, col in _SCENE:
            px = to_px(ox, oy)
            r = 6 if 'B' in label else 4
            cv2.circle(out, px, r, col, -1)

        # Drone: cyan dot + heading arrow
        dp = to_px(x, y)
        cv2.circle(out, dp, 5, (0, 255, 255), -1)
        ae = (int(dp[0] + 12 * math.cos(yaw)),
              int(dp[1] - 12 * math.sin(yaw)))
        cv2.arrowedLine(out, dp, ae, (0, 255, 255), 2, tipLength=0.35)

        for cx, cy in self._detected_victims:
            cv2.circle(out, (cx, cy), 16, (0, 255, 255), 2)
            cv2.putText(out, 'VICTIM', (cx - 26, cy - 22), FONT, 0.45,
                        (0, 255, 255), 1, cv2.LINE_AA)

        cv2.putText(out, 'MAP', (mx0 + 4, my0 + 13),
                    FONT, 0.38, (130, 130, 130), 1, cv2.LINE_AA)

        return out

    # ── Thread-safe frame accessor (called from main thread) ──────────────────

    def get_frame(self) -> np.ndarray | None:
        with self._lock:
            return self._latest_frame

    def destroy_node(self) -> None:
        cv2.destroyAllWindows()
        super().destroy_node()


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = CameraViewer()

    # ── ROS2 spin in a background daemon thread ───────────────────────────────
    # daemon=True: thread exits automatically when the main thread ends.
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    if _HAS_DISPLAY:
        # ── OpenCV display loop on the MAIN thread ────────────────────────────
        # cv2.waitKey MUST be called from the main thread.  If called from a
        # ROS2 callback thread, the window appears but is permanently frozen:
        # it cannot be resized, moved, or interacted with in any way.
        WIN = 'Drone Camera - downward view'
        cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)   # WINDOW_NORMAL = user-resizable
        cv2.resizeWindow(WIN, 800, 600)

        placeholder = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(placeholder, 'Waiting for camera feed...',
                    (60, 230), cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                    (80, 80, 80), 2, cv2.LINE_AA)
        cv2.putText(placeholder, '(bridge may still be starting)',
                    (90, 270), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (55, 55, 55), 1, cv2.LINE_AA)

        try:
            while rclpy.ok():
                frame = node.get_frame()
                cv2.imshow(WIN, frame if frame is not None else placeholder)
                key = cv2.waitKey(33)   # 33 ms ~ 30 fps; MUST be main thread
                if key in (ord('q'), ord('Q'), 27):   # q / Q / Esc
                    break
        except KeyboardInterrupt:
            pass
        finally:
            cv2.destroyAllWindows()
    else:
        node.get_logger().info('No DISPLAY set — log-only mode')
        try:
            spin_thread.join()
        except KeyboardInterrupt:
            pass

    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass   # already shut down by SIGINT handler


if __name__ == '__main__':
    main()
