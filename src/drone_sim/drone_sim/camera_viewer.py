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
  * "Waiting for camera feed..." placeholder before first frame arrives
  * q / Esc to close the viewer without killing the whole simulation
"""

import math
import os
import threading

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
    (12,   3,  'V1',  (  0,   0, 255)),   # victim 1 (red)
    (-6,  14,  'V2',  (  0,   0, 255)),   # victim 2 (red)
]


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

        self._detected_victims = self._find_red_victims(raw)
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

    def _find_red_victims(self, frame: np.ndarray) -> list[tuple[int, int]]:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower1 = np.array([0, 120, 80])
        upper1 = np.array([10, 255, 255])
        lower2 = np.array([160, 120, 80])
        upper2 = np.array([180, 255, 255])
        mask = cv2.inRange(hsv, lower1, upper1)
        mask |= cv2.inRange(hsv, lower2, upper2)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        centers: list[tuple[int, int]] = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 150:
                continue
            (cx, cy), radius = cv2.minEnclosingCircle(contour)
            centers.append((int(cx), int(cy)))
        return centers

    def _pixel_to_world(self, u: int, v: int) -> tuple[float, float, float]:
        if self._camera_info is None or self._drone_z <= 0.01:
            return (float('nan'), float('nan'), 0.0)

        fx = self._camera_info.k[0]
        fy = self._camera_info.k[4]
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
            pose = Pose()
            pose.position.x = wx
            pose.position.y = wy
            pose.position.z = wz
            pose.orientation.w = 1.0
            msg.poses.append(pose)
            coords.append((wx, wy, wz))

        self._victim_pub.publish(msg)
        if coords:
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
                    'Phase 5 - downward camera  640x480  30Hz  FOV=60deg',
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
