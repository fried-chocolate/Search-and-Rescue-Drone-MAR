#!/usr/bin/env python3
"""
Camera viewer node — Phase 5.

Subscribes to the drone's downward-facing camera feed bridged from Gazebo:
  /drone/camera       → sensor_msgs/msg/Image
  /drone/camera_info  → sensor_msgs/msg/CameraInfo

Uses cv_bridge + OpenCV to decode and display frames in real time.
Falls back to logging-only mode if no DISPLAY is available (headless VMs).
"""

import os

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

# Detect headless environments (VirtualBox without 3D acceleration, SSH, etc.)
_HAS_DISPLAY = bool(os.environ.get('DISPLAY'))


class CameraViewer(Node):

    def __init__(self):
        super().__init__('camera_viewer')

        self._bridge = CvBridge()
        self._frame_count = 0
        self._last_info_logged = False

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(
            Image,
            '/drone/camera',
            self._image_callback,
            rclpy.qos.qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            '/drone/camera_info',
            self._info_callback,
            rclpy.qos.qos_profile_sensor_data,
        )

        self.get_logger().info(
            f'Camera viewer started. Display available: {_HAS_DISPLAY}'
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _image_callback(self, msg: Image) -> None:
        self._frame_count += 1

        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Image conversion failed: {exc}')
            return

        h, w = frame.shape[:2]

        # Overlay frame counter on the image
        cv2.putText(
            frame,
            f'Frame {self._frame_count}  {w}x{h}',
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2,
        )

        if _HAS_DISPLAY:
            cv2.imshow('Drone Camera — downward view', frame)
            cv2.waitKey(1)

        # Log a status line every 30 frames (~1 s at 30 Hz)
        if self._frame_count % 30 == 0:
            self.get_logger().info(
                f'Camera feed: frame {self._frame_count}, '
                f'size {w}x{h}, stamp {msg.header.stamp.sec}s'
            )

    def _info_callback(self, msg: CameraInfo) -> None:
        if not self._last_info_logged:
            self.get_logger().info(
                f'Camera info received: {msg.width}x{msg.height}, '
                f'distortion model: {msg.distortion_model}'
            )
            self._last_info_logged = True

    def destroy_node(self) -> None:
        if _HAS_DISPLAY:
            cv2.destroyAllWindows()
        super().destroy_node()


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = CameraViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
