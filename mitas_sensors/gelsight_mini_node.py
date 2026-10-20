#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image

from mitas_sensors.gsdevice import GelSight

import cv2
from cv_bridge import CvBridge
import numpy as np


class GelSightMiniNode(Node):
    """
    ROS2 node for the GelSight Mini tactile sensor.

    Publishes sensor images to /mitas_sensors/gelsight_mini/image.
    All parameters are configured via config/sensors.yaml.
    """

    def __init__(self):
        super().__init__("gelsight_mini_node")

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter("gelsight.capture_width",  895)
        self.declare_parameter("gelsight.capture_height", 672)
        self.declare_parameter("gelsight.output_width",   320)
        self.declare_parameter("gelsight.output_height",  240)
        self.declare_parameter("gelsight.publish_rate_hz", 25.0)
        self.declare_parameter("gelsight.border_crop_fraction", 1.0 / 7.0)
        self.declare_parameter("gelsight.device_name", "GelSight")
        self.declare_parameter("gelsight.rotate_180", False)
        self.declare_parameter("gelsight.topic", "gelsight_mini/image")
        self.declare_parameter("gelsight.frame_id", "gelsight_mini")
        self.declare_parameter("gelsight.qos.reliability", "reliable")
        self.declare_parameter("gelsight.qos.history",     "keep_last")
        self.declare_parameter("gelsight.qos.depth",       10)

        capture_w   = self.get_parameter("gelsight.capture_width").value
        capture_h   = self.get_parameter("gelsight.capture_height").value
        self.out_w  = self.get_parameter("gelsight.output_width").value
        self.out_h  = self.get_parameter("gelsight.output_height").value
        rate_hz     = self.get_parameter("gelsight.publish_rate_hz").value
        self.border = self.get_parameter("gelsight.border_crop_fraction").value
        device_name = self.get_parameter("gelsight.device_name").value
        self.rotate_180 = self.get_parameter("gelsight.rotate_180").value
        topic       = self.get_parameter("gelsight.topic").value
        self.frame_id = self.get_parameter("gelsight.frame_id").value

        self.get_logger().info(
            f"GelSight config — device: {device_name!r}, "
            f"capture: {capture_w}x{capture_h}, "
            f"output: {self.out_w}x{self.out_h}, rate: {rate_hz} Hz, "
            f"rotate_180: {self.rotate_180}"
        )

        # ── Camera ────────────────────────────────────────────────────────────
        self.cam = GelSight(img_shape=(capture_w, capture_h), camera_name=device_name)

        # ── ROS2 infrastructure ───────────────────────────────────────────────
        self.bridge = CvBridge()
        qos = self._build_qos_profile("gelsight.qos")
        self.publisher_ = self.create_publisher(Image, topic, qos)
        self.timer = self.create_timer(1.0 / rate_hz, self._publish_image)

        self.get_logger().info(
            f"GelSight Mini node started. Publishing on ~/{topic}"
        )

    # ── QoS helper ────────────────────────────────────────────────────────────

    def _build_qos_profile(self, base: str) -> QoSProfile:
        reliability_str = str(self.get_parameter(f"{base}.reliability").value).lower()
        history_str     = str(self.get_parameter(f"{base}.history").value).lower()
        depth           = int(self.get_parameter(f"{base}.depth").value)

        reliability_map = {
            "reliable":   QoSReliabilityPolicy.RELIABLE,
            "best_effort": QoSReliabilityPolicy.BEST_EFFORT,
        }
        history_map = {
            "keep_last": QoSHistoryPolicy.KEEP_LAST,
            "keep_all":  QoSHistoryPolicy.KEEP_ALL,
        }

        reliability = reliability_map.get(reliability_str, QoSReliabilityPolicy.RELIABLE)
        history     = history_map.get(history_str, QoSHistoryPolicy.KEEP_LAST)

        if reliability_str not in reliability_map:
            self.get_logger().warn(
                f"Unknown reliability '{reliability_str}', defaulting to RELIABLE"
            )
        if history_str not in history_map:
            self.get_logger().warn(
                f"Unknown history '{history_str}', defaulting to KEEP_LAST"
            )

        return QoSProfile(reliability=reliability, history=history, depth=depth)

    # ── Timer callback ────────────────────────────────────────────────────────

    def _publish_image(self):
        img = self.cam.latest_img

        # Crop border
        bx = int(img.shape[0] * self.border)
        by = int(np.floor(img.shape[1] * self.border))
        img = img[bx:img.shape[0] - bx, by:img.shape[1] - by]
        img = img[:, :-1]  # trim last column for clean resolution

        # Resize to output resolution
        img = cv2.resize(img, (self.out_w, self.out_h))

        if self.rotate_180:
            img = cv2.rotate(img, cv2.ROTATE_180)

        msg = self.bridge.cv2_to_imgmsg(img, "rgb8")
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.publisher_.publish(msg)


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    print("""
       GELSIGHT MINI
    .------------------.
    |.----------------.|
    |  ______________  |
    | |              | |
    | |              | |
    | |______________| |
    |__________________|
    """)

    rclpy.init(args=args)
    node = GelSightMiniNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
