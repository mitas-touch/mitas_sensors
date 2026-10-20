#!/usr/bin/env python3

import signal
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

import dv_processing as dv
import numpy as np
from datetime import timedelta


class EvetacNode(Node):
    """
    ROS2 node for the Evetac event-based tactile sensor (DV camera).

    Connects to the USB DV event camera, accumulates events into frames,
    and publishes them as sensor_msgs/Image on /mitas_sensors/evetac/image.
    All parameters are configured via config/sensors.yaml.
    """

    def __init__(self):
        super().__init__("evetac_node")

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter("evetac.accumulation_time_ms", 5.0)
        self.declare_parameter("evetac.decay_tau_ms", 15.0)
        self.declare_parameter("evetac.output_width",   320)
        self.declare_parameter("evetac.output_height",  240)
        self.declare_parameter("evetac.publish_rate_hz",      200.0)
        self.declare_parameter("evetac.neutral_potential",    0.5)
        self.declare_parameter("evetac.event_contribution",   0.2)
        self.declare_parameter("evetac.min_potential",        0.0)
        self.declare_parameter("evetac.max_potential",        1.0)
        self.declare_parameter("evetac.topic",    "evetac/image")
        self.declare_parameter("evetac.frame_id", "evetac")
        self.declare_parameter("evetac.qos.reliability", "reliable")
        self.declare_parameter("evetac.qos.history",     "keep_last")
        self.declare_parameter("evetac.qos.depth",       10)

        accum_ms    = self.get_parameter("evetac.accumulation_time_ms").value
        self.out_w  = self.get_parameter("evetac.output_width").value
        self.out_h  = self.get_parameter("evetac.output_height").value
        rate_hz     = self.get_parameter("evetac.publish_rate_hz").value
        neutral     = self.get_parameter("evetac.neutral_potential").value
        contrib     = self.get_parameter("evetac.event_contribution").value
        min_pot     = self.get_parameter("evetac.min_potential").value
        max_pot     = self.get_parameter("evetac.max_potential").value
        topic       = self.get_parameter("evetac.topic").value
        self.frame_id = self.get_parameter("evetac.frame_id").value

        self.get_logger().info(
            f"Evetac config — accumulation: {accum_ms} ms, "
            f"decay: {self.get_parameter('evetac.decay_tau_ms').value} ms, "
            f"rate: {rate_hz} Hz"
        )

        # ── Camera ────────────────────────────────────────────────────────────
        self.get_logger().info("Opening DV camera...")
        try:
            self.capture = dv.io.camera.open()
            if not self.capture.isEventStreamAvailable():
                raise RuntimeError("Event stream not available on this device.")
            self.width, self.height = self.capture.getEventResolution()
            self.get_logger().info(f"DV camera opened: {self.width}x{self.height}")
        except Exception as e:
            self.get_logger().error(f"Camera open failed: {e}")
            raise

        # ── Event accumulator ─────────────────────────────────────────────────
        self.accumulator = dv.Accumulator((self.width, self.height))
        self.accumulator.setNeutralPotential(neutral)
        self.accumulator.setEventContribution(contrib)
        self.accumulator.setMinPotential(min_pot)
        self.accumulator.setMaxPotential(max_pot)

        # Exponential decay keeps the background from saturating.
        # setDecayParam expects microseconds.
        tau_ms = self.get_parameter("evetac.decay_tau_ms").value
        self.accumulator.setDecayFunction(dv.Accumulator.Decay.EXPONENTIAL)
        self.accumulator.setDecayParam(tau_ms * 1000)
        self.accumulator.setSynchronousDecay(True)

        self.slicer = dv.EventStreamSlicer()
        self.slicer.doEveryTimeInterval(
            timedelta(milliseconds=accum_ms),
            self._process_events,
        )

        # ── ROS2 infrastructure ───────────────────────────────────────────────
        self.bridge = CvBridge()
        qos = self._build_qos_profile("evetac.qos")
        self.publisher_ = self.create_publisher(Image, topic, qos)
        self.timer = self.create_timer(1.0 / rate_hz, self._poll_camera)

        self.get_logger().info(
            f"Evetac node started. Publishing on ~/{topic}"
        )

    # ── QoS helper ────────────────────────────────────────────────────────────

    def _build_qos_profile(self, base: str) -> QoSProfile:
        reliability_str = str(self.get_parameter(f"{base}.reliability").value).lower()
        history_str     = str(self.get_parameter(f"{base}.history").value).lower()
        depth           = int(self.get_parameter(f"{base}.depth").value)

        reliability_map = {
            "reliable":    QoSReliabilityPolicy.RELIABLE,
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

    # ── Camera polling ────────────────────────────────────────────────────────

    def _poll_camera(self):
        events = self.capture.getNextEventBatch()
        if events is not None:
            self.slicer.accept(events)

    # ── Event processing ──────────────────────────────────────────────────────

    def _process_events(self, events):
        self.accumulator.accept(events)
        frame = self.accumulator.generateFrame()
        if frame is None:
            return

        image = frame.image  # float32 in [0, 1]
        image = (image * 255).astype(np.uint8)

        height, width = image.shape[:2]
        if (width != self.out_w) or (height != self.out_h):
            image = cv2.resize(image, (self.out_w, self.out_h), interpolation=cv2.INTER_AREA)
        self._publish_image(image)

    # ── Image publishing ──────────────────────────────────────────────────────

    def _publish_image(self, image: np.ndarray):
        msg = self.bridge.cv2_to_imgmsg(image, encoding="mono8")
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.publisher_.publish(msg)

    # ── Cleanup ───────────────────────────────────────────────────────────────

    def shutdown(self):
        self.get_logger().info("Evetac node shutting down.")
        self.destroy_node()


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = EvetacNode()

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    def _signal_handler(sig, frame):
        node.shutdown()
        executor.shutdown()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, _signal_handler)

    executor.spin()


if __name__ == "__main__":
    main()
