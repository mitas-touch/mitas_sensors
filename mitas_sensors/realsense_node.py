import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

import cv2
from mitas_sensors.rs_capture import RSCapture


class RealSensePublisher(Node):
    """
    ROS 2 node for the Intel RealSense wrist camera.

    Publishes color images and CameraInfo. All parameters are configured
    via config/sensors.yaml.
    """

    def __init__(self):
        super().__init__("realsense_node")
        self.realsense = None

        self.declare_parameter("realsense.serial_number", "")
        self.declare_parameter("realsense.capture_width", 640)
        self.declare_parameter("realsense.capture_height", 480)
        self.declare_parameter("realsense.output_width", 320)
        self.declare_parameter("realsense.output_height", 240)
        self.declare_parameter("realsense.publish_rate_hz", 30.0)
        self.declare_parameter("realsense.topic", "realsense/image")
        self.declare_parameter("realsense.frame_id", "realsense")
        self.declare_parameter("realsense.qos.reliability", "best_effort")
        self.declare_parameter("realsense.qos.history", "keep_last")
        self.declare_parameter("realsense.qos.depth", 5)

        serial = str(self.get_parameter("realsense.serial_number").value).strip()
        capture_w = int(self.get_parameter("realsense.capture_width").value)
        capture_h = int(self.get_parameter("realsense.capture_height").value)
        self.out_w = int(self.get_parameter("realsense.output_width").value)
        self.out_h = int(self.get_parameter("realsense.output_height").value)
        rate_hz = float(self.get_parameter("realsense.publish_rate_hz").value)
        topic = self.get_parameter("realsense.topic").value
        self.frame_id = self.get_parameter("realsense.frame_id").value

        self.get_logger().info(
            f"RealSense config — serial: {serial!r}, "
            f"capture: {capture_w}x{capture_h}, "
            f"output: {self.out_w}x{self.out_h}, rate: {rate_hz} Hz"
        )

        detected = RSCapture.get_device_serial_numbers()
        if not serial or serial not in detected:
            self.get_logger().error(
                "Set realsense.serial_number in config/sensors.yaml "
                f"(rs-enumerate-devices). Detected: {detected or 'none'}."
            )
            raise RuntimeError("RealSense serial_number is not set to a connected camera")

        self.bridge = CvBridge()
        qos = self._build_qos_profile("realsense.qos")
        self.publisher = self.create_publisher(Image, topic, qos)
        self.info_pub = self.create_publisher(CameraInfo, "realsense/camera_info", qos)

        self.realsense = RSCapture(
            name="RS",
            serial_number=serial,
            dim=(capture_w, capture_h),
            fps=int(rate_hz),
            depth=False,
        )
        self.intrinsics = self.realsense.get_intrinsics()
        self.timer = self.create_timer(1.0 / rate_hz, self.publish_frame)

        self.get_logger().info(f"RealSense node started. Publishing on ~/{topic}")

    def _build_qos_profile(self, base: str) -> QoSProfile:
        reliability_str = str(self.get_parameter(f"{base}.reliability").value).lower()
        history_str = str(self.get_parameter(f"{base}.history").value).lower()
        depth = int(self.get_parameter(f"{base}.depth").value)

        reliability_map = {
            "reliable": QoSReliabilityPolicy.RELIABLE,
            "best_effort": QoSReliabilityPolicy.BEST_EFFORT,
        }
        history_map = {
            "keep_last": QoSHistoryPolicy.KEEP_LAST,
            "keep_all": QoSHistoryPolicy.KEEP_ALL,
        }

        reliability = reliability_map.get(reliability_str, QoSReliabilityPolicy.BEST_EFFORT)
        history = history_map.get(history_str, QoSHistoryPolicy.KEEP_LAST)

        if reliability_str not in reliability_map:
            self.get_logger().warn(
                f"Unknown reliability '{reliability_str}', defaulting to BEST_EFFORT"
            )
        if history_str not in history_map:
            self.get_logger().warn(
                f"Unknown history '{history_str}', defaulting to KEEP_LAST"
            )

        return QoSProfile(reliability=reliability, history=history, depth=depth)

    def _resize_image(self, img):
        height, width = img.shape[:2]
        if width == self.out_w and height == self.out_h:
            return img
        return cv2.resize(img, (self.out_w, self.out_h), interpolation=cv2.INTER_AREA)

    def publish_frame(self):
        success, img = self.realsense.read()
        if not success or img is None:
            self.get_logger().warn("No frame received from RealSense")
            return

        try:
            img = self._resize_image(img)
            image_msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
            image_msg.header.stamp = self.get_clock().now().to_msg()
            image_msg.header.frame_id = self.frame_id
            self.publisher.publish(image_msg)

            camera_info_msg = self._create_camera_info(self.intrinsics)
            camera_info_msg.header = image_msg.header
            self.info_pub.publish(camera_info_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to convert/publish image: {e}")

    def _create_camera_info(self, intrinsics):
        src_w = float(intrinsics.width) if intrinsics.width else 1.0
        src_h = float(intrinsics.height) if intrinsics.height else 1.0
        scale_x = self.out_w / src_w
        scale_y = self.out_h / src_h

        msg = CameraInfo()
        msg.width = self.out_w
        msg.height = self.out_h
        msg.distortion_model = "plumb_bob"
        msg.d = list(intrinsics.coeffs)

        fx = intrinsics.fx * scale_x
        fy = intrinsics.fy * scale_y
        ppx = intrinsics.ppx * scale_x
        ppy = intrinsics.ppy * scale_y

        msg.k = [
            fx, 0.0, ppx,
            0.0, fy, ppy,
            0.0, 0.0, 1.0,
        ]
        msg.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ]
        msg.p = [
            fx, 0.0, ppx, 0.0,
            0.0, fy, ppy, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        return msg

    def destroy_node(self):
        if self.realsense is not None:
            self.realsense.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RealSensePublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
