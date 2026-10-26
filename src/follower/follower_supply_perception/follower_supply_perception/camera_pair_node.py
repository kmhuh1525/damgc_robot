"""Rectify each latest image and publish calibration with its exact stamp."""
from copy import deepcopy
import time
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

class CameraPairNode(Node):
    def __init__(self):
        super().__init__("camera_pair")
        cv2.setNumThreads(1)
        self.bridge = CvBridge()
        self.info = None
        self.maps = None
        self.key = None
        self.last_processed = 0.0
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.image_pub = self.create_publisher(Image, "tag_camera/image_rect", 1)
        self.info_pub = self.create_publisher(CameraInfo, "tag_camera/camera_info", 1)
        self.create_subscription(CameraInfo, "camera/camera_info", self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, "camera/image_raw", self.on_image, qos)

    def on_info(self, msg):
        if msg.k[0] > 0 and msg.k[4] > 0:
            self.info = msg

    def on_image(self, msg):
        if self.info is None or (msg.width, msg.height) != (self.info.width, self.info.height):
            return
        # Reject old acquisitions rather than pretending an old image is new.
        stamp = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp) / 1e9
        if not 0 <= age <= 2.0:
            return
        monotonic_now = time.monotonic()
        if monotonic_now - getattr(self, "last_processed", 0.0) < 0.1:
            return
        self.last_processed = monotonic_now
        try:
            info = deepcopy(self.info)
            key = (msg.width, msg.height, tuple(info.k), tuple(info.d), tuple(info.r), tuple(info.p))
            if key != self.key:
                k = np.asarray(info.k).reshape(3, 3)
                projection = np.asarray(info.p).reshape(3, 4)[:, :3]
                self.maps = cv2.initUndistortRectifyMap(k, np.asarray(info.d), np.asarray(info.r).reshape(3, 3), projection, (msg.width, msg.height), cv2.CV_32FC1)
                self.key = key
            gray = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
            rect = cv2.remap(gray, *self.maps, interpolation=cv2.INTER_LINEAR)
            output = self.bridge.cv2_to_imgmsg(rect, encoding="mono8")
            output.header = msg.header
            info.header = msg.header
            info.k = np.asarray(info.p).reshape(3, 4)[:, :3].reshape(-1).tolist()
            info.d = [0.0] * len(info.d)
            info.r = np.eye(3).reshape(-1).tolist()
            self.info_pub.publish(info)
            self.image_pub.publish(output)
        except Exception as exc:
            self.get_logger().error(str(exc), throttle_duration_sec=5.0)

def main(args=None):
    rclpy.init(args=args)
    node = CameraPairNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
