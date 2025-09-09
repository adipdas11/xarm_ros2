#!/usr/bin/env python3
# tool_camera.py — ROS2 node publishing camera frames to /tool_camera (RELIABLE)
# Usage:
#   ros2 run tool_controller tool_camera --ros-args -p camera_index:=10

import time
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# ---- defaults (can be adjusted if needed) ----
DEFAULT_CAMERA_INDEX = 4
WIDTH, HEIGHT, FPS = 640, 480, 30
FOURCC = "MJPG"           # device may ignore; we log negotiated FOURCC
FRAME_ID = "tool_camera"
TOPIC = "/tool_camera"
# ------------------------------------------------

def open_camera(index: int):
    """
    Try CAP_ANY first (let OpenCV pick the backend), then CAP_V4L2.
    Configure stream; warm up a few frames; verify we can read.
    """
    tried = []
    for backend in (cv2.CAP_ANY, cv2.CAP_V4L2):
        try:
            cap = cv2.VideoCapture(index) if backend == cv2.CAP_ANY else cv2.VideoCapture(index, backend)
            tried.append(("CAP_ANY" if backend == cv2.CAP_ANY else "CAP_V4L2", bool(cap and cap.isOpened())))
            if not cap or not cap.isOpened():
                continue

            # Help some drivers return color frames
            cap.set(cv2.CAP_PROP_CONVERT_RGB, 1)

            # Best-effort configuration
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*FOURCC))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(WIDTH))
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(HEIGHT))
            cap.set(cv2.CAP_PROP_FPS, int(FPS))

            # Warmup reads
            for _ in range(5):
                cap.read()
                time.sleep(0.01)

            ok, frame = cap.read()
            if not ok or frame is None:
                cap.release()
                continue

            return cap, tried
        except Exception:
            continue
    return None, tried

class CameraPublisherNode(Node):
    def __init__(self):
        super().__init__("camera_publisher")
        self.bridge = CvBridge()

        # Parameter: camera_index (set via --ros-args -p camera_index:=N)
        self.declare_parameter("camera_index", DEFAULT_CAMERA_INDEX)
        cam_index = int(self.get_parameter("camera_index").value)
        self.get_logger().info(f"Using camera_index param: {cam_index}")

        # QoS: RELIABLE for maximum compatibility with common viewers (RViz)
        qos_reliable = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.pub = self.create_publisher(Image, TOPIC, qos_reliable)

        # Open camera
        self.cap, tried = open_camera(cam_index)
        for name, ok in tried:
            self.get_logger().info(f"Open attempt {name}: {'OK' if ok else 'FAIL'}")
        if not self.cap:
            self.get_logger().error(f"Could not open OpenCV camera index {cam_index}")
            raise SystemExit(1)

        # Log negotiated properties
        w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = float(self.cap.get(cv2.CAP_PROP_FPS))
        fourcc = int(self.cap.get(cv2.CAP_PROP_FOURCC))
        fourcc_str = "".join([chr((fourcc >> 8*i) & 0xFF) for i in range(4)])
        self.get_logger().info(
            f"Publishing {TOPIC} (RELIABLE) from index {cam_index} @ {w}x{h} FOURCC={fourcc_str or '????'} ~{fps:.1f} FPS"
        )

        # Timer @ target FPS
        period = 1.0 / FPS if FPS > 0 else 0.033
        self.timer = self.create_timer(period, self._publish_frame)
        self._seq = 0

    def _publish_frame(self):
        ok, frame = self.cap.read()
        if not ok or frame is None:
            self.get_logger().warn("Camera read failed; skipping frame")
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = FRAME_ID
        self.pub.publish(msg)

        self._seq += 1
        # if self._seq % int(max(FPS, 1) * 5) == 0:
        #     self.get_logger().info(f"Published {self._seq} frames")

    def destroy_node(self):
        try:
            if self.cap and self.cap.isOpened():
                self.cap.release()
        except Exception:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CameraPublisherNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
