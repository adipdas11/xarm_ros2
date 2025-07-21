#!/usr/bin/env python3
"""
transform_node.py
ROS2 node that provides a service to transform 3D points from camera frame to robot base frame.
Includes logging with emojis and waits until the TF transform is available.
"""
import os
import time

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import PointStamped
from hd_disassembly.srv import TransformPoint

import tf2_ros
import tf2_geometry_msgs  # noqa: F401

class TransformNode(Node):
    def __init__(self):
        super().__init__('transform_node')
        self.get_logger().info("🚀 TransformNode starting up...")

        # Load parameters from YAML
        pkg_share = get_package_share_directory('hd_disassembly')
        yaml_path = os.path.join(pkg_share, 'config', 'disassembler_params.yaml')
        self.declare_parameter('tf.source_frame', 'camera_color_optical_frame')
        self.declare_parameter('tf.target_frame', 'link_base')

        self.source_frame = self.get_parameter('tf.source_frame').value
        self.target_frame = self.get_parameter('tf.target_frame').value

        self.get_logger().info(f"🔧 Source frame: {self.source_frame}")
        self.get_logger().info(f"🔧 Target frame: {self.target_frame}")

        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Service server
        self.srv = self.create_service(
            TransformPoint,
            'transform_point',
            self.handle_transform_point
        )

        self.get_logger().info("✅ Service 'transform_point' is ready")

    def handle_transform_point(self, request, response):
        """
        Handle a TransformPoint request: wait for TF, then transform.
        """
        in_frame = request.point_in.header.frame_id
        self.get_logger().info(f"📩 Received request to transform point from '{in_frame}' to '{self.target_frame}'")

        # Wait for transform availability
        start_time = self.get_clock().now()
        timeout_sec = 5.0
        while rclpy.ok() and not self.tf_buffer.can_transform(
            self.target_frame,
            in_frame,
            rclpy.time.Time()  # latest
        ):
            elapsed = (self.get_clock().now() - start_time).nanoseconds * 1e-9
            if elapsed > timeout_sec:
                self.get_logger().error(f"❌ Timeout ({timeout_sec}s) waiting for TF from '{in_frame}' to '{self.target_frame}'")
                return response
            self.get_logger().info(f"⏳ Waiting for TF... ({elapsed:.1f}s)")
            time.sleep(0.1)

        # Perform transformation
        try:
            self.get_logger().info("🔄 Transform available, looking up transform...")
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                in_frame,
                rclpy.time.Time()  # latest
            )
            self.get_logger().info("🔄 Applying transform to point...")
            response.point_out = tf2_geometry_msgs.do_transform_point(
                request.point_in,
                transform
            )
            response.point_out.header.frame_id = self.target_frame
            self.get_logger().info("✅ Point transformed successfully 🚀")
        except Exception as e:
            self.get_logger().error(f"❌ Error during transform: {e}")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TransformNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Keyboard interrupt, shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
