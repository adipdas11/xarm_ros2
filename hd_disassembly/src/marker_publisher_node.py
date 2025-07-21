#!/usr/bin/env python3
"""
marker_publisher_node.py

ROS2 node that subscribes to TrackedPart msgs, transforms each part position
from camera frame to robot base frame via service, and publishes visualization
markers in RViz (MarkerArray) to show part locations and labels.

Uses a MultiThreadedExecutor to avoid deadlock when calling the transform service
inside the subscription callback.
"""

import os
import yaml
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from ament_index_python.packages import get_package_share_directory

from visualization_msgs.msg import Marker, MarkerArray
from hd_disassembly.srv import TransformPoint
from hd_disassembly.msg import TrackedPart

class MarkerPublisherNode(Node):
    def __init__(self):
        super().__init__('marker_publisher_node')
        self.get_logger().info("🚀 MarkerPublisherNode starting up...")

        # Load parameters
        pkg_share = get_package_share_directory('hd_disassembly')
        yaml_path = os.path.join(pkg_share, 'config', 'disassembler_params.yaml')
        with open(yaml_path, 'r') as f:
            cfg = yaml.safe_load(f)

        self.marker_lifetime = cfg['marker_publisher']['marker_lifetime']
        self.get_logger().info(f"🔧 Marker lifetime set to {self.marker_lifetime}s")

        # Service client for transforming points
        self.transform_client = self.create_client(TransformPoint, 'transform_point')
        while not self.transform_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("⏳ Waiting for transform_point service...")
        self.get_logger().info("✅ Connected to transform_point service")

        # Subscription to tracked parts
        self.subscription = self.create_subscription(
            TrackedPart,
            'tracked_parts',
            self.tracked_callback,
            10)
        self.get_logger().info("✅ Subscribed to 'tracked_parts'")

        # Publisher for MarkerArray
        self.marker_pub = self.create_publisher(MarkerArray, 'part_markers', 10)

    def tracked_callback(self, msg: TrackedPart):
        self.get_logger().info(f"📩 Received TrackedPart id={msg.track_id}, part={msg.part_label}")

        # Prepare transform request
        req = TransformPoint.Request()
        req.point_in = msg.position  # PointStamped in camera frame

        # Call transform service asynchronously
        future = self.transform_client.call_async(req)
        future.add_done_callback(lambda f, tid=msg.track_id, label=msg.part_label: 
                                 self.on_transform_done(f, tid, label))

    def on_transform_done(self, future, track_id, part_label):
        try:
            res = future.result()
            out = res.point_out

            # Build sphere marker
            marker = Marker()
            marker.header = out.header
            marker.header.frame_id = out.header.frame_id
            marker.ns = 'parts'
            marker.id = track_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = out.point
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.04
            marker.scale.y = 0.04
            marker.scale.z = 0.04
            r, g, b = self._color_for_label(part_label)
            marker.color.r = r; marker.color.g = g; marker.color.b = b; marker.color.a = 1.0
            marker.lifetime.sec = int(self.marker_lifetime)

            # Build text label marker
            text = Marker()
            text.header = marker.header
            text.ns = 'part_labels'
            text.id = track_id
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position = out.point
            text.pose.position.z += 0.05
            text.pose.orientation.w = 1.0
            text.scale.z = 0.04
            text.color = marker.color
            text.text = f"{part_label}_{track_id}"
            text.lifetime = marker.lifetime

            # Publish both
            m_array = MarkerArray()
            m_array.markers.append(marker)
            m_array.markers.append(text)
            self.marker_pub.publish(m_array)

            self.get_logger().info(f"✅ Published MarkerArray for id={track_id}")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to transform/publish for id={track_id}: {e}")

    def _color_for_label(self, label: str):
        h = abs(hash(label)) % 0xFFFFFF
        return ((h >> 16) & 0xFF) / 255.0, ((h >> 8) & 0xFF) / 255.0, (h & 0xFF) / 255.0


def main(args=None):
    rclpy.init(args=args)
    node = MarkerPublisherNode()
    # Use MultiThreadedExecutor to avoid deadlock
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Shutting down MarkerPublisherNode...")
    finally:
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
