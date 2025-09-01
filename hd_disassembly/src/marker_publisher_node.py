#!/usr/bin/env python3
"""
marker_publisher_node.py (dual-arm aware)

Subscribes:  /tracked_parts  (hd_disassembly/TrackedPart)  [points in camera frame]
Calls:       /transform_point_tool   (hd_disassembly/TransformPoint)
             /transform_point_manip  (hd_disassembly/TransformPoint)
Publishes:   /part_markers (visualization_msgs/MarkerArray) in the *target base* frame

- Chooses the TF service per-part using disassembler_params.yaml:
    disassembly.arm_assign.{screw->tool, lid->manip, ...}
- Falls back gracefully if a specific service isn't available (uses /transform_point if present).
- Uses MultiThreadedExecutor to avoid deadlocks while waiting on services inside the callback.
"""

import os
import yaml
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from ament_index_python.packages import get_package_share_directory

from builtin_interfaces.msg import Duration as RosDuration
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped
from hd_disassembly.srv import TransformPoint
from hd_disassembly.msg import TrackedPart


class MarkerPublisherNode(Node):
    def __init__(self):
        super().__init__('marker_publisher_node')
        self.get_logger().info("🚀 MarkerPublisherNode starting up…")

        # ----------------- Load config -----------------
        pkg_share = get_package_share_directory('hd_disassembly')
        yaml_path = os.path.join(pkg_share, 'config', 'disassembler_params.yaml')
        with open(yaml_path, 'r') as f:
            cfg = yaml.safe_load(f) or {}

        # Marker lifetime (seconds)
        lifetime_sec = float(cfg.get('marker_publisher', {}).get('marker_lifetime', 0.5))
        self.marker_lifetime = RosDuration(sec=int(lifetime_sec), nanosec=int((lifetime_sec % 1.0) * 1e9))
        self.get_logger().info(f"🔧 Marker lifetime: {lifetime_sec:.2f}s")

        # Arm assignment (label -> 'tool'/'manip')
        self.arm_assign = (cfg.get('disassembly', {}) or {}).get('arm_assign', {}) or {}
        if not self.arm_assign:
            self.get_logger().warn("⚠️ No disassembly.arm_assign in YAML; defaulting all to 'manip'")
        else:
            self.get_logger().info(f"🗺️  Arm assignment: {self.arm_assign}")

        # ----------------- TF services -----------------
        # Try to connect to the new per-arm services; keep a fallback to legacy /transform_point if present.
        self.cli_tool: Optional[rclpy.client.Client] = self.create_client(TransformPoint, 'transform_point_tool')
        self.cli_man:  Optional[rclpy.client.Client] = self.create_client(TransformPoint, 'transform_point_manip')
        self.cli_legacy: Optional[rclpy.client.Client] = self.create_client(TransformPoint, 'transform_point')

        # Wait briefly for any that exist
        found_any = False
        for name, cli in [('transform_point_tool', self.cli_tool),
                          ('transform_point_manip', self.cli_man),
                          ('transform_point', self.cli_legacy)]:
            if cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f"✅ Found service: {name}")
                found_any = True
        if not found_any:
            self.get_logger().warn("⏳ No transform services available yet; callbacks will retry.")

        # ----------------- Sub + Pub -----------------
        self.subscription = self.create_subscription(
            TrackedPart, 'tracked_parts', self.tracked_callback, 10
        )
        self.marker_pub = self.create_publisher(MarkerArray, 'part_markers', 10)
        self.get_logger().info("✅ Subscribed to /tracked_parts; publishing /part_markers")

    # ----------------- Helpers -----------------
    def _color_for_label(self, label: str) -> Tuple[float, float, float]:
        """Deterministic RGB in [0..1] based on label string."""
        h = abs(hash(label)) & 0xFFFFFF
        return ((h >> 16) & 0xFF) / 255.0, ((h >> 8) & 0xFF) / 255.0, (h & 0xFF) / 255.0

    def _pick_client_for_label(self, label: str) -> Tuple[Optional[rclpy.client.Client], str]:
        """
        Decide which transform service to call for this label based on arm_assign.
        Returns (client, arm_tag) where arm_tag is 'tool' or 'manip' (or 'legacy' if fallback).
        """
        which = self.arm_assign.get(label, self.arm_assign.get('default', 'manip')).lower()
        if which == 'tool' and self.cli_tool and self.cli_tool.service_is_ready():
            return self.cli_tool, 'tool'
        if which == 'manip' and self.cli_man and self.cli_man.service_is_ready():
            return self.cli_man, 'manip'
        # Fallback
        if self.cli_legacy and self.cli_legacy.service_is_ready():
            self.get_logger().warn(f"🛈 Using legacy /transform_point for label '{label}' (no per-arm service ready)")
            return self.cli_legacy, 'legacy'
        # Nothing ready
        return None, 'none'

    # ----------------- Callback -----------------
    def tracked_callback(self, msg: TrackedPart):
        label = msg.part_label or 'unknown'
        tid = int(msg.track_id)
        self.get_logger().debug(f"📥 TrackedPart id={tid} label={label} frame={msg.position.header.frame_id}")

        client, arm_tag = self._pick_client_for_label(label)
        if client is None:
            self.get_logger().warn(f"⏳ No transform service available yet for {label} (id={tid}); skipping.")
            return

        # Build request (point is in camera frame; transform node forces the source frame internally)
        req = TransformPoint.Request()
        req.point_in = msg.position  # PointStamped

        # Call async; capture id/label to finish later
        future = client.call_async(req)
        future.add_done_callback(lambda f, _tid=tid, _label=label, _arm=arm_tag: self._on_tf_done(f, _tid, _label, _arm))

    def _on_tf_done(self, future, tid: int, label: str, arm_tag: str):
        try:
            res = future.result()
            out: PointStamped = res.point_out
            # If service failed silently, header may be empty
            if not out.header.frame_id:
                self.get_logger().warn(f"⚠️ Transform returned empty frame for id={tid}, label={label}")
                return

            # Sphere marker at transformed point
            marker = Marker()
            marker.header = out.header
            marker.ns = f'parts_{arm_tag}'
            marker.id = tid
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = out.point
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.03  # 3 cm sphere
            r, g, b = self._color_for_label(label)
            marker.color.r = r; marker.color.g = g; marker.color.b = b; marker.color.a = 1.0
            marker.lifetime = self.marker_lifetime

            # Text marker slightly above
            text = Marker()
            text.header = out.header
            text.ns = f'labels_{arm_tag}'
            text.id = tid
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position = out.point
            text.pose.position.z += 0.05  # +5 cm
            text.pose.orientation.w = 1.0
            text.scale.z = 0.035  # text height
            text.color.r = r; text.color.g = g; text.color.b = b; text.color.a = 1.0
            text.text = f"{label}_{tid}"
            text.lifetime = self.marker_lifetime

            # Publish
            arr = MarkerArray()
            arr.markers.extend([marker, text])
            self.marker_pub.publish(arr)
            self.get_logger().info(f"✅ Marker(s) published for id={tid}, label={label} in {out.header.frame_id}")
        except Exception as e:
            self.get_logger().error(f"❌ Transform/publish failed for id={tid}, label={label}: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = MarkerPublisherNode()
    # 2 threads so the callback can wait for a service without blocking the executor
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Shutting down MarkerPublisherNode…")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
