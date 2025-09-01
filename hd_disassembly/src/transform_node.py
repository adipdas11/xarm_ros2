#!/usr/bin/env python3
"""
transform_node.py — dual-arm transform service with detailed logging.

- Forces the source camera frame (so header mismatches won't break TF).
- Logs incoming coordinates and transformed coordinates.
- Logs the TF used (translation + quaternion).
- Services:
    /transform_point_tool   : camera -> R_link_base
    /transform_point_manip  : camera -> L_link_base
    /transform_point        : alias to tool
"""

import time
import rclpy
from rclpy.node import Node

from hd_disassembly.srv import TransformPoint
import tf2_ros
import tf2_geometry_msgs  # registers geometry types with TF2


class TransformNode(Node):
    def __init__(self):
        super().__init__('transform_node')
        self.get_logger().info("🚀 TransformNode (dual-arm) starting…")

        # Parameters (defaults align with your calibration)
        self.declare_parameter('tf.source_frame',        'R_camera_color_optical_frame')
        self.declare_parameter('tf.target_frame_tool',   'R_link_base')
        self.declare_parameter('tf.target_frame_manip',  'L_link_base')
        self.declare_parameter('tf.timeout_sec',         5.0)
        # If true, ignore request.header.frame_id and always use tf.source_frame.
        self.declare_parameter('force_source_frame',     True)

        self.source_frame   = self.get_parameter('tf.source_frame').value
        self.target_tool    = self.get_parameter('tf.target_frame_tool').value
        self.target_manip   = self.get_parameter('tf.target_frame_manip').value
        self.timeout_sec    = float(self.get_parameter('tf.timeout_sec').value)
        self.force_src      = bool(self.get_parameter('force_source_frame').value)

        self.get_logger().info(f"🔧 Using source frame: {self.source_frame}  (force={self.force_src})")
        self.get_logger().info(f"🔧 Tool  target: {self.target_tool}")
        self.get_logger().info(f"🔧 Manip target: {self.target_manip}")

        # TF
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Services
        self.create_service(TransformPoint, 'transform_point_tool',
                            lambda req, res: self._handle(req, res, self.target_tool))
        self.create_service(TransformPoint, 'transform_point_manip',
                            lambda req, res: self._handle(req, res, self.target_manip))
        # Back-compat alias (defaults to tool base)
        self.create_service(TransformPoint, 'transform_point',
                            lambda req, res: self._handle(req, res, self.target_tool))

        self.get_logger().info("✅ Services ready: /transform_point_tool, /transform_point_manip, /transform_point")

    def _handle(self, request: TransformPoint.Request,
                response: TransformPoint.Response,
                target_frame: str):

        # Original header frame (may be empty or mismatched)
        hdr_frame = request.point_in.header.frame_id or '(empty)'
        used_src  = self.source_frame if self.force_src else (request.point_in.header.frame_id or self.source_frame)

        px = float(request.point_in.point.x)
        py = float(request.point_in.point.y)
        pz = float(request.point_in.point.z)

        if self.force_src and hdr_frame != self.source_frame:
            self.get_logger().warn(
                f"🔁 Forcing source frame: header='{hdr_frame}' → used='{self.source_frame}'"
            )

        self.get_logger().info(
            f"📥 Incoming point  frame='{hdr_frame}'  used_src='{used_src}'  "
            f"xyz=({px:.4f}, {py:.4f}, {pz:.4f}) m"
        )

        # Wait for TF availability
        t0 = time.time()
        while rclpy.ok() and not self.tf_buffer.can_transform(target_frame, used_src, rclpy.time.Time()):
            if (time.time() - t0) > self.timeout_sec:
                self.get_logger().error(
                    f"⛔ TF timeout ({self.timeout_sec:.1f}s) waiting '{used_src}' → '{target_frame}'"
                )
                return response
            time.sleep(0.05)

        try:
            tf = self.tf_buffer.lookup_transform(target_frame, used_src, rclpy.time.Time())

            # Log the TF used
            tt = tf.transform.translation
            tq = tf.transform.rotation
            self.get_logger().info(
                "🧭 Using TF "
                f"T=({tt.x:.4f},{tt.y:.4f},{tt.z:.4f})  "
                f"Q=({tq.x:.4f},{tq.y:.4f},{tq.z:.4f},{tq.w:.4f})"
            )

            # Transform the point
            out = tf2_geometry_msgs.do_transform_point(request.point_in, tf)
            out.header.frame_id = target_frame  # make sure the frame is set
            response.point_out = out

            ox = float(out.point.x); oy = float(out.point.y); oz = float(out.point.z)
            self.get_logger().info(
                f"📤 Transformed point frame='{target_frame}'  "
                f"xyz=({ox:.4f}, {oy:.4f}, {oz:.4f}) m ✅"
            )

        except Exception as e:
            self.get_logger().error(f"❌ Transform failed: {e}")

        return response


def main(args=None):
    rclpy.init(args=args)
    node = TransformNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
