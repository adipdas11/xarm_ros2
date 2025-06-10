#!/usr/bin/env python3

"""
PoseTransformService: 
Provides the TransformPose service, wrapping TF2 lookup/transform to convert PoseStamped between frames.
"""

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from disassembly_v3.srv import TransformPose
import tf2_geometry_msgs  # for transform interoperability

class PoseTransformService(Node):
    def __init__(self):
        super().__init__('pose_transform_service')
        # TF buffer & listener
        self.tf_buffer = Buffer()
        TransformListener(self.tf_buffer, self)

        # Service
        self.create_service(
            TransformPose,
            'TransformPose',
            self.handle_transform
        )
        self.get_logger().info('✅ PoseTransformService ready')

    def handle_transform(self, req, resp):
        try:
            # Attempt the transform
            tf_stamped = self.tf_buffer.transform(
                req.pose,
                req.target_frame,
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            resp.transformed_pose = tf_stamped
            resp.success = True
            resp.message = ''
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            resp.success = False
            resp.message = str(e)
        return resp


def main():
    rclpy.init()
    node = PoseTransformService()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
