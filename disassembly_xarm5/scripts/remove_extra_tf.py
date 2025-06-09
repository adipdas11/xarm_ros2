#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from tf2_msgs.msg import TFMessage

class TfStaticFilter(Node):
    def __init__(self):
        super().__init__('tf_static_filter')

        # QoSProfile that matches /tf_static publishers (latched/transient_local, reliable).
        qos = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        # Subscribe to the “real” /tf_static with transient_local durability.
        self.sub = self.create_subscription(
            TFMessage,
            '/tf_static',
            self.callback,
            qos
        )

        # Re-publisher on /tf_static (overwrite) with the same QoSProfile.
        self.pub = self.create_publisher(
            TFMessage,
            '/tf_static',
            qos
        )

        self.get_logger().info('✅ tf_static_filter initialized (listening on /tf_static).')

    def callback(self, msg: TFMessage):
        # Build a new TFMessage containing only the transforms we want to keep.
        out_msg = TFMessage()
        for t in msg.transforms:
            if (t.header.frame_id == 'camera_color_frame' and
                t.child_frame_id == 'camera_color_optical_frame'):
                # Drop the RealSense camera driver’s optical‐frame static.
                continue
            out_msg.transforms.append(t)

        # Re‐publish the filtered set of static transforms back onto /tf_static.
        self.pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TfStaticFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
