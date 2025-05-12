#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster


class SliderVisualizer(Node):
    def __init__(self):
        super().__init__('slider_visualizer')
        self.br = TransformBroadcaster(self)
        self.marker_pub = self.create_publisher(Marker, 'linear_motor_marker', 10)

        # subscribe to the joint_states topic published by the motor node
        self.joint_sub = self.create_subscription(
            JointState,
            'linear_motor_joint_states',  # match the publisher from your motor node
            self.joint_callback,
            10
        )
        self.joint_position = 0.0  # in metres

        self.timer = self.create_timer(0.01, self.update)

        # fixed geometry offsets (all metres)
        self.base_z           = 0.095 / 2.0
        self.base_y           = 0.0
        self.slide_z          = 0.0535
        self.slide_y_half     = 0.160 / 2.0
        self.slide_offset_y   = 0.11

    def joint_callback(self, msg: JointState):
        if 'slider_joint' in msg.name:
            idx = msg.name.index('slider_joint')
            pos_m = msg.position[idx]    # now already in metres
            # clamp to [0, 0.7] metres
            self.joint_position = max(0.0, min(0.7, pos_m))

    def update(self):
        now = self.get_clock().now().to_msg()
        self._broadcast_transforms(now)
        self._publish_markers(now)

    def _broadcast_transforms(self, stamp):
        # world -> linear_motor_base_link
        tf_fixed = TransformStamped()
        tf_fixed.header.stamp = stamp
        tf_fixed.header.frame_id = 'world'
        tf_fixed.child_frame_id = 'linear_motor_base_link'
        tf_fixed.transform.translation.x = 0.0
        tf_fixed.transform.translation.y = self.base_y
        tf_fixed.transform.translation.z = self.base_z
        tf_fixed.transform.rotation.w = 1.0
        self.br.sendTransform(tf_fixed)

        # linear_motor_base_link -> robot_base (the carriage)
        tf_slide = TransformStamped()
        tf_slide.header.stamp = stamp
        tf_slide.header.frame_id = 'linear_motor_base_link'
        tf_slide.child_frame_id = 'robot_base'
        tf_slide.transform.translation.x = 0.0
        tf_slide.transform.translation.y = (
            self.joint_position
            - self.base_y
            + self.slide_y_half
            + self.slide_offset_y
        )
        tf_slide.transform.translation.z = self.slide_z
        tf_slide.transform.rotation.w = 1.0
        self.br.sendTransform(tf_slide)

    def _publish_markers(self, stamp):
        # Base cuboid on linear_motor_base_link
        m1 = Marker(
            header=Marker().header.__class__(stamp=stamp, frame_id='linear_motor_base_link'),
            ns='linear_motor_base',
            id=0,
            type=Marker.CUBE,
            action=Marker.ADD,
            scale=Marker().scale.__class__(x=0.140, y=1.070, z=0.095),
            pose=Marker().pose.__class__(orientation=Marker().pose.orientation.__class__(w=1.0),
                                         position=Marker().pose.position.__class__(y=1.070 / 2.0)),
            color=Marker().color.__class__(r=0.0, g=0.0, b=1.0, a=1.0),
        )
        self.marker_pub.publish(m1)

        # Carriage cuboid on robot_base
        m2 = Marker(
            header=Marker().header.__class__(stamp=stamp, frame_id='robot_base'),
            ns='robot_base',
            id=1,
            type=Marker.CUBE,
            action=Marker.ADD,
            scale=Marker().scale.__class__(x=0.135, y=0.160, z=0.012),
            pose=Marker().pose.__class__(orientation=Marker().pose.orientation.__class__(w=1.0)),
            color=Marker().color.__class__(r=0.0, g=1.0, b=0.0, a=1.0),
        )
        self.marker_pub.publish(m2)


def main(args=None):
    rclpy.init(args=args)
    node = SliderVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
