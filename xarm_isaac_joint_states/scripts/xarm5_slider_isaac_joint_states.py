#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import message_filters

class XArm5JointCommander(Node):
    def __init__(self):
        super().__init__('xarm5_joint_commander')
        # Publisher for the commanded joint state
        self.pub = self.create_publisher(JointState, 'xarm5_joint_command', 10)

        # Subscribers (synchronized) to the two JointState topics
        sub_js = message_filters.Subscriber(self, JointState, 'joint_states')
        sub_lm = message_filters.Subscriber(self, JointState, 'linear_motor_joint_states')
        ts = message_filters.ApproximateTimeSynchronizer(
            [sub_js, sub_lm], queue_size=10, slop=0.1
        )
        ts.registerCallback(self.joint_callback)

    def joint_callback(self, js_msg: JointState, lm_msg: JointState):
        # Helper to lookup a named joint in a JointState message
        def get_pos(msg: JointState, name: str) -> float:
            try:
                idx = msg.name.index(name)
                return msg.position[idx]
            except ValueError:
                self.get_logger().error(f"Joint '{name}' not found in message")
                return 0.0

        # 1) slider_joint from linear_motor_joint_states
        slider = get_pos(lm_msg, 'slider_joint')

        # 2) joint1…joint5 and drive_joint from joint_states
        joint1 = get_pos(js_msg, 'joint1')
        joint2 = get_pos(js_msg, 'joint2')
        joint3 = get_pos(js_msg, 'joint3')
        joint4 = get_pos(js_msg, 'joint4')
        joint5 = get_pos(js_msg, 'joint5')
        drive  = get_pos(js_msg, 'drive_joint')

        # 3) synthesize knuckle/finger joints by sign-flipping drive_joint
        left_inner_knuckle__joint   = -drive
        right_outer_knuckle_joint   =  drive
        right_inner_knuckle__joint  =  drive
        left_finger_joint           =  drive
        right_finger_joint          = -drive

        # 4) build and publish the combined JointState
        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = [
            'slider_joint',
            'joint1', 'joint2', 'joint3', 'joint4', 'joint5',
            'drive_joint',
            'left_inner_knuckle__joint',
            'right_outer_knuckle_joint',
            'right_inner_knuckle__joint',
            'left_finger_joint',
            'right_finger_joint'
        ]
        out.position = [
            slider,
            joint1, joint2, joint3, joint4, joint5,
            -drive,
            left_inner_knuckle__joint,
            right_outer_knuckle_joint,
            right_inner_knuckle__joint,
            left_finger_joint,
            right_finger_joint
        ]

        self.pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = XArm5JointCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
