#!/usr/bin/env python3
"""
ROS2 node for controlling and monitoring a direct-drive linear motor via xArmSDK.
Publishes joint states (in metres) on `linear_motor_joint_states` and listens for
position commands (in metres) on `linear_motor_joint_commands`. Falls back to a
fake hardware implementation if connection to the real motor fails.

IP is taken from the ROS parameter `ip`.
"""
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from xarm.wrapper import XArmAPI


class FakeXArm:
    """A fake linear motor interface for testing without hardware."""
    def __init__(self, ip=None):
        self.pos = 0.0  # stored in mm

    def motion_enable(self, ena): return 0
    def clean_error(self): return 0
    def set_mode(self, mode): return 0
    def set_state(self, state): return 0

    def set_linear_track_back_origin(self, wait=False):
        self.pos = 0.0
        return 0

    def get_linear_track_on_zero(self):
        return 0, int(self.pos == 0.0)

    def set_linear_track_enable(self, ena): return 0
    def set_linear_track_speed(self, speed): return 0

    def get_linear_track_pos(self):
        # returns (code, position_mm)
        return 0, self.pos

    def set_linear_track_pos(self, target_mm, wait=False):
        # target_mm: float
        self.pos = float(target_mm)
        return 0


class LinearMotorNode(Node):
    def __init__(self):
        super().__init__('linear_motor_node')

        # declare and read the 'ip' parameter
        self.declare_parameter('ip', '192.168.1.239')
        ip = self.get_parameter('ip').get_parameter_value().string_value

        try:
            self.arm = XArmAPI(ip)
            self.get_logger().info(f"Connected to real xArm at {ip}")
        except Exception as e:
            self.get_logger().error(f"Could not connect to {ip}: {e}. Using fake hardware.")
            self.arm = FakeXArm(ip)

        # initialize hardware
        time.sleep(1)
        self.arm.set_linear_track_back_origin(wait=True)
        self.arm.get_linear_track_on_zero()
        self.arm.set_linear_track_enable(True)
        self.arm.set_linear_track_speed(500)  # mm/s

        # ROS interfaces
        self.state_pub = self.create_publisher(JointState, 'linear_motor_joint_states', 10)
        self.cmd_sub   = self.create_subscription(
            JointState,
            'linear_motor_joint_commands',
            self.cmd_callback,
            10
        )
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        code, pos_mm = self.arm.get_linear_track_pos()
        if code == 0:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = ['slider_joint']
            # convert mm -> m for ROS message
            msg.position = [pos_mm / 1000.0]
            self.state_pub.publish(msg)
        else:
            self.get_logger().error(f"Failed to read track position, code={code}")

    def cmd_callback(self, msg: JointState):
        if 'slider_joint' in msg.name:
            idx = msg.name.index('slider_joint')
            target_m  = msg.position[idx]
            # clamp to [0, 0.7] m
            target_m = max(0.0, min(0.7, target_m))
            target_mm = target_m * 1000.0
            code = self.arm.set_linear_track_pos(target_mm, wait=True)
            if code != 0:
                self.get_logger().error(
                    f"Failed to set track to {target_mm} mm (requested {target_m} m), code={code}"
                )
        else:
            self.get_logger().warn("Received JointState without 'slider_joint'")


def main(args=None):
    rclpy.init(args=args)
    node = LinearMotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down linear motor node...')
    finally:
        node.arm.set_linear_track_enable(False)
        node.arm.motion_enable(False)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
