#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from geometry_msgs.msg import TwistStamped, PoseStamped
from sensor_msgs.msg import JointState
from disassembly_v3.action import ApproachUntilContact, MoveToPose

class ContactApproachServer(Node):
    def __init__(self):
        super().__init__('contact_approach_action_server')
        # Effort monitoring
        self.latest_eff = 0.0
        self.create_subscription(JointState, '/joint_states',
                                 self.joint_cb, 10)

        # Publisher for direct velocity descent
        self.cmd_pub = self.create_publisher(TwistStamped,
                                             '/servo_server/delta_twist_cmds', 10)

        # Action client for MoveToPose
        self.move_client = ActionClient(self, MoveToPose, 'MoveToPose')

        # Our own action server
        self._action_server = ActionServer(
            self,
            ApproachUntilContact,
            'ApproachUntilContact',
            execute_callback=self.execute_cb
        )
        self.get_logger().info('✅ ContactApproachServer ready')

    def joint_cb(self, msg: JointState):
        # combine efforts of joints 2–4
        e2 = msg.effort[msg.name.index('joint2')]
        e3 = msg.effort[msg.name.index('joint3')]
        e4 = msg.effort[msg.name.index('joint4')]
        self.latest_eff = max(abs(e2), abs(e3), abs(e4))

    def execute_cb(self, goal_handle):
        goal = goal_handle.request

        # 1) Move above target
        above = PoseStamped()
        above.header = goal.target.header
        above.pose = goal.target.pose
        above.pose.position.z += goal.approach_height

        # wait for MoveToPose available
        self.move_client.wait_for_server()
        move_goal = MoveToPose.Goal()
        move_goal.target            = above
        move_goal.tolerance         = 0.01
        move_goal.max_linear_speed  = goal.velocity_scale
        move_goal.max_angular_speed = 0.0

        # send and wait
        fh = self.move_client.send_goal_async(move_goal)
        rclpy.spin_until_future_complete(self, fh)
        if not fh.result().accepted:
            goal_handle.abort()
            return ApproachUntilContact.Result()

        res_f = fh.result().result_future
        rclpy.spin_until_future_complete(self, res_f)
        if not res_f.result().success:
            goal_handle.abort()
            return ApproachUntilContact.Result()

        # 2) Descend until contact
        rate = self.create_rate(50)
        start_time = self.get_clock().now().nanoseconds * 1e-9
        contact_pose = PoseStamped()
        contact_pose.header = above.header

        while rclpy.ok():
            elapsed = self.get_clock().now().nanoseconds * 1e-9 - start_time
            if elapsed > goal_timeout:
                break

            # small downward velocity
            twist = TwistStamped()
            twist.header.stamp = self.get_clock().now().to_msg()
            twist.twist.linear.z = -goal.velocity_scale
            self.cmd_pub.publish(twist)

            # feedback current effort
            fb = ApproachUntilContact.Feedback()
            fb.current_effort = float(self.latest_eff)
            goal_handle.publish_feedback(fb)

            # when effort crosses threshold → assume contact
            if self.latest_eff > goal.effort_threshold:
                # capture current pose via TF if you like
                contact_pose.pose = goal.target.pose  # approximate
                break

            rate.sleep()

        # stop motion
        zero = TwistStamped()
        zero.header.stamp = self.get_clock().now().to_msg()
        self.cmd_pub.publish(zero)

        # finish
        result = ApproachUntilContact.Result()
        result.success = (self.latest_eff > goal.effort_threshold)
        result.contact_pose = contact_pose
        goal_handle.succeed()
        return result


def main():
    rclpy.init()
    node = ContactApproachServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
