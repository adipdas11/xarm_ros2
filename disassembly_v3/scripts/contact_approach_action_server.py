#!/usr/bin/env python3

"""
ContactApproachServer: 
Implements the ApproachUntilContact action server: 
first calls MoveToPose to hover above, then descends under velocity control until effort based contact detection.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from geometry_msgs.msg import TwistStamped, PoseStamped
from sensor_msgs.msg import JointState
from disassembly_v3.action import ApproachUntilContact, MoveToPose


class ContactApproachServer(Node):
    def __init__(self):
        super().__init__('contact_approach_action_server')

        # --- Effort monitoring subscription ---
        self.latest_eff = 0.0
        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_cb,
            10
        )

        # --- Publisher for direct-cartesian velocity descent ---
        self.cmd_pub = self.create_publisher(
            TwistStamped,
            '/servo_server/delta_twist_cmds',
            10
        )

        # --- Action client to call the MoveToPose action server ---
        self.move_client = ActionClient(self, MoveToPose, 'MoveToPose')

        # --- This node’s own action server ---
        self._action_server = ActionServer(
            self,
            ApproachUntilContact,
            'ApproachUntilContact',
            execute_callback=self.execute_cb
        )

        self.get_logger().info('✅ ContactApproachServer ready')

    def joint_cb(self, msg: JointState):
        """
        Callback for /joint_states: updates the maximum absolute effort observed
        among joints 2, 3, and 4.
        """
        try:
            e2 = msg.effort[msg.name.index('joint2')]
            e3 = msg.effort[msg.name.index('joint3')]
            e4 = msg.effort[msg.name.index('joint4')]
            self.latest_eff = max(abs(e2), abs(e3), abs(e4))
        except ValueError:
            # joint name not found in this message
            pass

    def execute_cb(self, goal_handle):
        """
        Execute callback for the ApproachUntilContact action.
        1) Moves above the target via MoveToPose.
        2) Descends straight down until joint effort exceeds the threshold.
        """
        goal = goal_handle.request

        # 1) Build the "above target" pose
        above = PoseStamped()
        above.header = goal.target.header
        above.pose = goal.target.pose
        above.pose.position.z += goal.approach_height

        # Wait for MoveToPose action server
        self.move_client.wait_for_server()

        # 2) Send MoveToPose goal
        move_goal = MoveToPose.Goal()
        move_goal.target            = above
        move_goal.tolerance         = 0.01
        move_goal.max_linear_speed  = goal.velocity_scale
        move_goal.max_angular_speed = 0.0

        send_goal_future = self.move_client.send_goal_async(move_goal)
        rclpy.spin_until_future_complete(self, send_goal_future)
        move_handle = send_goal_future.result()

        if not move_handle.accepted:
            self.get_logger().error('❌ MoveToPose goal was rejected')
            goal_handle.abort()
            return ApproachUntilContact.Result()

        # 3) Wait for the MoveToPose result
        get_result_future = move_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        move_result = get_result_future.result().result

        if not move_result.success:
            self.get_logger().error('❌ MoveToPose execution failed')
            goal_handle.abort()
            return ApproachUntilContact.Result()

        # 4) Descend until contact detected
        self.get_logger().info('⏬ Starting descent until contact')
        loop_rate = self.create_rate(50)  # 50 Hz descent
        contact_pose = PoseStamped()
        contact_pose.header = above.header

        while rclpy.ok():
            # Publish a small downward velocity
            twist = TwistStamped()
            twist.header.stamp = self.get_clock().now().to_msg()
            twist.twist.linear.z = -abs(goal.velocity_scale)
            self.cmd_pub.publish(twist)

            # Provide feedback on current effort
            fb = ApproachUntilContact.Feedback()
            fb.current_effort = float(self.latest_eff)
            goal_handle.publish_feedback(fb)

            # Check for contact
            if self.latest_eff > goal.effort_threshold:
                self.get_logger().info('⚠️ Contact detected via effort threshold')
                # Approximate contact pose as the target pose at contact
                contact_pose.pose = goal.target.pose
                break

            loop_rate.sleep()

        # 5) Stop motion immediately
        zero = TwistStamped()
        zero.header.stamp = self.get_clock().now().to_msg()
        self.cmd_pub.publish(zero)

        # 6) Return the result
        result = ApproachUntilContact.Result()
        result.success = True
        result.contact_pose = contact_pose
        result.message = 'Approach complete'
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
