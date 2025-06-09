#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from std_msgs.msg import Int8
from disassembly_v3.action import UnscrewScrew

class UnscrewActionServer(Node):
    def __init__(self):
        super().__init__('unscrew_action_server')
        # Publisher to the tool motor
        self.tool_pub = self.create_publisher(Int8, '/tool_cmd', 10)

        # For monitoring efforts (could subscribe if needed)
        self.latest_eff = 0.0

        # Action server
        self._action_server = ActionServer(
            self,
            UnscrewScrew,
            'UnscrewScrew',
            execute_callback=self.execute_cb
        )
        self.get_logger().info('✅ UnscrewActionServer ready')

    def execute_cb(self, goal_handle):
        goal = goal_handle.request
        start_time = time.time()
        goal_handle.publish_feedback(UnscrewScrew.Feedback(joint_efforts=[0.0,0.0,0.0]))

        # start motor
        self.tool_pub.publish(Int8(data=-1))

        while time.time() - start_time < goal.timeout:
            # TODO: read real joint efforts
            efforts = [0.0, 0.0, 0.0]  # replace with real data
            # feedback
            feedback = UnscrewScrew.Feedback(joint_efforts=efforts)
            goal_handle.publish_feedback(feedback)

            # if spike > torque_threshold → retract
            if max(abs(e) for e in efforts) > goal.torque_threshold:
                # retract in z by z_retract_step
                # you could call the approach action with zero descent
                self.get_logger().info('🔄 Retracting slightly')
                # TODO: implement small z retraction
            time.sleep(0.5)

        # stop motor
        self.tool_pub.publish(Int8(data=0))

        # finish
        res = UnscrewScrew.Result()
        res.success = True
        res.message = 'Unscrew completed'
        goal_handle.succeed()
        return res


def main():
    rclpy.init()
    node = UnscrewActionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
