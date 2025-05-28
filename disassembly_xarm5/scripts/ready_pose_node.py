#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from xarm_msgs.srv import PlanPose, PlanExec
from geometry_msgs.msg import Pose

class ReadyPoseNode(Node):
    MAX_RETRIES = 3

    def __init__(self):
        super().__init__('ready_pose_node')
        self.pose_plan_client = self.create_client(PlanPose, '/xarm_pose_plan')
        self.exec_plan_client = self.create_client(PlanExec, '/xarm_exec_plan')

        # your ready‐pose
        self.ready_pose = Pose()
        self.ready_pose.position.x = 0.4
        self.ready_pose.position.y = 0.0
        self.ready_pose.position.z = 0.15
        self.ready_pose.orientation.x = 1.0
        self.ready_pose.orientation.y = 0.0
        self.ready_pose.orientation.z = 0.0
        self.ready_pose.orientation.w = 0.0

        self.send_ready_pose()

    def send_ready_pose(self):
        retries = 0
        while retries < self.MAX_RETRIES:
            # 1) Plan
            if not self.pose_plan_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn('Service /xarm_pose_plan not available')
                retries += 1
                continue

            plan_req = PlanPose.Request()
            plan_req.target = self.ready_pose

            self.get_logger().info(f'Calling /xarm_pose_plan → {self.ready_pose}')
            plan_future = self.pose_plan_client.call_async(plan_req)
            rclpy.spin_until_future_complete(self, plan_future, timeout_sec=2.0)
            response = plan_future.result()

            if response is None:
                self.get_logger().warn('No response from PlanPose')
                retries += 1
                continue

            # **Use the correct field** (e.g. response.success)
            if not response.success:  
                self.get_logger().warn(f'PlanPose failed: {response.message}')
                retries += 1
                continue

            # 2) Execute
            if not self.exec_plan_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn('Service /xarm_exec_plan not available')
                retries += 1
                continue

            exec_req = PlanExec.Request()
            exec_req.wait = True      # don't forget to set wait!
            self.get_logger().info('Calling /xarm_exec_plan')
            exec_future = self.exec_plan_client.call_async(exec_req)
            rclpy.spin_until_future_complete(self, exec_future, timeout_sec=5.0)
            exec_resp = exec_future.result()

            if exec_resp is None or not exec_resp.success:
                self.get_logger().warn(f'PlanExec failed: {exec_resp}')
                retries += 1
                continue

            self.get_logger().info('Pose planned and executed successfully!')
            return

        self.get_logger().error(f'Failed after {self.MAX_RETRIES} attempts.')

def main(args=None):
    rclpy.init(args=args)
    node = ReadyPoseNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
