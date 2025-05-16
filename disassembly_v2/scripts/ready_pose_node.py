#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from xarm_msgs.srv import PlanPose, PlanExec
from geometry_msgs.msg import Pose
import time

class ReadyPoseNode(Node):
    MAX_RETRIES = 3

    def __init__(self):
        super().__init__('ready_pose_node')

        # Service Clients for the arms
        self.arm_r_pose_plan_client = self.create_client(PlanPose, '/R_/xarm_pose_plan')
        self.arm_r_exec_plan_client = self.create_client(PlanExec, '/R_/xarm_exec_plan')
        self.arm_l_pose_plan_client = self.create_client(PlanPose, '/L_/xarm_pose_plan')
        self.arm_l_exec_plan_client = self.create_client(PlanExec, '/L_/xarm_exec_plan')

        self.get_logger().info('ReadyPoseNode initialized.')

        # Define ready poses for both arms
        self.pose_l = Pose()
        self.pose_l.position.x = 0.43
        self.pose_l.position.y = 0.0
        self.pose_l.position.z = 0.43
        self.pose_l.orientation.x = 1.0
        self.pose_l.orientation.y = 0.0
        self.pose_l.orientation.z = 0.0
        self.pose_l.orientation.w = 0.0

        self.pose_r = Pose()
        self.pose_r.position.x = 0.47
        self.pose_r.position.y = 1.0
        self.pose_r.position.z = 0.46
        self.pose_r.orientation.x = 1.0
        self.pose_r.orientation.y = 0.0
        self.pose_r.orientation.z = 0.0
        self.pose_r.orientation.w = 0.0

        # Send the poses to both arms
        self.send_ready_poses()

    def send_ready_poses(self):
        # Send L-arm pose
        self.get_logger().info('Sending ready pose to L-arm...')
        self.send_pose(self.arm_l_pose_plan_client, self.arm_l_exec_plan_client, self.pose_l, "L-arm")

        # Send R-arm pose
        self.get_logger().info('Sending ready pose to R-arm...')
        self.send_pose(self.arm_r_pose_plan_client, self.arm_r_exec_plan_client, self.pose_r, "R-arm")

    def send_pose(self, pose_plan_client, exec_plan_client, pose, arm_name):
        # Reset retry count for a new pose
        retry_count = 0

        while retry_count < self.MAX_RETRIES:
            try:
                # Send Pose using PlanPose service
                if pose_plan_client.wait_for_service(timeout_sec=1.0):
                    request = PlanPose.Request()
                    # Set position
                    request.target.position.x = pose.position.x
                    request.target.position.y = pose.position.y
                    request.target.position.z = pose.position.z
                    # Set orientation
                    request.target.orientation.x = pose.orientation.x
                    request.target.orientation.y = pose.orientation.y
                    request.target.orientation.z = pose.orientation.z
                    request.target.orientation.w = pose.orientation.w

                    pose_plan_client.call_async(request)
                    self.get_logger().info(f"{arm_name} Pose sent: {pose}")

                    # Wait for the arm to reach the pose
                    time.sleep(2)  # can be replaced with actual feedback mechanism

                    # Execute the pose plan with PlanExec service
                    if exec_plan_client.wait_for_service(timeout_sec=1.0):
                        exec_request = PlanExec.Request()  # No 'data' field in PlanExec.Request
                        exec_plan_client.call_async(exec_request)
                        self.get_logger().info(f"{arm_name} pose execution started")
                        return  # Pose and execution completed, exit the loop
                    else:
                        self.get_logger().warn(f"{arm_name} /xarm_exec_plan service not available")
                else:
                    self.get_logger().warn(f"{arm_name} /xarm_pose_plan service not available")
                
                retry_count += 1
                self.get_logger().warn(f"[{arm_name}] Retry {retry_count}/{self.MAX_RETRIES}...")

            except Exception as e:
                self.get_logger().error(f'[{arm_name}] Exception during sending pose or execution: {e}')
                retry_count += 1

        self.get_logger().error(f"[{arm_name}] Pose execution failed after {self.MAX_RETRIES} retries")
        # No shutdown here to avoid context errors.
        # Remove rclpy.shutdown() call from the retry loop

def main(args=None):
    rclpy.init(args=args)
    node = ReadyPoseNode()
    executor = MultiThreadedExecutor()
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
