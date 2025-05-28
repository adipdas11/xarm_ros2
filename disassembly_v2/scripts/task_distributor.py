#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from xarm_msgs.srv import PlanPose, PlanExec
from std_msgs.msg import String
import json
import time

class TaskDistributorNode(Node):
    MAX_RETRIES = 3

    def __init__(self):
        super().__init__('task_distributor_node')

        # Service Clients for both arms
        self.arm_r_pose_plan_client = self.create_client(PlanPose, '/R_/xarm_pose_plan')
        self.arm_r_exec_plan_client = self.create_client(PlanExec, '/R_/xarm_exec_plan')
        self.arm_l_pose_plan_client = self.create_client(PlanPose, '/L_/xarm_pose_plan')
        self.arm_l_exec_plan_client = self.create_client(PlanExec, '/L_/xarm_exec_plan')

        # Topic subscription for Disassembly Sequence (DSP)
        self.create_subscription(String, 'disassembly_sequence', self.dsp_callback, 10)

        self.get_logger().info('TaskDistributorNode initialized.')

    def dsp_callback(self, msg: String):
        # Parse the received disassembly sequence
        try:
            task_sequence = json.loads(msg.data)
            self.get_logger().info(f"Received disassembly sequence: {json.dumps(task_sequence, indent=2)}")

            # Go through each task in the sequence
            for task in task_sequence['tasks']:
                part = task['part']
                pose = task['pose']
                task_type = task['task_type']
                arm_to_use = task['arm']  # "L-arm" or "R-arm" for manipulation arm

                # Print task details
                self.get_logger().info(f"Distributing task: {task_type} for part {part}")

                # Send pose to the correct arm
                if arm_to_use == 'L-arm':
                    self.send_pose(self.arm_l_pose_plan_client, self.arm_l_exec_plan_client, pose, "L-arm")
                elif arm_to_use == 'R-arm':
                    self.send_pose(self.arm_r_pose_plan_client, self.arm_r_exec_plan_client, pose, "R-arm")
                
                # Perform the task (e.g., unscrewing, picking)
                self.perform_task(task_type, arm_to_use)

        except Exception as e:
            self.get_logger().error(f"Error processing disassembly sequence: {e}")

    def send_pose(self, pose_plan_client, exec_plan_client, pose, arm_name):
        # Reset retry count for a new pose
        retry_count = 0

        while retry_count < self.MAX_RETRIES:
            try:
                # Send Pose using PlanPose service
                if pose_plan_client.wait_for_service(timeout_sec=1.0):
                    request = PlanPose.Request()
                    request.pose.position.x = pose['position']['x']
                    request.pose.position.y = pose['position']['y']
                    request.pose.position.z = pose['position']['z']
                    request.pose.orientation.x = pose['orientation']['x']
                    request.pose.orientation.y = pose['orientation']['y']
                    request.pose.orientation.z = pose['orientation']['z']
                    request.pose.orientation.w = pose['orientation']['w']

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

    def perform_task(self, task_type, arm_name):
        """ Execute the task for the given arm. """
        self.get_logger().info(f"Performing task: {task_type} on {arm_name}")

        # For now, let's simulate the task execution. 
        # In practice, you would call the relevant service for unscrewing, picking, etc.
        if task_type == 'unscrewing':
            self.get_logger().info(f"Executing unscrewing task for {arm_name}")
            # Implement unscrewing logic here
        elif task_type == 'picking':
            self.get_logger().info(f"Executing picking task for {arm_name}")
            # Implement picking logic here
        elif task_type == 'placing':
            self.get_logger().info(f"Executing placing task for {arm_name}")
            # Implement placing logic here
        else:
            self.get_logger().warn(f"Unknown task type: {task_type}")


def main(args=None):
    rclpy.init(args=args)
    node = TaskDistributorNode()
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
