#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from xarm_msgs.srv import PlanPose, PlanExec
import json
import time

class TaskDistributorNode(Node):
    def __init__(self):
        super().__init__('task_distributor_node')

        # Subscriber to disassembly sequence tasks
        self.seq_sub = self.create_subscription(String, 'disassembly_sequence', self.seq_callback, 10)
        self.graph_sub = self.create_subscription(String, 'part_graph', self.graph_callback, 10)

        # Publishers for task distribution (Optional)
        self.manip_pub = self.create_publisher(String, 'manipulation_tasks', 10)
        self.unscrew_pub = self.create_publisher(String, 'unscrewing_tasks', 10)

        # ROS2 Service Clients for arms
        self.get_logger().info("Setting up service clients...")
        self.arm_r_pose_plan_client = self.create_client(PlanPose, '/R_/xarm_pose_plan')
        self.arm_r_exec_plan_client = self.create_client(PlanExec, '/R_/xarm_exec_plan')
        self.arm_l_pose_plan_client = self.create_client(PlanPose, '/L_/xarm_pose_plan')
        self.arm_l_exec_plan_client = self.create_client(PlanExec, '/L_/xarm_exec_plan')

        self.get_logger().info('TaskDistributorNode initialized.')

        self.part_info = {}  # part_id -> label
        self.current_sequence = []

    def graph_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.part_info = {node['id']: node['label'] for node in data.get('nodes', [])}
        except Exception as e:
            self.get_logger().error(f'Failed to parse part graph: {e}')

    def seq_callback(self, msg):
        try:
            sequence = json.loads(msg.data)
            self.current_sequence = sequence
            manip_tasks = []
            unscrew_tasks = []

            # Split tasks into manipulation and unscrewing tasks
            for pid in sequence:
                label = self.part_info.get(pid, '').lower()
                if 'screw' in label:
                    unscrew_tasks.append(pid)
                else:
                    manip_tasks.append(pid)

            manip_msg = String()
            manip_msg.data = json.dumps(manip_tasks)
            self.manip_pub.publish(manip_msg)

            unscrew_msg = String()
            unscrew_msg.data = json.dumps(unscrew_tasks)
            self.unscrew_pub.publish(unscrew_msg)

            self.get_logger().info(f'Distributed {len(manip_tasks)} manipulation and {len(unscrew_tasks)} unscrewing tasks.')

            # Execute tasks one by one (assuming they are ordered)
            self.execute_tasks(manip_tasks, unscrew_tasks)

        except Exception as e:
            self.get_logger().error(f'Failed to process sequence: {e}')

    def execute_tasks(self, manip_tasks, unscrew_tasks):
        for part_id in manip_tasks:
            self.get_logger().info(f"Executing manipulation task for {part_id}")
            pose = self.get_part_pose(part_id)
            self.execute_manipulation_task(pose)

        for part_id in unscrew_tasks:
            self.get_logger().info(f"Executing unscrewing task for {part_id}")
            pose = self.get_part_pose(part_id)
            self.execute_unscrewing_task(pose)

    def get_part_pose(self, part_id):
        # Fetch pose data from the graph for this part
        # Here I'm assuming the part graph has 'position' stored as [x, y, z]
        part_position = self.part_info.get(part_id, {}).get('position', [0, 0, 0])
        return part_position  # Return pose as [x, y, z]

    def execute_manipulation_task(self, pose):
        # Send pose to arm's service
        if self.arm_r_pose_plan_client.wait_for_service(timeout_sec=1.0):
            request = PlanPose.Request()
            request.position = pose
            self.arm_r_pose_plan_client.call_async(request)
            self.get_logger().info(f"Sent pose {pose} to /R_/xarm_pose_plan")
        else:
            self.get_logger().warn("/R_/xarm_pose_plan service not available")

        # Execute the plan after the arm reaches the pose
        time.sleep(2)  # wait for arm to reach the pose (this can be replaced with actual feedback)
        if self.arm_r_exec_plan_client.wait_for_service(timeout_sec=1.0):
            request = PlanExec.Request()
            request.data = 1  # Assuming '1' triggers the execution of the plan
            self.arm_r_exec_plan_client.call_async(request)
            self.get_logger().info(f"Executed manipulation task for pose {pose}")
        else:
            self.get_logger().warn("/R_/xarm_exec_plan service not available")

    def execute_unscrewing_task(self, pose):
        # Send pose to tooling arm's service
        if self.arm_l_pose_plan_client.wait_for_service(timeout_sec=1.0):
            request = PlanPose.Request()
            request.position = pose
            self.arm_l_pose_plan_client.call_async(request)
            self.get_logger().info(f"Sent pose {pose} to /L_/xarm_pose_plan")
        else:
            self.get_logger().warn("/L_/xarm_pose_plan service not available")

        # Execute the unscrewing plan after the tooling arm reaches the pose
        time.sleep(2)  # wait for arm to reach the pose (this can be replaced with actual feedback)
        if self.arm_l_exec_plan_client.wait_for_service(timeout_sec=1.0):
            request = PlanExec.Request()
            request.data = 1  # Assuming '1' triggers the unscrewing action
            self.arm_l_exec_plan_client.call_async(request)
            self.get_logger().info(f"Executed unscrewing task for pose {pose}")
        else:
            self.get_logger().warn("/L_/xarm_exec_plan service not available")

def main(args=None):
    rclpy.init(args=args)
    node = TaskDistributorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
