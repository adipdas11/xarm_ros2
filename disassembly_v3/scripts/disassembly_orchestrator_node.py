#!/usr/bin/env python3
import json
import rclpy
from rclpy.node import Node
from disassembly_v3.srv import GetSequence, GetPartGraph, TransformPose
from disassembly_v3.action import (
    MoveToPose,
    ApproachUntilContact,
    UnscrewScrew
)
from geometry_msgs.msg import PoseStamped

class DisassemblyOrchestrator(Node):
    def __init__(self):
        super().__init__('disassembly_orchestrator')

        # Clients for services & actions
        self.seq_cli    = self.create_client(GetSequence,    'GetSequence')
        self.graph_cli  = self.create_client(GetPartGraph,   'GetPartGraph')
        self.tf_cli     = self.create_client(TransformPose,  'TransformPose')
        self.move_cli   = rclpy.action.ActionClient(self, MoveToPose,            'MoveToPose')
        self.app_cli    = rclpy.action.ActionClient(self, ApproachUntilContact, 'ApproachUntilContact')
        self.uns_cli    = rclpy.action.ActionClient(self, UnscrewScrew,         'UnscrewScrew')

        # wait for everything
        self.seq_cli.wait_for_service()
        self.graph_cli.wait_for_service()
        self.tf_cli.wait_for_service()
        self.move_cli.wait_for_server()
        self.app_cli.wait_for_server()
        self.uns_cli.wait_for_server()

        # define home pose
        self.home = PoseStamped()
        self.home.header.frame_id = 'link_base'
        self.home.pose.position.x = 0.4
        self.home.pose.position.y = 0.0
        self.home.pose.position.z = 0.15
        self.home.pose.orientation.x = 1.0
        self.home.pose.orientation.w = 0.0

        # begin orchestration
        self.get_logger().info('▶️  Starting orchestration…')
        self.start_sequence()

    def start_sequence(self):
        # 1) move to home
        self.call_move(self.home, tol=0.01, lin=0.5, ang=0.2)

        # 2) loop over screws
        while True:
            # fetch sequence
            seq_req = GetSequence.Request()
            seq_res = self.seq_cli.call(seq_req)
            if not seq_res.sequence:
                self.get_logger().info('🎉 Disassembly complete!')
                return

            for screw_id in seq_res.sequence:
                # get graph, extract camera pose
                graph = self.graph_cli.call(GetPartGraph.Request())
                data  = json.loads(graph.json_graph)
                node  = next(n for n in data['nodes'] if n['id']==screw_id)
                # 3D pose in camera frame
                cam_pose = PoseStamped()
                cam_pose.header.frame_id = 'camera_color_optical_frame'
                cam_pose.pose.position.x, cam_pose.pose.position.y, cam_pose.pose.position.z = node['position']
                cam_pose.pose.orientation = self.home.pose.orientation

                # 3) transform pose into base frame
                tf_req = TransformPose.Request()
                tf_req.pose = cam_pose
                tf_req.target_frame = 'link_base'
                tf_res = self.tf_cli.call(tf_req)
                if not tf_res.success:
                    self.get_logger().error(f"TF failed: {tf_res.message}")
                    continue
                base_pose = tf_res.transformed_pose

                # 4) Approach until contact
                app_goal = ApproachUntilContact.Goal()
                app_goal.target = base_pose
                app_goal.approach_height   = 0.02
                app_goal.velocity_scale    = 0.2
                app_goal.effort_threshold  = 1.0
                app_fut = self.app_cli.send_goal_async(app_goal)
                rclpy.spin_until_future_complete(self, app_fut)
                app_res = app_fut.result().result
                if not app_res.success:
                    self.get_logger().error('Approach failed')
                    continue

                # 5) Unscrew
                uns_goal = UnscrewScrew.Goal()
                uns_goal.screw_id          = screw_id
                uns_goal.torque_threshold  = 2.0
                uns_goal.z_retract_step    = 0.001
                uns_goal.timeout           = 15.0
                uns_fut = self.uns_cli.send_goal_async(uns_goal)
                rclpy.spin_until_future_complete(self, uns_fut)
                uns_res = uns_fut.result().result
                if not uns_res.success:
                    self.get_logger().warn('Unscrew may have failed')

                # 6) return home between screws
                self.call_move(self.home, tol=0.01, lin=0.5, ang=0.2)

    def call_move(self, pose, tol, lin, ang):
        goal = MoveToPose.Goal()
        goal.target             = pose
        goal.tolerance          = tol
        goal.max_linear_speed   = lin
        goal.max_angular_speed  = ang
        fut = self.move_cli.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        fut2 = fut.result().result_future
        rclpy.spin_until_future_complete(self, fut2)
        return fut2.result()

def main():
    rclpy.init()
    node = DisassemblyOrchestrator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
