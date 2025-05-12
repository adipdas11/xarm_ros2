#!/usr/bin/env python3

import sys
import time
import math

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose

from xarm_msgs.srv import PlanPose, PlanExec, PlanJoint
from ufactory_linear_motor_description.srv import MoveLinearMotor

from scipy.spatial.transform import Rotation as SciRot


def yaw_only_quaternion(marker_q, fixed_roll=0.0, fixed_pitch=0.0):
    """
    Take a geometry_msgs Quaternion `marker_q`,
    extract its yaw, and return a new quaternion
    with fixed roll/pitch and that yaw.
    """
    # build SciPy rotation from marker quaternion
    r = SciRot.from_quat([
        marker_q.x,
        marker_q.y,
        marker_q.z,
        marker_q.w
    ])
    roll, pitch, yaw = r.as_euler('xyz', degrees=False)
    # rebuild with fixed roll/pitch and extracted yaw
    qx, qy, qz, qw = SciRot.from_euler(
        'xyz',
        [fixed_roll, fixed_pitch, yaw],
        degrees=False
    ).as_quat()
    return qx, qy, qz, qw


class PickPlaceNode(Node):
    MAX_RETRIES = 3

    def __init__(self):
        super().__init__('pick_place_node')

        # Define pick→place ID pairs
        self.pairs = [(0, 4), (1, 5), (2, 6)]
        self.current_index = 0

        # track retries per tag
        self.retry_counts = {}

        # Service clients
        self.pose_cli      = self.create_client(PlanPose,       '/xarm_pose_plan')
        self.exec_cli      = self.create_client(PlanExec,       '/xarm_exec_plan')
        self.grip_plan_cli = self.create_client(PlanJoint,      '/xarm_gripper_joint_plan')
        self.grip_exec_cli = self.create_client(PlanExec,       '/xarm_gripper_exec_plan')
        self.linear_cli    = self.create_client(MoveLinearMotor, '/move_linear_motor')

        # verify services
        for cli, name in [
            (self.pose_cli, '/xarm_pose_plan'),
            (self.exec_cli, '/xarm_exec_plan'),
            (self.grip_plan_cli, '/xarm_gripper_joint_plan'),
            (self.grip_exec_cli, '/xarm_gripper_exec_plan'),
            (self.linear_cli, '/move_linear_motor')
        ]:
            if not cli.wait_for_service(timeout_sec=5.0):
                self.get_logger().error(f'Service {name} unavailable, shutting down')
                rclpy.shutdown()
                sys.exit(1)

        # subscribe to markers
        self.marker_sub   = self.create_subscription(
            Marker, '/aruco/cube_marker', self.marker_callback, 10
        )
        self.marker_poses = {}    # id -> Pose

        # state for each pair
        self.stage          = 0      # 0=waiting to pick, 1=await linear→place
        self.linear_done    = False
        self.linear_target  = None

        # define home pose
        self.home = Pose()
        self.home.position.x    = 0.37
        self.home.position.y    = 0.0
        self.home.position.z    = 0.35
        self.home.orientation.x = 1.0
        self.home.orientation.y = 0.0
        self.home.orientation.z = 0.0
        self.home.orientation.w = 0.0

        # drive state machine
        self.create_timer(0.1, self.state_machine_callback)
        self.get_logger().info('PickPlaceNode initialized, waiting for marker ID 0')

    def marker_callback(self, msg: Marker):
        # cache latest marker poses
        self.marker_poses[msg.id] = msg.pose

    # --------------------
    # Async service methods with retry limits
    # --------------------

    def send_pose(self, pose: Pose, tag: str):
        # reset retry count for new tag
        if tag not in self.retry_counts:
            self.retry_counts[tag] = 0

        # log the requested pose
        self.get_logger().info(
            f'[{tag}] PlanPose target position=({pose.position.x:.3f}, '
            f'{pose.position.y:.3f}, {pose.position.z:.3f}), '
            f'orientation=({pose.orientation.x:.3f}, '
            f'{pose.orientation.y:.3f}, {pose.orientation.z:.3f}, '
            f'{pose.orientation.w:.3f})'
        )

        self.current_pose = pose
        req = PlanPose.Request()
        req.target = pose
        fut = self.pose_cli.call_async(req)
        fut.add_done_callback(lambda f: self.on_pose_response(f, tag))

    def on_pose_response(self, future, tag: str):
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().error(f'[{tag}] PlanPose exception: {e}')
            return

        self.get_logger().info(f'[{tag}] PlanPose response: success={resp.success}')
        time.sleep(1.0)

        if resp.success:
            self.retry_counts[tag] = 0
            self.send_exec(tag)
        else:
            self.retry_counts[tag] += 1
            if self.retry_counts[tag] < self.MAX_RETRIES:
                self.get_logger().warn(f'[{tag}] PlanPose failed, retry {self.retry_counts[tag]}/{self.MAX_RETRIES}')
                time.sleep(1.0)
                self.send_pose(self.current_pose, tag)
            else:
                self.get_logger().error(f'[{tag}] PlanPose failed {self.MAX_RETRIES} times, shutting down')
                rclpy.shutdown()

    def send_exec(self, tag: str):
        req = PlanExec.Request()
        req.wait = True
        fut = self.exec_cli.call_async(req)
        fut.add_done_callback(lambda f: self.on_exec_response(f, tag))

    def on_exec_response(self, future, tag: str):
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().error(f'[{tag}] PlanExec exception: {e}')
            return

        self.get_logger().info(f'[{tag}] PlanExec response: success={resp.success}')
        time.sleep(1.0)

        if resp.success:
            self.retry_counts[tag] = 0
            if tag.startswith('pick'):
                self.send_gripper_close(tag)
            elif tag.startswith('home_after_pick'):
                self.send_linear_move(0.5)
            elif tag.startswith('place'):
                self.send_gripper_open(tag)
            elif tag.startswith('home_after_place'):
                self.send_linear_move(0.0)
        else:
            self.retry_counts[tag] += 1
            if self.retry_counts[tag] < self.MAX_RETRIES:
                self.get_logger().warn(f'[{tag}] PlanExec failed, retry {self.retry_counts[tag]}/{self.MAX_RETRIES}')
                time.sleep(1.0)
                self.send_exec(tag)
            else:
                self.get_logger().error(f'[{tag}] PlanExec failed {self.MAX_RETRIES} times, shutting down')
                rclpy.shutdown()

    def send_gripper_close(self, tag: str):
        req = PlanJoint.Request()
        req.target = [0.85] * 6
        fut = self.grip_plan_cli.call_async(req)
        fut.add_done_callback(lambda f: self.on_gripper_plan_close(f, tag))

    def on_gripper_plan_close(self, future, tag: str):
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().error(f'[{tag}] Gripper plan exception: {e}')
            return

        self.get_logger().info(f'[{tag}] Gripper plan response: success={resp.success}')
        time.sleep(1.0)

        if resp.success:
            fut2 = self.grip_exec_cli.call_async(PlanExec.Request(wait=True))
            fut2.add_done_callback(lambda f: self.on_gripper_exec_close(f, tag))
        else:
            self.retry_counts[tag] = self.retry_counts.get(tag, 0) + 1
            if self.retry_counts[tag] < self.MAX_RETRIES:
                self.get_logger().warn(f'[{tag}] Gripper plan failed, retry {self.retry_counts[tag]}/{self.MAX_RETRIES}')
                time.sleep(1.0)
                self.send_gripper_close(tag)
            else:
                self.get_logger().error(f'[{tag}] Gripper plan failed {self.MAX_RETRIES} times, shutting down')
                rclpy.shutdown()

    def on_gripper_exec_close(self, future, tag: str):
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().error(f'[{tag}] Gripper exec exception: {e}')
            return

        self.get_logger().info(f'[{tag}] Gripper exec response: success={resp.success}')
        time.sleep(1.0)

        if resp.success:
            self.send_pose(self.home, f'home_after_pick{tag[-1]}')
        else:
            self.retry_counts[tag] = self.retry_counts.get(tag, 0) + 1
            if self.retry_counts[tag] < self.MAX_RETRIES:
                self.get_logger().warn(f'[{tag}] Gripper exec failed, retry {self.retry_counts[tag]}/{self.MAX_RETRIES}')
                time.sleep(1.0)
                self.send_gripper_close(tag)
            else:
                self.get_logger().error(f'[{tag}] Gripper exec failed {self.MAX_RETRIES} times, shutting down')
                rclpy.shutdown()

    def send_gripper_open(self, tag: str):
        req = PlanJoint.Request()
        req.target = [0.0] * 6
        fut = self.grip_plan_cli.call_async(req)
        fut.add_done_callback(lambda f: self.on_gripper_plan_open(f, tag))

    def on_gripper_plan_open(self, future, tag: str):
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().error(f'[{tag}] Gripper plan exception: {e}')
            return

        self.get_logger().info(f'[{tag}] Gripper plan response: success={resp.success}')
        time.sleep(1.0)

        if resp.success:
            fut2 = self.grip_exec_cli.call_async(PlanExec.Request(wait=True))
            fut2.add_done_callback(lambda f: self.on_gripper_exec_open(f, tag))
        else:
            self.retry_counts[tag] = self.retry_counts.get(tag, 0) + 1
            if self.retry_counts[tag] < self.MAX_RETRIES:
                self.get_logger().warn(f'[{tag}] Gripper plan failed, retry {self.retry_counts[tag]}/{self.MAX_RETRIES}')
                time.sleep(1.0)
                self.send_gripper_open(tag)
            else:
                self.get_logger().error(f'[{tag}] Gripper plan failed {self.MAX_RETRIES} times, shutting down')
                rclpy.shutdown()

    def on_gripper_exec_open(self, future, tag: str):
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().error(f'[{tag}] Gripper exec exception: {e}')
            return

        self.get_logger().info(f'[{tag}] Gripper exec response: success={resp.success}')
        time.sleep(1.0)

        if resp.success:
            idx = self.current_index
            self.send_pose(self.home, f'home_after_place{idx}')
        else:
            self.retry_counts[tag] = self.retry_counts.get(tag, 0) + 1
            if self.retry_counts[tag] < self.MAX_RETRIES:
                self.get_logger().warn(f'[{tag}] Gripper exec failed, retry {self.retry_counts[tag]}/{self.MAX_RETRIES}')
                time.sleep(1.0)
                self.send_gripper_open(tag)
            else:
                self.get_logger().error(f'[{tag}] Gripper exec failed {self.MAX_RETRIES} times, shutting down')
                rclpy.shutdown()

    def send_linear_move(self, position: float):
        self.linear_target = position
        req = MoveLinearMotor.Request()
        req.target_position_m = position
        self.get_logger().info(f'[linear] Sending linear move to {position:.3f} m...')
        fut = self.linear_cli.call_async(req)
        fut.add_done_callback(self.on_linear_response)

    def on_linear_response(self, future):
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().error(f'[linear] exception: {e}')
            time.sleep(1.0)
            return self.send_linear_move(self.linear_target)

        self.get_logger().info(f'[linear] response: success={resp.success}, message="{resp.message}"')
        time.sleep(2.0)

        tag = 'linear' + (str(self.current_index) if self.linear_done else '_first')
        if not self.linear_done:
            if resp.success:
                self.linear_done = True
                self.get_logger().info('Linear move complete, waiting for place marker')
            else:
                self.retry_counts[tag] = self.retry_counts.get(tag, 0) + 1
                if self.retry_counts[tag] < self.MAX_RETRIES:
                    self.get_logger().warn(f'[linear] Linear move failed, retry {self.retry_counts[tag]}/{self.MAX_RETRIES}')
                    time.sleep(1.0)
                    self.send_linear_move(self.linear_target)
                else:
                    self.get_logger().error(f'[linear] Linear move failed {self.MAX_RETRIES} times, shutting down')
                    rclpy.shutdown()
        else:
            if resp.success:
                self.current_index += 1
                if self.current_index < len(self.pairs):
                    self.get_logger().info(f'Advancing to next pair: {self.pairs[self.current_index]}')
                    self.linear_done = False
                    self.stage = 0
                else:
                    self.get_logger().info('All pairs processed—sequence complete')
                    self.stage = 3
            else:
                self.retry_counts[tag] = self.retry_counts.get(tag, 0) + 1
                if self.retry_counts[tag] < self.MAX_RETRIES:
                    self.get_logger().warn(f'[linear] Final move failed, retry {self.retry_counts[tag]}/{self.MAX_RETRIES}')
                    time.sleep(1.0)
                    self.send_linear_move(self.linear_target)
                else:
                    self.get_logger().error(f'[linear] Final move failed {self.MAX_RETRIES} times, shutting down')
                    rclpy.shutdown()

    # --------------------
    # State Machine
    # --------------------

    def state_machine_callback(self):
        if self.current_index >= len(self.pairs):
            return  # done

        pick_id, place_id = self.pairs[self.current_index]

        # Stage 0: wait to pick
        if self.stage == 0 and pick_id in self.marker_poses:
            p = self.marker_poses[pick_id]
            pick = Pose()
            pick.position.x = p.position.x
            pick.position.y = p.position.y
            pick.position.z = p.position.z 

            # only match marker yaw; fix roll=0, pitch=0
            qx, qy, qz, qw = yaw_only_quaternion(p.orientation,
                                                 fixed_roll=math.pi,
                                                 fixed_pitch=0.0)
            pick.orientation.x = qx
            pick.orientation.y = qy
            pick.orientation.z = qz
            pick.orientation.w = qw

            self.send_pose(pick, f'pick{pick_id}')
            self.stage = 1
            self.get_logger().info(f'Pick {pick_id} → {place_id} started')
            return

        # Stage 1: after linear_done, wait for place marker
        if self.stage == 1 and self.linear_done:
            if place_id in self.marker_poses:
                p = self.marker_poses[place_id]
                place = Pose()
                place.position.x = p.position.x
                place.position.y = p.position.y
                place.position.z = p.position.z + 0.02
                place.orientation.x = 1.0
                place.orientation.y = 0.0
                place.orientation.z = 0.0
                place.orientation.w = 0.0

                # only match marker yaw; fix roll=0, pitch=0
                # qx, qy, qz, qw = yaw_only_quaternion(p.orientation,
                #                                      fixed_roll=math.pi,
                #                                      fixed_pitch=0.0)
                # place.orientation.x = qx
                # place.orientation.y = qy
                # place.orientation.z = qz
                # place.orientation.w = qw

                self.send_pose(place, f'place{pick_id}')
                self.stage = 2
                self.get_logger().info(f'Place {pick_id} → {place_id} started')
            else:
                self.get_logger().info(f'Waiting for marker ID {place_id}...')
            return


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
