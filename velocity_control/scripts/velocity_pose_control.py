#!/usr/bin/env python3

import os
import sys
import time
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import tf2_ros
from tf2_ros import TransformException

# add xArm wrapper to path
sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))
from xarm.wrapper import XArmAPI


def quaternion_matrix(quat):
    """
    Build a 4×4 rotation matrix from quaternion [x, y, z, w].
    """
    x, y, z, w = quat
    xx, yy, zz = x*x, y*y, z*z
    xy, xz, xw = x*y, x*z, x*w
    yz, yw, zw = y*z, y*w, z*w
    return np.array([
        [w*w + xx - yy - zz, 2*(xy - zw),      2*(xz + yw),      0.0],
        [2*(xy + zw),        w*w - xx + yy - zz, 2*(yz - xw),     0.0],
        [2*(xz - yw),        2*(yz + xw),       w*w - xx - yy + zz, 0.0],
        [0.0,                0.0,               0.0,               1.0],
    ], dtype=float)


class XArmVelocityController(Node):
    def __init__(self):
        super().__init__('xarm_velocity_controller')

        # hard-coded IP
        ip = '192.168.1.239'
        self.get_logger().info(f'🚀 Connecting to xArm at {ip}…')
        self.arm = XArmAPI(ip)
        self.arm.motion_enable(True)
        self.get_logger().info('✅ Motion enabled')

        # Cartesian-velocity mode
        self.arm.set_mode(5)
        self.arm.set_state(0)
        time.sleep(1)
        self.get_logger().info('🎯 Cartesian-velocity mode enabled')

        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer, self)

        # frames
        self.base_frame = 'link_base'
        self.ee_frame = 'link_tcp'

        # target pose
        self.ready_pose = Pose()
        self.ready_pose.position.x = 0.4
        self.ready_pose.position.y = 0.0
        self.ready_pose.position.z = 0.015
        self.ready_pose.orientation.x = 1.0
        self.ready_pose.orientation.y = 0.0
        self.ready_pose.orientation.z = 0.0
        self.ready_pose.orientation.w = 0.0

        # control parameters
        self.rate_hz = 20.0
        self.pos_thresh_m = 0.025            # 2 cm

        # distinct effort thresholds
        self.probe_effort_threshold   = -5.5   # for entering retract after probing
        self.retract_effort_threshold = -7.0   # for finishing retract

        self.probe_speed = 30.0             # mm/s probing

        # retract gain & caps
        self.retract_kp = 10.0              # mm/s per Amp above threshold
        self.min_retract_speed = 5.0        # mm/s
        self.max_retract_speed = 50.0       # mm/s

        # stuck detection params
        self.stuck_epsilon = 0.0005         # 0.5 mm
        self.stuck_count = 0
        self.stuck_count_goal = int(0.5*self.rate_hz)

        # retract stability check
        self.stable_duration = 5.0          # seconds to confirm stability
        self.stable_thresh   = 0.5          # A allowable change per sample

        # stages
        self.stage = 'goto'

        threading.Thread(target=self.control_loop, daemon=True).start()

    def control_loop(self):
        rate = self.create_rate(self.rate_hz)
        self.get_logger().info('🔄 Starting control loop…')
        while rclpy.ok() and self.stage != 'done':
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.base_frame, self.ee_frame, rclpy.time.Time())
            except TransformException as e:
                self.get_logger().warn(f'⚠️ TF lookup failed: {e}')
                rate.sleep()
                continue

            cx, cy, cz = (tf.transform.translation.x,
                          tf.transform.translation.y,
                          tf.transform.translation.z)

            if self.stage == 'goto':
                self._goto_pose(cx, cy, cz)
            elif self.stage == 'probe':
                self._probe_down(cz)
            elif self.stage == 'retract':
                self._retract_sequence()

            rate.sleep()

        self.get_logger().info('✅ All sequences complete.')

    def _goto_pose(self, cx, cy, cz):
        tx, ty, tz = (self.ready_pose.position.x,
                      self.ready_pose.position.y,
                      self.ready_pose.position.z)
        dx, dy, dz = tx-cx, ty-cy, tz-cz
        vx, vy, vz = [np.clip(1000.0*d, -100, 100) for d in (dx, dy, dz)]
        self.arm.vc_set_cartesian_velocity([vx, vy, vz, 0, 0, 0])
        self.get_logger().info(f'📍 Goto err(m): [{dx:.4f},{dy:.4f},{dz:.4f}]')
        if abs(dx)<self.pos_thresh_m and abs(dy)<self.pos_thresh_m and abs(dz)<self.pos_thresh_m:
            self.get_logger().info('🎯 Pose reached → probe.')
            self.stage = 'probe'
            self.last_z = cz
            self.stuck_count = 0

    def _probe_down(self, cz):
        self.arm.vc_set_cartesian_velocity([0,0,-self.probe_speed,0,0,0])
        self.get_logger().info(f'⬇️ Probing @ {self.probe_speed}mm/s, Z={cz:.4f}m')

        self.arm.set_report_tau_or_i(1)
        _, efforts = self.arm.get_joints_torque()
        j3 = efforts[2]
        cond = (j3 > self.probe_effort_threshold)
        self.get_logger().info(f'🔋 J3={j3:.2f}A, probe_thr={self.probe_effort_threshold}A, cond={cond}')

        if cond:
            self.arm.vc_set_cartesian_velocity([0]*6)
            self.get_logger().info('⚠️ Probe thr met → retract.')
            self.stage = 'retract'
            return

        if abs(cz-self.last_z)<self.stuck_epsilon:
            self.stuck_count+=1
        else:
            self.stuck_count=0
        self.last_z=cz

        if self.stuck_count>=self.stuck_count_goal:
            self.arm.vc_set_cartesian_velocity([0]*6)
            self.get_logger().info('⚠️ Stuck → retract.')
            self.stage='retract'

    def _retract_sequence(self):
        self.get_logger().info('🔼 Starting smooth retract...')
        prev_j3 = None
        stable_start = None

        while rclpy.ok():
            self.arm.set_report_tau_or_i(1)
            _, efforts = self.arm.get_joints_torque()
            j3 = efforts[2]
            self.get_logger().info(f'🔋 J3 retract={j3:.2f}A')

            # finished when back under retract threshold, after stability window
            if j3 <= self.retract_effort_threshold:
                if stable_start is None:
                    stable_start = time.time()
                elif time.time() - stable_start >= self.stable_duration:
                    break
            else:
                stable_start = None
                # speed ∝ (j3 - threshold)
                error = j3 - self.retract_effort_threshold
                speed = self.retract_kp * error
                speed = np.clip(speed, self.min_retract_speed, self.max_retract_speed)
                self.arm.vc_set_cartesian_velocity([0,0,speed,0,0,0])
                self.get_logger().info(f'🔼 Speed={speed:.1f}mm/s')

            time.sleep(1.0/self.rate_hz)

        self.arm.vc_set_cartesian_velocity([0]*6)
        self.get_logger().info('✅ Retract complete.')
        self.stage='done'


def main(args=None):
    rclpy.init(args=args)
    node = XArmVelocityController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.arm.vc_set_cartesian_velocity([0]*6)
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()
