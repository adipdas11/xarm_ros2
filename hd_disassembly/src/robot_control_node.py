#!/usr/bin/env python3
"""
robot_control_node.py

ROS2 node that:
  - Subscribes to /tracked_parts for real camera-frame poses
  - Calls /get_sequence to fetch parts in priority order
  - Calls /transform_point to get each part’s pose in the robot base frame
  - Uses XArmAPI to:
      • Move to home pose
      • Approach part (hover stop_height above)
      • Probe down using Cartesian velocity until joint_3 torque crosses threshold
      • Retract 5 mm for clearance
      • Unscrew using external tool on /tool_cmd, retracting 1 mm each time torque ≥ threshold,
        and stop when torque stays within ±0.1 Nm for 5 s
      • Return to home pose, wait 30 s, then proceed
  - Always ends back at home pose
  - All speeds, thresholds, and poses are loaded from disassembler_params.yaml
  - Logs each action step with emojis
  - Converts home‐pose quaternion→RPY via internal function
"""

import os
import math
import time
import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
from ament_index_python.packages import get_package_share_directory

from hd_disassembly.srv import GetSequence, TransformPoint
from hd_disassembly.msg import TrackedPart
from geometry_msgs.msg import PoseStamped
from xarm.wrapper import XArmAPI


def quaternion_to_euler(x, y, z, w):
    sinr = 2.0 * (w*x + y*z)
    cosr = 1.0 - 2.0 * (x*x + y*y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (w*y - z*x)
    pitch = math.copysign(math.pi/2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    siny = 2.0 * (w*z + x*y)
    cosy = 1.0 - 2.0 * (y*y + z*z)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')
        self.get_logger().info("🤖 RobotControlNode starting up…")

        # Load config
        pkg = get_package_share_directory('hd_disassembly')
        with open(os.path.join(pkg, 'config', 'disassembler_params.yaml')) as f:
            rc = yaml.safe_load(f)['robot']

        # Declare & fetch parameters
        self.declare_parameter('robot.ip',             rc.get('ip', '192.168.1.239'))
        self.declare_parameter('robot.move_speed_m_s', rc.get('move_speed_m_s', 0.1))
        self.declare_parameter('robot.stop_height',    rc['stop_height'])
        self.declare_parameter('robot.probing_speed',  rc['probing_speed'])
        self.declare_parameter('robot.unscrew_speed',  rc['unscrew_speed'])
        thr = rc['effort_threshold']
        self.declare_parameter('robot.probe_thresh',   thr['probing'])
        self.declare_parameter('robot.unscrew_thresh', thr['unscrew'])

        ip                   = self.get_parameter('robot.ip').value
        self.move_speed      = self.get_parameter('robot.move_speed_m_s').value * 1000.0
        self.probe_speed     = self.get_parameter('robot.probing_speed').value  * 1000.0
        self.unscrew_speed   = self.get_parameter('robot.unscrew_speed').value * 1000.0
        self.stop_height     = self.get_parameter('robot.stop_height').value   * 1000.0
        self.probe_thresh    = self.get_parameter('robot.probe_thresh').value
        self.unscrew_thresh  = self.get_parameter('robot.unscrew_thresh').value

        # Home pose + orientation
        hp = rc['home_pose']
        self.home_pose = PoseStamped()
        self.home_pose.header.frame_id = rc.get('home_frame', 'link_base')
        self.home_pose.pose.position.x = hp['position']['x']
        self.home_pose.pose.position.y = hp['position']['y']
        self.home_pose.pose.position.z = hp['position']['z']
        q = hp['orientation']
        self.home_rpy = quaternion_to_euler(q['x'], q['y'], q['z'], q['w'])

        # Log setup
        self.get_logger().info(f"🏠 Home XYZ: {self.home_pose.pose.position}")
        self.get_logger().info(f"🏠 Home RPY: {self.home_rpy}")
        self.get_logger().info(f"⏫ Stop height: {self.stop_height:.1f} mm")
        self.get_logger().info(
            f"🚀 Speeds (move/probe/unscrew): "
            f"{self.move_speed:.1f}/{self.probe_speed:.1f}/{self.unscrew_speed:.1f}"
        )

        # Initialize XArm API
        self.arm = XArmAPI(port=ip)
        self.arm.connect()
        self.arm.motion_enable(True)
        time.sleep(0.1)
        self.arm.set_mode(0);   time.sleep(0.1)
        self.arm.set_state(0);  time.sleep(0.1)
        self.arm.set_report_tau_or_i(1)
        self.get_logger().info(f"✅ Connected to XArm @ {ip}")

        # Publisher for tool_cmd
        self.tool_pub = self.create_publisher(Int8, '/tool_cmd', 10)

        # Setup service clients
        self.seq_cli = self.create_client(GetSequence, 'get_sequence')
        self.tf_cli  = self.create_client(TransformPoint, 'transform_point')
        for name, cli in [('get_sequence', self.seq_cli), ('transform_point', self.tf_cli)]:
            while not cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn(f"⏳ Waiting for {name} service…")
            self.get_logger().info(f"✅ {name} ready")

        # Subscribe tracked parts
        self.part_pose_buffer = {}
        self.create_subscription(TrackedPart, 'tracked_parts', self._pose_cb, 10)

        # Execute
        self.execute_sequence()

    def _pose_cb(self, msg: TrackedPart):
        self.part_pose_buffer[msg.track_id] = msg.position

    def execute_sequence(self):
        # 1) Move HOME
        self.get_logger().info("🏠 Moving to HOME…")
        self._go_home()
        self.get_logger().info("✅ At HOME")

        # 2) Start sequence
        req = GetSequence.Request(); req.request_type = 'start'
        fut = self.seq_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        res = fut.result()
        if not res:
            self.get_logger().error("❌ get_sequence 'start' failed")
            return

        # 3) Loop through parts
        while not res.finished:
            pid, label = res.part_id, res.part_label
            self.get_logger().info(f"🔀 Part #{pid} ({label})")

            # wait for pose...
            deadline = time.time() + 5.0
            while pid not in self.part_pose_buffer and time.time() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)
            if pid not in self.part_pose_buffer:
                self.get_logger().error(f"❌ No pose for part {pid}")
                return
            cam_pt = self.part_pose_buffer[pid]

            # 4) Transform to base frame
            tp_req = TransformPoint.Request(); tp_req.point_in = cam_pt
            fut_tf = self.tf_cli.call_async(tp_req)
            rclpy.spin_until_future_complete(self, fut_tf)
            tf_res = fut.result() if False else fut_tf.result()
            if not tf_res:
                self.get_logger().error("❌ transform_point failed")
                return
            bp = tf_res.point_out.point
            self.get_logger().info("📍 Transformed to base frame")

            # 5) Hover above part
            x_mm = bp.x * 1000.0
            y_mm = bp.y * 1000.0
            z_mm = bp.z * 1000.0 + self.stop_height
            r, p, yaw = self.home_rpy
            self.arm.set_position(
                x_mm, y_mm, z_mm, r, p, yaw,
                speed=self.move_speed, is_radian=True, wait=True
            )
            self.get_logger().info("✅ Hovered above part")

            # 6) Probe until contact
            self.get_logger().info(f"🔬 Probing until τ₃ ≥ {self.probe_thresh:.2f} N…")
            self.arm.set_mode(5); time.sleep(0.1)
            self.arm.set_state(0); time.sleep(0.1)
            self.arm.vc_set_cartesian_velocity([0,0,-self.probe_speed,0,0,0])
            while True:
                _, torq = self.arm.get_joints_torque()
                tau = torq[2]
                if tau >= self.probe_thresh:
                    self.get_logger().info(f"📏 Contact detected τ₃={tau:.2f} N")
                    break
            self.arm.vc_set_cartesian_velocity([0,0,0,0,0,0])
            self.arm.set_mode(0); time.sleep(0.1)
            self.arm.set_state(0); time.sleep(0.1)

            # 7) Retract 5 mm for clearance
            self.get_logger().info("↕️ Retracting 5 mm for clearance…")
            self.arm.set_tool_position(
                x=0, y=0, z=-2.0,
                speed=self.probe_speed, wait=True
            )
            self.get_logger().info("⏫ Retracted 5 mm; ready to unscrew")

            # 8) Unscrew: retract 1 mm whenever τ₃ ≥ threshold, stop when torque stable
            noise_thresh = 0.5   # Nm
            stable_time  = 1.5   # seconds
            _, torq0 = self.arm.get_joints_torque()
            last_effort = torq0[2]
            last_change = time.time()

            self.get_logger().info("🔩 Tool ON, starting unscrew…")
            cmd = Int8(data=-1)
            self.tool_pub.publish(cmd)

            while True:
                _, torq = self.arm.get_joints_torque()
                effort = torq[2]

                # if torque crosses threshold, retract 1 mm and reset timer
                if effort >= self.unscrew_thresh:
                    self.get_logger().info(f"⬆️ τ₃={effort:.2f} ≥ {self.unscrew_thresh:.2f}, retracting 1 mm")
                    self.arm.set_tool_position(x=0, y=0, z=-1.0,
                                               speed=self.probe_speed, wait=True)
                    last_change = time.time()
                    last_effort = effort

                # if torque change > noise_thresh, reset timer
                elif abs(effort - last_effort) > noise_thresh:
                    last_change = time.time()
                    last_effort = effort

                elapsed = time.time() - last_change
                self.get_logger().info(f"⚙️ τ₃={effort:.2f} N, stable={elapsed:.1f}s")

                # stop when torque has not changed > noise_thresh for stable_time
                if elapsed >= stable_time:
                    self.get_logger().info(f"🆙 Torque stable for {stable_time}s, stopping")
                    cmd.data = 0
                    self.tool_pub.publish(cmd)
                    break

                time.sleep(0.05)

            # revert mode
            self.arm.set_mode(0); time.sleep(0.1)
            self.arm.set_state(0); time.sleep(0.1)
            self.get_logger().info("✅ Unscrewing complete")

            # 9) Return HOME & wait 30 s
            self._go_home()
            self.get_logger().info("✅ At HOME; waiting 5 s")
            time.sleep(5)

            # 10) Next part
            req.request_type = 'next'
            fut = self.seq_cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut)
            res = fut.result()
            if not res:
                return

        # 11) Finished → HOME
        self.get_logger().info("🎉 All parts removed. Final HOME")
        self._go_home()

    def _go_home(self):
        hp = self.home_pose.pose.position
        r, p, yaw = self.home_rpy
        self.arm.motion_enable(True)
        self.arm.set_mode(0); time.sleep(0.1)
        self.arm.set_state(0); time.sleep(0.1)
        self.arm.set_position(
            hp.x*1000, hp.y*1000, hp.z*1000,
            r, p, yaw,
            speed=self.move_speed,
            is_radian=True,
            wait=True
        )


def main(args=None):
    rclpy.init(args=args)
    node = RobotControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
