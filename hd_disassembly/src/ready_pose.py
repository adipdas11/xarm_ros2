#!/usr/bin/env python3
import math
import time
import rclpy
from rclpy.node import Node
from xarm.wrapper import XArmAPI

def quat_to_rpy(qx, qy, qz, qw):
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (qw * qy - qz * qx)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw

class DualArmHomeThenStart(Node):
    def __init__(self):
        super().__init__('dual_arm_home_then_start')

        # Params (same IPs as your FSM)
        self.R_ip = '192.168.1.239'
        self.L_ip = '192.168.1.195'
        self.speed = 100.0   # mm/s
        self.acc   = 2000.0  # mm/s^2

        # RIGHT start pose
        self.R_start_xyz_m = (0.37, -0.38, 0.1)
        R_qx, R_qy, R_qz, R_qw = 1.0, 0.0, 0.0, 0.0
        self.R_start_rpy_deg = tuple(math.degrees(a) for a in quat_to_rpy(R_qx, R_qy, R_qz, R_qw))

        # LEFT start pose
        self.L_start_xyz_m = (0.4, 0.0, 0.2)
        L_qx, L_qy, L_qz, L_qw = 1.0, 0.0, 0.0, 0.0
        self.L_start_rpy_deg = tuple(math.degrees(a) for a in quat_to_rpy(L_qx, L_qy, L_qz, L_qw))

        # Connect both arms
        self.get_logger().info(f'🤖 Connecting RIGHT @ {self.R_ip}')
        try:
            self.R_arm = XArmAPI(port=self.R_ip, is_radian=False, protocol_type=3)
        except TypeError:
            self.R_arm = XArmAPI(port=self.R_ip, is_radian=False, protocol=3)

        self.get_logger().info(f'🤖 Connecting LEFT  @ {self.L_ip}')
        try:
            self.L_arm = XArmAPI(port=self.L_ip, is_radian=False, protocol_type=3)
        except TypeError:
            self.L_arm = XArmAPI(port=self.L_ip, is_radian=False, protocol=3)

        # Prep & go HOME
        self._prepare_arm(self.R_arm, 'R')
        self._prepare_arm(self.L_arm, 'L')

        self.get_logger().info('🏠 Sending BOTH arms to HOME…')
        # self._go_home(self.R_arm, 'R')
        # self._go_home(self.L_arm, 'L')

        # Move to start poses
        self._move_to_start(self.R_arm, 'R', self.R_start_xyz_m, self.R_start_rpy_deg)
        self._move_to_start(self.L_arm, 'L', self.L_start_xyz_m, self.L_start_rpy_deg)

        self.get_logger().info('✅ Both arms at start. Exiting.')
        rclpy.shutdown()

    def _prepare_arm(self, arm, tag):
        try:
            if getattr(arm, 'has_error', False):
                self.get_logger().warn(f'🧹 [{tag}] has_error → clean_error()'); arm.clean_error()
            if getattr(arm, 'has_warn', False):
                self.get_logger().warn(f'🧹 [{tag}] has_warn → clean_warn()'); arm.clean_warn()
        except Exception as e:
            self.get_logger().warn(f'[{tag}] While clearing errs/warns: {e}')
        for name, fn in [
            ('motion_enable', lambda: arm.motion_enable(True)),
            ('set_mode(0)',   lambda: arm.set_mode(0)),
            ('set_state(0)',  lambda: arm.set_state(0)),
        ]:
            try:
                code = fn(); code = code[0] if isinstance(code, tuple) else code
                if code != 0:
                    self.get_logger().warn(f'[{tag}] {name} returned code {code}')
            except Exception as e:
                self.get_logger().warn(f'[{tag}] {name} raised: {e}')
        time.sleep(0.1)

    def _go_home(self, arm, tag):
        code = arm.move_gohome(is_radian=False, wait=True)
        if code != 0:
            self.get_logger().warn(f'🏠 [{tag}] HOME returned code {code} (err={arm.error_code}, warn={arm.warn_code})')
        else:
            self.get_logger().info(f'✅ [{tag}] at HOME')

    def _move_to_start(self, arm, tag, xyz_m, rpy_deg):
        x, y, z = (v * 1000.0 for v in xyz_m)  # to mm
        roll, pitch, yaw = rpy_deg
        self.get_logger().info(
            f'➡️ {tag} ➜ start XYZ(mm)=({x:.1f},{y:.1f},{z:.1f}) RPY(deg)=({roll:.2f},{pitch:.2f},{yaw:.2f})'
        )
        code = arm.set_position(
            x=x, y=y, z=z,
            roll=roll, pitch=pitch, yaw=yaw,
            speed=self.speed, mvacc=self.acc,
            is_radian=False, wait=True
        )
        if code != 0:
            self.get_logger().error(f'❌ [{tag}] set_position failed (code={code}, err={arm.error_code}, warn={arm.warn_code})')
        else:
            self.get_logger().info(f'✅ [{tag}] start pose reached.')

def main(args=None):
    rclpy.init(args=args)
    DualArmHomeThenStart()

if __name__ == '__main__':
    main()
