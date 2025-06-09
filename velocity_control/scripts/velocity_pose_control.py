#!/usr/bin/env python3

import rclpy
import numpy as np
import csv
import os
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import JointState
import tf2_ros


def quaternion_inverse(q):
    return np.array([-q[0], -q[1], -q[2], q[3]], dtype=float) / np.dot(q, q)


def quaternion_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by + ay*bw + az*bx - ax*bz,
        aw*bz + az*bw + ax*by - ay*bx,
        aw*bw - ax*bx - ay*by - az*bz,
    ], dtype=float)


class PIDController:
    def __init__(self, Kp, Ki, Kd, imax=None, deadband=0.0):
        self.Kp = np.array(Kp, dtype=float)
        self.Ki = np.array(Ki, dtype=float)
        self.Kd = np.array(Kd, dtype=float)
        self.imax = np.array(imax, dtype=float) if imax is not None else None
        self.deadband = float(deadband)
        self.reset()

    def reset(self):
        self._err_int = np.zeros_like(self.Kp)
        self._last_err = np.zeros_like(self.Kp)

    def update(self, error, dt):
        err = error.copy()
        for i in range(len(err)):
            if abs(err[i]) < self.deadband:
                err[i] = 0.0

        if dt > 0 and np.any(self.Ki != 0.0):
            self._err_int += err * dt
            if self.imax is not None:
                if self.imax.ndim == 0:
                    max_i = abs(self.imax)
                    self._err_int = np.clip(self._err_int, -max_i, max_i)
                else:
                    # elementwise clamp
                    self._err_int = np.clip(self._err_int, -self.imax, self.imax)

        derr = (err - self._last_err) / dt if dt > 0 else np.zeros_like(err)
        self._last_err = err.copy()

        p_term = self.Kp * err
        i_term = self.Ki * self._err_int
        d_term = self.Kd * derr

        return p_term + i_term + d_term


class CartesianVelocityServo(Node):
    def __init__(self):
        super().__init__('cartesian_velocity_servo')

        # 1) Frames, rate, tolerance
        self.declare_parameter('goal_frame', 'link_base')
        self.declare_parameter('tcp_frame', 'link_tcp')
        self.declare_parameter('update_rate', 50.0)
        self.declare_parameter('tolerance', 0.01)

        # 2) PID gains for linear motion
        Kp_lin = self.declare_parameter('Kp_lin', [3.0, 3.0, 3.0]).value
        Ki_lin = self.declare_parameter('Ki_lin', [0.3, 0.3, 0.3]).value
        Kd_lin = self.declare_parameter('Kd_lin', [0.6, 0.6, 0.6]).value
        imax_lin = self.declare_parameter('imax_lin', [0.1, 0.1, 0.1]).value
        dead_lin = self.declare_parameter('deadband_lin', 0.002).value
        self.pid_lin = PIDController(Kp_lin, Ki_lin, Kd_lin, imax=imax_lin, deadband=dead_lin)

        # 3) PID gains for angular motion
        Kp_ang = self.declare_parameter('Kp_ang', [3.0, 3.0, 3.0]).value
        Ki_ang = self.declare_parameter('Ki_ang', [0.3, 0.3, 0.3]).value
        Kd_ang = self.declare_parameter('Kd_ang', [0.6, 0.6, 0.6]).value
        imax_ang = self.declare_parameter('imax_ang', [0.1, 0.1, 0.1]).value
        dead_ang = self.declare_parameter('deadband_ang', 0.01).value
        self.pid_ang = PIDController(Kp_ang, Ki_ang, Kd_ang, imax=imax_ang, deadband=dead_ang)

        # 4) Maximum velocity caps (linear [m/s], angular [rad/s])
        self.declare_parameter('max_linear_vel', 1.0)
        self.declare_parameter('max_angular_vel', 1.0)
        self.max_lin = float(self.get_parameter('max_linear_vel').value)
        self.max_ang = float(self.get_parameter('max_angular_vel').value)

        # 5) Effort‐threshold (if joint2, 3, or 4 > this, stop immediately)
        self.declare_parameter('effort_threshold', 1.0)
        self.effort_threshold = float(self.get_parameter('effort_threshold').value)

        # 6) Fixed goal pose
        self.goal = PoseStamped()
        self.goal.header.frame_id = self.get_parameter('goal_frame').value
        self.goal.pose.position.x = 0.4
        self.goal.pose.position.y = 0.0
        self.goal.pose.position.z = 0.15
        self.goal_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        self.tolerance = self.get_parameter('tolerance').value

        # 7) TF listener
        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer, self)

        # 8) Publisher for Cartesian velocity
        self.pub = self.create_publisher(TwistStamped, '/servo_server/delta_twist_cmds', 10)

        # 9) Subscription for JointState to collect efforts + instant threshold check
        self.latest_eff = {'joint2': 0.0, 'joint3': 0.0, 'joint4': 0.0}
        self.effort_data = {
            'time': [],
            'joint1': [],
            'joint2': [],
            'joint3': [],
            'joint4': [],
            'joint5': []
        }
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )

        # 10) Timer for control loop
        rate = self.get_parameter('update_rate').value
        self._last_time = self.get_clock().now().nanoseconds * 1e-9
        self.create_timer(1.0 / rate, self.control_loop)

        self.get_logger().info('✅ CartesianVelocityServo ready')

    def joint_states_callback(self, msg: JointState):
        """
        Whenever a JointState arrives:
         - extract efforts for joint1..joint5,
         - append them to self.effort_data (for CSV later),
         - update self.latest_eff for joint2,3,4,
         - if joint2/3/4 > threshold, stop motion immediately.
        """
        now = self.get_clock().now().nanoseconds * 1e-9

        # Look for indices of joint names in this message
        effs = {'joint1': None, 'joint2': None, 'joint3': None, 'joint4': None, 'joint5': None}
        for i, name in enumerate(msg.name):
            if name in effs:
                effs[name] = msg.effort[i]

        # Only log when all five are present
        if all(effs[j] is not None for j in effs):
            self.effort_data['time'].append(now)
            self.effort_data['joint1'].append(effs['joint1'])
            self.effort_data['joint2'].append(effs['joint2'])
            self.effort_data['joint3'].append(effs['joint3'])
            self.effort_data['joint4'].append(effs['joint4'])
            self.effort_data['joint5'].append(effs['joint5'])

            # Update latest efforts for 2,3,4
            self.latest_eff['joint2'] = effs['joint2']
            self.latest_eff['joint3'] = effs['joint3']
            self.latest_eff['joint4'] = effs['joint4']

            # Instant threshold check:
            if (
                    effs['joint3'] > self.effort_threshold
                    or effs['joint4'] > self.effort_threshold):
                self.get_logger().warn(
                    f"⚠️ Effort threshold exceeded: "
                    f" joint2={effs['joint2']:.2f}, "
                    f" joint3={effs['joint3']:.2f}, "
                    f" joint4={effs['joint4']:.2f}. "
                    f"Stopping motion instantly."
                )
                # Publish zero velocity, write CSV, then shutdown immediately
                self._publish_zero()
                self._write_efforts_to_csv()
                rclpy.shutdown()
                return

    def get_current_transform(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.goal.header.frame_id,
                self.get_parameter('tcp_frame').value,
                rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return None, None

        pos = np.array([
            t.transform.translation.x,
            t.transform.translation.y,
            t.transform.translation.z,
        ], dtype=float)

        rot = np.array([
            t.transform.rotation.x,
            t.transform.rotation.y,
            t.transform.rotation.z,
            t.transform.rotation.w,
        ], dtype=float)

        return pos, rot

    def control_loop(self):
        # 1) Compute dt
        now = self.get_clock().now().nanoseconds * 1e-9
        dt = now - self._last_time
        if dt <= 0:
            return
        self._last_time = now

        # 2) Get current TCP pose
        cur_pos, cur_quat = self.get_current_transform()
        if cur_pos is None:
            return

        # 3) Position error
        tgt_pos = np.array([
            self.goal.pose.position.x,
            self.goal.pose.position.y,
            self.goal.pose.position.z,
        ], dtype=float)
        err_pos = tgt_pos - cur_pos

        # 4) Orientation error (axis–angle)
        q_err = quaternion_multiply(self.goal_quat, quaternion_inverse(cur_quat))
        q_err /= np.linalg.norm(q_err)  # normalize
        angle = 2.0 * np.arccos(np.clip(q_err[3], -1.0, 1.0))
        if abs(angle) < 1e-6:
            axis = np.zeros(3)
        else:
            axis = q_err[:3] / np.sin(angle / 2.0)
        err_ang = axis * angle

        # 5) Log the errors
        self.get_logger().info(
            f"Position error = [x: {err_pos[0]:.4f}, y: {err_pos[1]:.4f}, z: {err_pos[2]:.4f}]"
        )
        self.get_logger().info(
            f"Orientation error = [rx: {err_ang[0]:.4f}, ry: {err_ang[1]:.4f}, rz: {err_ang[2]:.4f}]"
        )

        # 6) Check if goal reached
        if np.linalg.norm(err_pos) < self.tolerance and np.linalg.norm(err_ang) < self.tolerance:
            self.get_logger().info('✅ Goal reached')
            self._publish_zero()
            self._write_efforts_to_csv()
            rclpy.shutdown()
            return

        # 7) PID outputs
        vel_lin = self.pid_lin.update(err_pos, dt)   # shape (3,)
        vel_ang = self.pid_ang.update(err_ang, dt)   # shape (3,)

        # 8) Cap each axis
        vel_lin = np.clip(vel_lin, -self.max_lin, self.max_lin)
        vel_ang = np.clip(vel_ang, -self.max_ang, self.max_ang)

        # 9) Publish the command
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.twist.linear.x  = float(vel_lin[0])
        twist.twist.linear.y  = float(vel_lin[1])
        twist.twist.linear.z  = float(vel_lin[2])
        twist.twist.angular.x = float(vel_ang[0])
        twist.twist.angular.y = float(vel_ang[1])
        twist.twist.angular.z = float(vel_ang[2])
        self.pub.publish(twist)

    def _publish_zero(self):
        zero = TwistStamped()
        zero.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(zero)

    def _write_efforts_to_csv(self):
        """
        Write collected effort data into a CSV file.
        """
        # Adjust this path to wherever you like
        csv_path = '/home/adip/workspaces/dev_ws/src/xarm_ros2/velocity_control/EffortCSV/efforts.csv'
        os.makedirs(os.path.dirname(csv_path), exist_ok=True)

        with open(csv_path, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            # Header
            writer.writerow(['time', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5'])
            # Rows
            for i in range(len(self.effort_data['time'])):
                row = [
                    self.effort_data['time'][i],
                    self.effort_data['joint1'][i],
                    self.effort_data['joint2'][i],
                    self.effort_data['joint3'][i],
                    self.effort_data['joint4'][i],
                    self.effort_data['joint5'][i],
                ]
                writer.writerow(row)
        self.get_logger().info(f"📈 Effort data written to {csv_path}")


def main():
    rclpy.init()
    node = CartesianVelocityServo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
