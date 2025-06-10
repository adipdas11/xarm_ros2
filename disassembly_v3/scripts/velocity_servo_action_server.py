#!/usr/bin/env python3

"""
VelocityServoActionServer: 
Implements the MoveToPose action server: uses PID on Cartesian errors and 
publishes velocity commands, aborting if joint efforts exceed threshold.
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.time import Time as RclpyTime
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
import numpy as np
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException, TimeoutException

from disassembly_v3.action import MoveToPose


def quaternion_inverse(q: np.ndarray) -> np.ndarray:
    return np.array([-q[0], -q[1], -q[2], q[3]], dtype=float) / np.dot(q, q)


def quaternion_multiply(a: np.ndarray, b: np.ndarray) -> np.ndarray:
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

    def update(self, err: np.ndarray, dt: float) -> np.ndarray:
        # Deadband
        err = np.where(np.abs(err) < self.deadband, 0.0, err)
        # Integral
        if dt > 0 and np.any(self.Ki != 0.0):
            self._err_int += err * dt
            if self.imax is not None:
                self._err_int = np.clip(self._err_int, -self.imax, self.imax)
        # Derivative
        derr = (err - self._last_err) / dt if dt > 0 else np.zeros_like(err)
        self._last_err = err.copy()
        # PID sum
        return self.Kp * err + self.Ki * self._err_int + self.Kd * derr


class VelocityServoActionServer(Node):
    def __init__(self):
        super().__init__('velocity_servo_action_server')

        # Parameters
        self.declare_parameter('goal_frame', 'link_base')
        self.declare_parameter('tcp_frame', 'link_tcp')
        self.declare_parameter('update_rate', 50.0)
        self.declare_parameter('tolerance', 0.01)
        self.declare_parameter('max_linear_vel', 1.0)
        self.declare_parameter('max_angular_vel', 1.0)
        self.declare_parameter('slow_zone', 0.1)
        self.effort_threshold = float(self.declare_parameter('effort_threshold', 1.0).value)

        # Linear PID
        Kp_lin   = self.declare_parameter('Kp_lin',   [3.0, 3.0, 3.0]).value
        Ki_lin   = self.declare_parameter('Ki_lin',   [0.3, 0.3, 0.3]).value
        Kd_lin   = self.declare_parameter('Kd_lin',   [0.6, 0.6, 0.6]).value
        imax_lin = self.declare_parameter('imax_lin', [0.1, 0.1, 0.1]).value
        dead_lin = self.declare_parameter('deadband_lin', 0.002).value
        self.pid_lin = PIDController(Kp_lin, Ki_lin, Kd_lin, imax_lin, dead_lin)
        self._orig_Kp_lin = np.array(Kp_lin)
        self._orig_Ki_lin = np.array(Ki_lin)
        self._orig_Kd_lin = np.array(Kd_lin)

        # Angular PID
        Kp_ang   = self.declare_parameter('Kp_ang',   [3.0, 3.0, 3.0]).value
        Ki_ang   = self.declare_parameter('Ki_ang',   [0.3, 0.3, 0.3]).value
        Kd_ang   = self.declare_parameter('Kd_ang',   [0.6, 0.6, 0.6]).value
        imax_ang = self.declare_parameter('imax_ang', [0.1, 0.1, 0.1]).value
        dead_ang = self.declare_parameter('deadband_ang', 0.01).value
        self.pid_ang = PIDController(Kp_ang, Ki_ang, Kd_ang, imax_ang, dead_ang)

        # Effort monitor
        self.latest_eff = {'joint2': 0.0, 'joint3': 0.0, 'joint4': 0.0}
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)

        # Publisher & TF
        self.cmd_pub = self.create_publisher(TwistStamped, '/servo_server/delta_twist_cmds', 10)
        self.tf_buffer = Buffer()
        TransformListener(self.tf_buffer, self)

        # Action server
        self._action_server = ActionServer(self, MoveToPose, 'MoveToPose', execute_callback=self.execute_cb)

        self.rate_hz = self.get_parameter('update_rate').value
        self.get_logger().info('✅ VelocityServoActionServer ready')

    def joint_cb(self, msg: JointState):
        for n, e in zip(msg.name, msg.effort):
            if n in self.latest_eff:
                self.latest_eff[n] = e

    def execute_cb(self, goal_handle):
        goal = goal_handle.request
        self.get_logger().info('▶️  Moving to pose via velocity control')

        self.pid_lin.reset()
        self.pid_ang.reset()

        last = self.get_clock().now().nanoseconds * 1e-9
        tol_pos = self.get_parameter('tolerance').value
        tol_ang = tol_pos
        max_lin = self.get_parameter('max_linear_vel').value
        max_ang = self.get_parameter('max_angular_vel').value
        slow_zone = self.get_parameter('slow_zone').value
        succeeded = False

        while rclpy.ok():
            now = self.get_clock().now().nanoseconds * 1e-9
            dt = now - last; last = now
            if dt <= 0: 
                continue

            base = goal.target.header.frame_id
            tcp  = self.get_parameter('tcp_frame').value

            if not self.tf_buffer.can_transform(base, tcp, RclpyTime()):
                time.sleep(0.02)
                continue

            try:
                t = self.tf_buffer.lookup_transform(base, tcp, RclpyTime())
            except (LookupException, ConnectivityException, ExtrapolationException, TimeoutException) as e:
                self.get_logger().error(f"TF lookup failed: {e}")
                goal_handle.abort()
                break

            # compute errors
            cur_p = np.array([t.transform.translation.x,
                              t.transform.translation.y,
                              t.transform.translation.z])
            tgt_p = np.array([goal.target.pose.position.x,
                              goal.target.pose.position.y,
                              goal.target.pose.position.z])
            err_pos = tgt_p - cur_p

            cur_q = np.array([t.transform.rotation.x,
                              t.transform.rotation.y,
                              t.transform.rotation.z,
                              t.transform.rotation.w])
            tgt_q = np.array([goal.target.pose.orientation.x,
                              goal.target.pose.orientation.y,
                              goal.target.pose.orientation.z,
                              goal.target.pose.orientation.w])
            qe = quaternion_multiply(tgt_q, quaternion_inverse(cur_q))
            qe /= np.linalg.norm(qe)
            ang = 2.0 * np.arccos(np.clip(qe[3], -1.0, 1.0))
            err_ang = np.zeros(3) if abs(ang) < 1e-6 else (qe[:3]/np.sin(ang/2.0))*ang

            # self.get_logger().info(f"❓ errors → pos: {err_pos.tolist()}, ang: {err_ang.tolist()}")

            # taper gains
            r = np.linalg.norm(err_pos)
            if r < slow_zone:
                s = r/slow_zone
                self.pid_lin.Kp = self._orig_Kp_lin * s
                self.pid_lin.Ki = self._orig_Ki_lin * s
                self.pid_lin.Kd = self._orig_Kd_lin * s
            else:
                self.pid_lin.Kp = self._orig_Kp_lin.copy()
                self.pid_lin.Ki = self._orig_Ki_lin.copy()
                self.pid_lin.Kd = self._orig_Kd_lin.copy()

            # PID outputs & clamp
            vl = np.clip(self.pid_lin.update(err_pos, dt), -max_lin, max_lin)
            va = np.clip(self.pid_ang.update(err_ang, dt), -max_ang, max_ang)

            twist = TwistStamped()
            twist.header.stamp = self.get_clock().now().to_msg()
            twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z = vl
            twist.twist.angular.x, twist.twist.angular.y, twist.twist.angular.z = va
            self.cmd_pub.publish(twist)

            # feedback
            fb = MoveToPose.Feedback()
            fb.linear_error, fb.angular_error = err_pos.tolist(), err_ang.tolist()
            goal_handle.publish_feedback(fb)

            # success?
            if r < tol_pos and np.linalg.norm(err_ang) < tol_ang:
                succeeded = True
                goal_handle.succeed()
                break

            # effort abort?
            if any(abs(self.latest_eff[j]) > self.effort_threshold for j in ('joint2','joint3','joint4')):
                self.get_logger().warn("⚠️ Effort exceeded, aborting")
                goal_handle.abort()
                break

            time.sleep(1.0/self.rate_hz)

        # ensure stop
        z = TwistStamped()
        z.header.stamp = self.get_clock().now().to_msg()
        self.cmd_pub.publish(z)

        res = MoveToPose.Result()
        res.success, res.message = succeeded, ''
        return res


def main():
    rclpy.init()
    node = VelocityServoActionServer()

    # <- Use a MultiThreadedExecutor so TF & JointState callbacks keep running
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
