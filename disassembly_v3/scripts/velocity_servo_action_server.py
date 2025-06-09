#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
import numpy as np
from disassembly_v3.action import MoveToPose

# Simple PID controller class
class PIDController:
    def __init__(self, Kp, Ki, Kd, imax=None, deadband=0.0):
        self.Kp = np.array(Kp, float)
        self.Ki = np.array(Ki, float)
        self.Kd = np.array(Kd, float)
        self.imax = np.array(imax, float) if imax is not None else None
        self.deadband = deadband
        self.reset()

    def reset(self):
        self.err_int = np.zeros_like(self.Kp)
        self.last_err = np.zeros_like(self.Kp)

    def update(self, err, dt):
        # deadband
        err = np.where(np.abs(err) < self.deadband, 0.0, err)
        # integral
        if dt > 0 and np.any(self.Ki != 0.0):
            self.err_int += err * dt
            if self.imax is not None:
                self.err_int = np.clip(self.err_int, -self.imax, self.imax)
        # derivative
        derr = (err - self.last_err) / dt if dt > 0 else np.zeros_like(err)
        self.last_err = err.copy()
        # PID output
        return self.Kp * err + self.Ki * self.err_int + self.Kd * derr

class VelocityServoActionServer(Node):
    def __init__(self):
        super().__init__('velocity_servo_action_server')

        # Parameters for PID gains and frames
        self.declare_parameter('goal_frame', 'link_base')
        self.declare_parameter('tcp_frame',  'link_tcp')
        self.declare_parameter('update_rate', 50.0)

        # Linear PID gains
        Kp_lin   = self.declare_parameter('Kp_lin',   [3.0,3.0,3.0]).value
        Ki_lin   = self.declare_parameter('Ki_lin',   [0.3,0.3,0.3]).value
        Kd_lin   = self.declare_parameter('Kd_lin',   [0.6,0.6,0.6]).value
        imax_lin = self.declare_parameter('imax_lin', [0.1,0.1,0.1]).value
        dead_lin = self.declare_parameter('deadband_lin', 0.002).value
        self.pid_lin = PIDController(Kp_lin, Ki_lin, Kd_lin, imax_lin, dead_lin)

        # Angular PID gains
        Kp_ang   = self.declare_parameter('Kp_ang',   [3.0,3.0,3.0]).value
        Ki_ang   = self.declare_parameter('Ki_ang',   [0.3,0.3,0.3]).value
        Kd_ang   = self.declare_parameter('Kd_ang',   [0.6,0.6,0.6]).value
        imax_ang = self.declare_parameter('imax_ang', [0.1,0.1,0.1]).value
        dead_ang = self.declare_parameter('deadband_ang', 0.01).value
        self.pid_ang = PIDController(Kp_ang, Ki_ang, Kd_ang, imax_ang, dead_ang)

        # Effort monitoring
        self.effort_threshold = float(self.declare_parameter('effort_threshold',1.0).value)
        self.latest_eff = {'joint2':0.0, 'joint3':0.0, 'joint4':0.0}
        self.create_subscription(JointState, '/joint_states',
                                 self.joint_cb, 10)

        # Publisher for velocity commands
        self.cmd_pub = self.create_publisher(TwistStamped,
                                             '/servo_server/delta_twist_cmds', 10)

        # TF setup
        from tf2_ros import Buffer, TransformListener
        self.tf_buffer = Buffer()
        TransformListener(self.tf_buffer, self)

        # Action server
        self._action_server = ActionServer(
            self,
            MoveToPose,
            'MoveToPose',
            execute_callback=self.execute_cb
        )

        # Logged rate
        self.rate_hz = self.get_parameter('update_rate').value
        self.get_logger().info('✅ VelocityServoActionServer ready')

    def joint_cb(self, msg: JointState):
        # capture latest efforts
        for name, effort in zip(msg.name, msg.effort):
            if name in self.latest_eff:
                self.latest_eff[name] = effort

    def execute_cb(self, goal_handle):
        goal = goal_handle.request
        self.get_logger().info(f"▶️  Moving to pose via velocity control")

        # reset PID integrators
        self.pid_lin.reset()
        self.pid_ang.reset()

        # track time
        last_time = self.get_clock().now().nanoseconds * 1e-9

        # loop until done/aborted
        while rclpy.ok():
            now = self.get_clock().now().nanoseconds * 1e-9
            dt = now - last_time
            last_time = now
            if dt <= 0:
                continue

            # 1) lookup current TCP pose in goal_frame
            try:
                from tf2_ros import TimeoutError
                trans = self.tf_buffer.lookup_transform(
                    goal.target.header.frame_id,
                    goal.target.header.frame_id,
                    rclpy.time.Time()
                )
            except Exception as e:
                goal_handle.abort()
                break

            # TODO: compute position & orientation error (axis-angle)
            err_pos = np.zeros(3)   # replace with real error
            err_ang = np.zeros(3)

            # 2) PID outputs
            vel_lin = self.pid_lin.update(err_pos, dt)
            vel_ang = self.pid_ang.update(err_ang, dt)

            # 3) clamp to goal.max_linear_speed / max_angular_speed
            vel_lin = np.clip(vel_lin, -goal.max_linear_speed, goal.max_linear_speed)
            vel_ang = np.clip(vel_ang, -goal.max_angular_speed, goal.max_angular_speed)

            # 4) publish velocity command
            twist = TwistStamped()
            twist.header.stamp = self.get_clock().now().to_msg()
            twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z = vel_lin
            twist.twist.angular.x, twist.twist.angular.y, twist.twist.angular.z = vel_ang
            self.cmd_pub.publish(twist)

            # 5) publish feedback
            feedback = MoveToPose.Feedback()
            feedback.linear_error  = list(err_pos)
            feedback.angular_error = list(err_ang)
            goal_handle.publish_feedback(feedback)

            # 6) check success (error norms < tolerance)
            if np.linalg.norm(err_pos) < goal.tolerance \
               and np.linalg.norm(err_ang) < goal.tolerance:
                goal_handle.succeed()
                break

            # 7) check effort abort
            if (self.latest_eff['joint2'] > self.effort_threshold
                or self.latest_eff['joint3'] > self.effort_threshold
                or self.latest_eff['joint4'] > self.effort_threshold):
                goal_handle.abort()
                break

            # sleep to keep rate
            rclpy.sleep(1.0 / self.rate_hz)

        # ensure zero velocity on finish
        zero = TwistStamped()
        zero.header.stamp = self.get_clock().now().to_msg()
        self.cmd_pub.publish(zero)

        # return result
        result = MoveToPose.Result()
        result.success = (goal_handle.status == goal_handle.STATUS_SUCCEEDED)
        result.message = ''
        return result


def main():
    rclpy.init()
    node = VelocityServoActionServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
