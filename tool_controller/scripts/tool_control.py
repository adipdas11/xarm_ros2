#!/usr/bin/env python3
"""
ROS2 node: tool_commander
Subscribes to 'tool_cmd' (std_msgs/Int8), sends the integer command (1, 0, -1) over serial to /dev/ttyACM0 (or configured port),
or—if the port isn’t available—fakes it by publishing an angle on 'tool_cmd_sim' that:
 - when cmd=1, sweeps from 0 → 2π, then wraps to 0, incrementing by a configurable step
 - when cmd=-1, sweeps from 0 → -2π, then wraps to 0, decrementing by that step
 - when cmd=0, holds at 0
"""
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, Float32
import serial


class ToolCommander(Node):
    def __init__(self):
        super().__init__('tool_commander')

        # parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('fake_step', 0.01) 
        port      = self.get_parameter('port').get_parameter_value().string_value
        baud      = self.get_parameter('baud').get_parameter_value().integer_value
        self.step = self.get_parameter('fake_step').get_parameter_value().double_value

        # try real port
        try:
            self.ser = serial.Serial(port, baud, timeout=1.0)
            self.fake = False
            self.get_logger().info(f"Opened serial port {port} @ {baud}bps")
        except (serial.SerialException, OSError):
            self.ser  = None
            self.fake = True
            self.get_logger().warn(f"Could not open {port}, entering FAKE mode")

        # in fake mode, prepare publisher and timer:
        if self.fake:
            self.sim_pub     = self.create_publisher(Float32, 'tool_cmd_sim', 10)
            self.fake_cmd    = 0
            self.fake_value  = 0.0
            self.create_timer(0.01, self._publish_fake)  # 100 Hz timer

        # subscribe to tool_cmd
        self.create_subscription(Int8, 'tool_cmd', self.handle_tool_cmd, 10)
        self.get_logger().info("Subscribed to 'tool_cmd' topic")

    def handle_tool_cmd(self, msg: Int8):
        cmd = msg.data
        if cmd not in (1, 0, -1):
            self.get_logger().warn(f"Received invalid command '{cmd}'")
            return

        if not self.fake:
            # real hardware
            to_send = f"{cmd}\n".encode('utf-8')
            try:
                self.ser.write(to_send)
                resp = self.ser.readline().decode('utf-8', errors='ignore').strip()
                self.get_logger().info(f"Sent {cmd}, received: '{resp}'")
            except serial.SerialException as e:
                self.get_logger().error(f"Serial error: {e}")
        else:
            # fake mode: set direction & reset starting value
            self.fake_cmd = cmd
            if cmd == 1:
                self.fake_value = 0.0
            elif cmd == -1:
                self.fake_value = 0.0
            else:  # cmd==0
                self.fake_value = 0.0
            self.get_logger().info(f"[FAKE] tool_cmd={cmd}, resetting value to {self.fake_value:.3f}")

    def _publish_fake(self):
        # produce and publish the simulated angle
        if self.fake_cmd == 0:
            val = 0.0
        elif self.fake_cmd == 1:
            val = self.fake_value
            # advance and wrap
            self.fake_value += self.step
            if self.fake_value > 2 * math.pi:
                self.fake_value = 0.0
        else:  # self.fake_cmd == -1
            val = self.fake_value
            self.fake_value -= self.step
            if self.fake_value < -2 * math.pi:
                self.fake_value = 0.0

        msg = Float32(data=val)
        self.sim_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ToolCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down tool_commander node")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
