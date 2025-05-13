#!/usr/bin/env python3
"""
ROS2 node: tool_commander
Subscribes to 'tool_cmd' (std_msgs/Int8), sends the integer command (1, 0, -1) over serial to /dev/ttyACM0 (or configured port),
reads back 'true'/'false', and logs the result.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
import serial

class ToolCommander(Node):
    def __init__(self):
        super().__init__('tool_commander')
        # Declare parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value

        # Open serial port
        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            self.get_logger().info(f"Opened serial port {port} @ {baud}bps")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port {port}: {e}")
            rclpy.shutdown()
            return

        # Subscription to tool_cmd topic
        self.sub = self.create_subscription(
            Int8,
            'tool_cmd',
            self.handle_tool_cmd,
            10)
        self.sub  # prevent unused variable warning
        self.get_logger().info("Subscribed to 'tool_cmd' topic")

    def handle_tool_cmd(self, msg: Int8):
        cmd = str(msg.data)
        # Validate
        if cmd not in ('1', '0', '-1'):
            self.get_logger().warn(f"Received invalid command '{cmd}'")
            return
        # Send over serial
        try:
            self.ser.write((cmd + "\n").encode('utf-8'))
        except serial.SerialException as e:
            self.get_logger().error(f"Serial write error: {e}")
            return

        # Read response line
        try:
            response = self.ser.readline().decode('utf-8').strip()
            self.get_logger().info(f"Sent {cmd}, received: '{response}'")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial read error: {e}")


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
