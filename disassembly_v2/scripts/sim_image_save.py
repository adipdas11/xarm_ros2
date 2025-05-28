#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from threading import Thread
import os
from datetime import datetime

class ColorImageSaver(Node):
    def __init__(self):
        super().__init__('color_image_saver')
        self.bridge = CvBridge()
        self.latest_image = None

        self.subscription = self.create_subscription(
            Image,
            '/xarm5/D435_1/color/image_raw',
            self.image_callback,
            10
        )
        self.get_logger().info('Subscribed to /xarm5/D435_1/color/image_raw')

        # Start the input thread
        self.input_thread = Thread(target=self.wait_for_input)
        self.input_thread.daemon = True
        self.input_thread.start()

    def image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {e}')

    def wait_for_input(self):
        while rclpy.ok():
            input("Press Enter to save the latest image...\n")
            if self.latest_image is not None:
                timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
                filename = f'color_image_{timestamp}.png'
                cv2.imwrite('/home/adip/sim_image/'+filename, self.latest_image)
                self.get_logger().info(f"Saved image as {filename}")
            else:
                self.get_logger().warn("No image received yet.")

def main(args=None):
    rclpy.init(args=args)
    node = ColorImageSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
