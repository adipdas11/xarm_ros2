#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection3DArray
import ollama
import json

class DSPGeneratorLLMNode(Node):
    def __init__(self):
        super().__init__('dsp_generator_llm_node')
        self.subscription = self.create_subscription(
            Detection3DArray,
            '/yolov11/part_detections',
            self.callback,
            10
        )
        self.processed = False
        self.get_logger().info("🧠 DSP Generator LLM Node Initialized. Waiting for parts...")

    def callback(self, msg):
        if self.processed:
            return  # Only process once

        part_labels = []
        for detection in msg.detections:
            if detection.results:
                label = detection.results[0].hypothesis.class_id
                part_labels.append(label)

        if not part_labels:
            self.get_logger().warn("⚠️ No part labels found in detection message.")
            return

        prompt = f"""
You are a disassembly assistant robot.

Given this list of parts:
{json.dumps(part_labels, indent=2)}

Return a step-by-step disassembly order. Screws must be removed before any other parts. Just output the list. Do not explain or add code.
"""

        try:
            response = ollama.chat(
                model='llama3.1',
                messages=[{'role': 'user', 'content': prompt}]
            )
            output = response['message']['content']
            self.get_logger().info("🛠 Disassembly Sequence (LLM):")
            for line in output.strip().split('\n'):
                self.get_logger().info(line.strip())
            self.processed = True
        except Exception as e:
            self.get_logger().error(f"❌ LLM Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DSPGeneratorLLMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
