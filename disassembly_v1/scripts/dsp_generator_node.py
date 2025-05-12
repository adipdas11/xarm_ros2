#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection3DArray
import networkx as nx

class DSPGeneratorNode(Node):
    def __init__(self):
        super().__init__('dsp_generator_node')
        self.subscription = self.create_subscription(
            Detection3DArray,
            '/yolov11/part_detections',
            self.callback,
            10
        )
        self.get_logger().info("🔧 DSP Generator Node Ready.")

    def callback(self, msg):
        part_z_map = {}
        for detection in msg.detections:
            if len(detection.results) == 0:
                continue

            label = detection.results[0].hypothesis.class_id  
            z = detection.results[0].pose.pose.position.z
            part_z_map[label] = z

        if not part_z_map:
            self.get_logger().warn("No valid detections with pose.")
            return

        # Sort by z-height (descending: top parts first)
        sorted_parts = sorted(part_z_map.items(), key=lambda item: item[1], reverse=True)

        # Build dependency graph: top parts must be removed before those below
        G = nx.DiGraph()
        for i in range(len(sorted_parts)):
            for j in range(i + 1, len(sorted_parts)):
                G.add_edge(sorted_parts[i][0], sorted_parts[j][0])

        try:
            sequence = list(nx.topological_sort(G))
            self.get_logger().info("🛠 Disassembly Sequence:")
            for step in sequence:
                self.get_logger().info(f"   ➤ Remove {step}")
        except nx.NetworkXUnfeasible:
            self.get_logger().error("❌ Cyclic dependency in DSP graph!")

def main(args=None):
    rclpy.init(args=args)
    node = DSPGeneratorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
