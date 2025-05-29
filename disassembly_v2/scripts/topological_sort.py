#!/usr/bin/env python3
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# match your executor’s priority ordering:
PART_PRIORITY = {
    'screw':   0,
    'lid':     1,
    'pcb':     2,
    'default': 3,
}


class SequenceGenerator(Node):
    def __init__(self):
        super().__init__('disassembly_sequence_node')

        # listen to the vision‐built graph
        self.create_subscription(
            String,
            '/yolov11/part_graph',
            self.on_graph,
            10
        )

        # publish the sorted sequence
        self.seq_pub = self.create_publisher(String, '/disassembly_sequence', 10)

        self.get_logger().info('✅ Disassembly Sequence Generator ready.')

    def on_graph(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn('Received bad JSON on /yolov11/part_graph')
            return

        parts = []
        for node in data.get('nodes', []):
            label = node.get('label', '')
            # include both screws and lids (and any other types in PART_PRIORITY)
            kind = label.split('_', 1)[0]
            if kind not in PART_PRIORITY:
                continue
            prio = PART_PRIORITY.get(kind, PART_PRIORITY['default'])
            parts.append((prio, label))

        # sort by priority then label
        parts.sort(key=lambda x: (x[0], x[1]))
        sequence = [lbl for _, lbl in parts]

        # publish as JSON list
        out = String()
        out.data = json.dumps(sequence)
        self.seq_pub.publish(out)

        self.get_logger().info(f"🔄 Published /disassembly_sequence: {sequence}")


def main(args=None):
    rclpy.init(args=args)
    node = SequenceGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
