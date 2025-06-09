#!/usr/bin/env python3
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from disassembly_v3.srv import GetSequence

# priority map for part classes
PART_PRIORITY = {
    'screw':   0,
    'lid':     1,
    'pcb':     2,
    'default': 3,
}

class SequencePlannerNode(Node):
    def __init__(self):
        super().__init__('sequence_planner_node')
        # 1) Subscribe to the part graph topic
        self.create_subscription(
            String,
            '/part_graph',
            self.graph_callback,
            10
        )

        # 2) Publisher for the screw sequence
        self.seq_pub = self.create_publisher(String, '/disassembly_sequence', 10)

        # 3) Cache last sequence
        self._sequence = []

        # 4) Service: GetSequence
        self.create_service(
            GetSequence,
            'GetSequence',
            self.handle_get_sequence
        )

        self.get_logger().info('✅ SequencePlannerNode ready')

    def graph_callback(self, msg: String):
        data = json.loads(msg.data)
        nodes = data.get('nodes', [])

        # 1) Filter for screws not yet removed
        screws = []
        for nd in nodes:
            label = nd['label']
            if not label.startswith('screw_'):
                continue
            parts = label.split('_')
            prio = PART_PRIORITY.get(parts[0], PART_PRIORITY['default'])
            screws.append((prio, label, nd['id']))

        # 2) Sort by priority then label
        screws.sort(key=lambda x: (x[0], x[1]))
        self._sequence = [nid for _,_,nid in screws]

        # 3) Publish sequence labels for logging/UI
        labels = [s[1] for s in screws]
        self.seq_pub.publish(String(data=json.dumps(labels)))
        self.get_logger().info(f"🔄 New sequence: {labels}")

    def handle_get_sequence(self, request, response):
        response.sequence = self._sequence
        return response


def main():
    rclpy.init()
    node = SequencePlannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
