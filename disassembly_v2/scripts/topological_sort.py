#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import networkx as nx

PART_PRIORITY = {
    'screw': 0,
    'lid': 1,
    'pcb': 2,
    'default': 3,
}

TOOLING_PARTS = {'screw'}
MANIPULATION_PARTS = {'lid', 'pcb'}

class DisassemblyPlanNode(Node):
    def __init__(self):
        super().__init__('disassembly_plan_node')
        self.sub = self.create_subscription(
            String,
            '/yolov11/part_graph',
            self.graph_callback,
            10)
        self.pub = self.create_publisher(String, 'disassembly_plan', 10)
        self.get_logger().info('DisassemblyPlanNode ready—listening on /part_graph')

    def graph_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            G = nx.DiGraph()
            node_map = {}

            for node in data['nodes']:
                label = node.get('label', node['id'])
                part_type = label.split('_')[0]
                priority = PART_PRIORITY.get(part_type, PART_PRIORITY['default'])
                G.add_node(node['id'], label=label, priority=priority, position=node.get('position'))
                node_map[node['id']] = part_type

            for edge in data['edges']:
                G.add_edge(edge['source'], edge['target'])

            if nx.is_directed_acyclic_graph(G):
                sorted_ids = sorted(nx.topological_sort(G), key=lambda nid: G.nodes[nid]['priority'])

                task_plan = []
                for nid in sorted_ids:
                    node = G.nodes[nid]
                    label = node['label']
                    part_type = node_map[nid]
                    task_type = 'tooling_arm' if part_type in TOOLING_PARTS else 'manipulation_arm'
                    position = node.get('position', [0.0, 0.0, 0.0])
                    task_plan.append({
                        'part': label,
                        'task_type': task_type,
                        'pose': position
                    })

                out = {'tasks': task_plan}
                msg_out = String()
                msg_out.data = json.dumps(out)
                self.pub.publish(msg_out)

                self.get_logger().info(f"Published disassembly plan with {len(task_plan)} tasks.")
            else:
                self.get_logger().warn('Cycle detected in graph — cannot generate plan')

        except Exception as e:
            self.get_logger().error(f"Error generating disassembly plan: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = DisassemblyPlanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
