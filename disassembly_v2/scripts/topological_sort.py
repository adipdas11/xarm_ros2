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

def get_priority(label):
    label = label.lower()
    for key in PART_PRIORITY:
        if key in label:
            return PART_PRIORITY[key]
    return PART_PRIORITY['default']

class DSPPlannerNode(Node):
    def __init__(self):
        super().__init__('dsp_planner_node')
        self.sub = self.create_subscription(String, 'part_graph', self.part_graph_callback, 10)
        self.pub = self.create_publisher(String, 'disassembly_sequence', 10)
        self.get_logger().info('DSP Planner Node started, listening to /part_graph')

    def part_graph_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            nodes = data.get('nodes', [])
            edges = data.get('edges', [])

            DG = nx.DiGraph()

            # Add nodes
            for n in nodes:
                pid = n['id']
                label = n.get('label', 'unknown')
                DG.add_node(pid, label=label)

            # Add edges with direction based on priority
            for e in edges:
                u, v = e['source'], e['target']
                u_label = DG.nodes[u].get('label', '')
                v_label = DG.nodes[v].get('label', '')
                u_prio = get_priority(u_label)
                v_prio = get_priority(v_label)
                if u_prio < v_prio:
                    DG.add_edge(u, v)
                elif v_prio < u_prio:
                    DG.add_edge(v, u)
                # else: equal priority, no directed edge added

            # Try topological sort
            try:
                topo_order = list(nx.topological_sort(DG))
            except nx.NetworkXUnfeasible:
                self.get_logger().warn('Cycle detected in graph, cannot do topological sort. Returning unsorted nodes.')
                topo_order = list(DG.nodes())

            self.get_logger().info(f'Disassembly sequence generated with {len(topo_order)} parts.')

            out_msg = String()
            out_msg.data = json.dumps(topo_order)
            self.pub.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing part graph: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = DSPPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
