#!/usr/bin/env python3
import signal
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import networkx as nx
import matplotlib.pyplot as plt

# Ensure Ctrl+C works
signal.signal(signal.SIGINT, signal.SIG_DFL)

class PartGraphVisualizer(Node):
    def __init__(self):
        super().__init__('part_graph_visualizer')
        self.sub = self.create_subscription(
            String,
            'part_graph',
            self.on_graph_msg,
            10)
        # Matplotlib interactive mode
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(6,6))
        plt.show(block=False)
        self.get_logger().info('PartGraphVisualizer ready—listening on /part_graph')

    def on_graph_msg(self, msg: String):
        data = json.loads(msg.data)
        G = nx.Graph()
        # add nodes with label and position if available
        for n in data['nodes']:
            node_id = n['id']
            pos = tuple(n['position'])
            label = n.get('label', node_id)
            G.add_node(node_id, pos=pos, label=label)

        # add edges (unlabeled)
        for e in data['edges']:
            G.add_edge(e['source'], e['target'])

        # clear & draw
        self.ax.clear()
        # project 3D positions to 2D for plotting (X vs Y)
        pos2d = {nid: attr['pos'][:2] for nid, attr in G.nodes(data=True)}
        # draw nodes
        nx.draw_networkx_nodes(G, pos2d, ax=self.ax, node_size=300, node_color='skyblue')
        # draw labels using the 'label' attribute
        labels = {nid: attr['label'] for nid, attr in G.nodes(data=True)}
        nx.draw_networkx_labels(G, pos2d, labels, ax=self.ax, font_size=8)
        # draw edges
        nx.draw_networkx_edges(G, pos2d, ax=self.ax)

        self.ax.set_title(f'Part Graph ({G.number_of_nodes()} nodes, {G.number_of_edges()} edges)')
        self.ax.axis('off')
        plt.pause(0.01)


def main(args=None):
    rclpy.init(args=args)
    node = PartGraphVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down visualizer...')
    finally:
        node.destroy_node()
        plt.close('all')
        rclpy.shutdown()

if __name__=='__main__':
    main()
