#!/usr/bin/env python3
"""
ROS2 node: GNN-based disassembly planner
Subscribes to 'part_graph' (JSON), runs a pretrained GNN to predict the next part to remove,
and publishes the selected part ID on 'next_part'.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import torch
import torch.nn.functional as F
from torch_geometric.data import Data
from torch_geometric.nn import GCNConv

# Define the GNN Model (example architecture)
class DisassemblyGNN(torch.nn.Module):
    def __init__(self, in_channels, hidden_channels):
        super().__init__()
        self.conv1 = GCNConv(in_channels, hidden_channels)
        self.conv2 = GCNConv(hidden_channels, hidden_channels)
        self.lin = torch.nn.Linear(hidden_channels, 1)

    def forward(self, x, edge_index):
        # x: [num_nodes, in_channels], edge_index: [2, num_edges]
        x = F.relu(self.conv1(x, edge_index))
        x = F.relu(self.conv2(x, edge_index))
        # compute a score per node
        out = self.lin(x).squeeze(-1)  # [num_nodes]
        return out

class GNNPlanner(Node):
    def __init__(self):
        super().__init__('gnn_planner')
        # Parameters
        self.declare_parameter('model_weights', 'gnn_weights.pt')
        self.declare_parameter('in_features', 4)  # e.g., (x,y,z,degree)
        self.declare_parameter('hidden_features', 32)
        weights_path = self.get_parameter('model_weights').get_parameter_value().string_value
        in_feats = self.get_parameter('in_features').get_parameter_value().integer_value
        hid_feats = self.get_parameter('hidden_features').get_parameter_value().integer_value

        # Load model
        self.model = DisassemblyGNN(in_feats, hid_feats)
        self.model.load_state_dict(torch.load(weights_path))
        self.model.eval()
        self.get_logger().info(f'Loaded GNN model from {weights_path}')

        # Subscription and publisher
        self.sub = self.create_subscription(String, 'part_graph', self.on_graph, 10)
        self.pub = self.create_publisher(String, 'next_part', 10)

    def on_graph(self, msg: String):
        data = json.loads(msg.data)
        nodes = data['nodes']
        edges = data['edges']
        num = len(nodes)
        # Build feature matrix x and map IDs->index
        id_to_idx = {}
        x_list = []
        for i, n in enumerate(nodes):
            pid = n['id']
            id_to_idx[pid] = i
            # features: position x,y,z and degree
            px, py, pz = n['position']
            degree = 0
            x_list.append([px, py, pz, degree])
        # update degree
        for e in edges:
            u = id_to_idx[e['source']]
            v = id_to_idx[e['target']]
            x_list[u][3] += 1
            x_list[v][3] += 1
        x = torch.tensor(x_list, dtype=torch.float)
        # build edge_index
        edge_index = torch.tensor([[id_to_idx[e['source']] for e in edges] +
                                   [id_to_idx[e['target']] for e in edges],
                                  [id_to_idx[e['target']] for e in edges] +
                                   [id_to_idx[e['source']] for e in edges]],
                                 dtype=torch.long)
        # Forward pass
        with torch.no_grad():
            scores = self.model(x, edge_index)
        # pick max score node
        idx = torch.argmax(scores).item()
        next_pid = nodes[idx]['id']
        self.get_logger().info(f'Next part to remove: {next_pid}')
        out_msg = String()
        out_msg.data = next_pid
        self.pub.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GNNPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
