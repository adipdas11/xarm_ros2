#!/usr/bin/env python3
"""
planner_node.py
ROS2 node that subscribes to tracked part detections, builds a part-priority graph,
performs topological sorting, and serves the disassembly sequence via a service.
"""
import os
import yaml
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from hd_disassembly.srv import GetSequence
from ament_index_python.packages import get_package_share_directory
from collections import defaultdict, deque

class PlannerNode(Node):
    def __init__(self):
        super().__init__('planner_node')
        self.get_logger().info("🚀 PlannerNode starting up…")

        # Load parameters from YAML
        pkg_share = get_package_share_directory('hd_disassembly')
        yaml_path = os.path.join(pkg_share, 'config', 'disassembler_params.yaml')
        with open(yaml_path, 'r') as f:
            cfg = yaml.safe_load(f)

        # Part priority map
        self.PART_PRIORITY = cfg['disassembly']['part_priority']
        self.get_logger().info(f"🔧 Loaded part priorities: {self.PART_PRIORITY}")

        # Internal state
        self.latest_parts = []       # list of dicts {id, part, position}
        self.sequence = []           # sorted list of part IDs
        self.index = 0               # next index in sequence

        # Subscriber to vision tracks JSON
        self.create_subscription(
            String,
            'vision_tracks',
            self.tracks_callback,
            10
        )
        self.get_logger().info("✅ Subscribed to 'vision_tracks'")

        # Service to get sequence
        self.create_service(
            GetSequence,
            'get_sequence',
            self.handle_get_sequence
        )
        self.get_logger().info("✅ Service 'get_sequence' ready")

    def tracks_callback(self, msg: String):
        try:
            parts = json.loads(msg.data)
            self.latest_parts = parts
            self.get_logger().debug(f"📥 Received {len(parts)} parts")
        except json.JSONDecodeError as e:
            self.get_logger().error(f"❌ Failed to parse vision_tracks JSON: {e}")

    def build_graph(self, parts):
        graph = defaultdict(set)
        indegree = defaultdict(int)
        # Ensure nodes exist
        for p in parts:
            pid = p['id']
            graph[pid]
        # Add edges based on priority
        for a in parts:
            for b in parts:
                pa = self.PART_PRIORITY.get(a['part'], self.PART_PRIORITY['default'])
                pb = self.PART_PRIORITY.get(b['part'], self.PART_PRIORITY['default'])
                if pa < pb:
                    if b['id'] not in graph[a['id']]:
                        graph[a['id']].add(b['id'])
                        indegree[b['id']] += 1
        return graph, indegree

    def topo_sort(self, graph, indegree):
        q = deque([n for n in graph if indegree[n] == 0])
        order = []
        while q:
            u = q.popleft()
            order.append(u)
            for v in graph[u]:
                indegree[v] -= 1
                if indegree[v] == 0:
                    q.append(v)
        return order

    def handle_get_sequence(self, req, res):
        rt = req.request_type.lower()
        self.get_logger().info(f"📩 get_sequence request: '{rt}'")

        if rt == 'start':
            if not self.latest_parts:
                self.get_logger().warn("⚠️ No parts available to build sequence")
            # Build and sort
            graph, indegree = self.build_graph(self.latest_parts)
            self.sequence = self.topo_sort(graph, indegree)
            self.index = 0
            self.get_logger().info(f"🔀 Sequence built: {self.sequence}")

        # Serve next
        if self.index < len(self.sequence):
            pid = self.sequence[self.index]
            # Find label
            label = next((p['part'] for p in self.latest_parts if p['id'] == pid), 'unknown')
            res.part_id = pid
            res.part_label = label
            res.finished = False
            self.index += 1
            self.get_logger().info(f"➡️ Next part: id={pid}, label={label}")
        else:
            res.part_id = -1
            res.part_label = ''
            res.finished = True
            self.get_logger().info("🏁 Disassembly sequence complete")

        return res


def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 PlannerNode interrupt, shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
