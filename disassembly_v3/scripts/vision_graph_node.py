#!/usr/bin/env python3

"""
VisionGraphNode: 
Runs YOLOv11 on synchronized color+depth images to build a 3D proximity graph of detected parts; 
publishes /part_graph and offers GetPartGraph service.
"""

import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge
import message_filters
import numpy as np
import networkx as nx
import cv2

# generated service
from disassembly_v3.srv import GetPartGraph

# ultralytics YOLO import
from ultralytics import YOLO

class VisionGraphNode(Node):
    def __init__(self):
        super().__init__('vision_graph_node')
        # 1) Mode parameter: sim or real
        self.declare_parameter('mode', 'sim')
        mode = self.get_parameter('mode').get_parameter_value().string_value

        # 2) Topics based on mode
        if mode == 'real':
            cam_info_topic = '/camera/camera/color/camera_info'
            color_topic    = '/camera/camera/color/image_raw'
            depth_topic    = '/camera/camera/depth/image_rect_raw'
        else:
            cam_info_topic = '/xarm5_d435/camera_info'
            color_topic    = '/xarm5_d435/color/image_raw'
            depth_topic    = '/xarm5_d435/depth/image_rect_raw'

        # 3) YOLO model + tracker
        self.model = YOLO(
            '/home/adip/workspaces/image_processing_ws/HardDrive_Segmentation/'
            'runs/segment/train6/weights/best.pt')
        self.model.fuse()
        self.model.tracker = 'bytetrack.yaml'

        # 4) For image conversion & intrinsics
        self.bridge = CvBridge()
        self.intrinsics = None

        # 5) Sync CameraInfo, Color, Depth
        info_sub = message_filters.Subscriber(self, CameraInfo, cam_info_topic)
        col_sub  = message_filters.Subscriber(self, Image,      color_topic)
        dep_sub  = message_filters.Subscriber(self, Image,      depth_topic)
        ts = message_filters.ApproximateTimeSynchronizer(
            [info_sub, col_sub, dep_sub], 10, 0.1
        )
        ts.registerCallback(self.image_callback)

        # 6) Publishers
        self.graph_pub = self.create_publisher(String, '/part_graph', 10)
        self.img_pub   = self.create_publisher(Image,  '/annotated_image', 10)

        # 7) Cache last graph JSON
        self._last_graph = json.dumps({'nodes': [], 'edges': []})

        # 8) Service: GetPartGraph
        self._srv = self.create_service(
            GetPartGraph,
            'GetPartGraph',
            self.handle_get_graph
        )

        self.get_logger().info('✅ VisionGraphNode ready')

    def handle_get_graph(self, request, response):
        response.json_graph = self._last_graph
        return response

    def image_callback(self, cam_info, color_msg, depth_msg):
        # 1) Read intrinsics once
        if self.intrinsics is None:
            k = cam_info.k
            self.intrinsics = (k[0], k[4], k[2], k[5])  # fx, fy, cx, cy

        # 2) Convert images
        color = self.bridge.imgmsg_to_cv2(color_msg, 'bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')

        # 3) YOLO tracking
        results = self.model.track(color, persist=True)[0]
        detections = []

        # 4) Parse detections
        if results.boxes and len(results.boxes.cls) > 0:
            boxes   = results.boxes.xyxy.cpu().numpy().astype(int)
            classes = results.boxes.cls.cpu().numpy().astype(int)
            ids     = (results.boxes.id.cpu().numpy().astype(int)
                       if results.boxes.id is not None
                       else np.arange(len(boxes)))
            names   = results.names
            fx, fy, cx, cy = self.intrinsics

            for i, cls_id in enumerate(classes):
                cls_name = names[cls_id]
                if cls_name.lower() == 'label':
                    continue

                x1, y1, x2, y2 = boxes[i]
                cx_px = (x1 + x2)//2
                cy_px = (y1 + y2)//2
                raw_z = depth[cy_px, cx_px]
                z = float(raw_z) / 1000.0 if depth.dtype == np.uint16 else float(raw_z)
                if z <= 0 or np.isnan(z):
                    continue

                X = (cx_px - cx)*z/fx
                Y = (cy_px - cy)*z/fy
                nid = f"part_{ids[i]}"

                detections.append({
                    'id':       nid,
                    'label':    f"{cls_name}_{ids[i]}",
                    'position': (X, Y, z)
                })

                # draw annotations
                cv2.rectangle(color, (x1,y1), (x2,y2), (0,0,255), 2)
                cv2.circle(color, (cx_px,cy_px), 4, (0,0,255), -1)
                txt = f"{cls_name}_{ids[i]}:({X:.2f},{Y:.2f},{z:.2f})"
                cv2.putText(color, txt, (x1,y1-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)

        # 5) Build proximity graph
        G = nx.Graph()
        for det in detections:
            G.add_node(det['id'],
                       label=det['label'],
                       position=det['position'])
        # connect nodes closer than 5 cm
        for i in range(len(detections)):
            for j in range(i+1, len(detections)):
                p1 = np.array(detections[i]['position'])
                p2 = np.array(detections[j]['position'])
                if np.linalg.norm(p1 - p2) < 0.05:
                    G.add_edge(detections[i]['id'], detections[j]['id'])

        # 6) Publish graph JSON
        graph_dict = {
            'nodes': [{'id': n,
                       'label': G.nodes[n]['label'],
                       'position': G.nodes[n]['position']}
                      for n in G.nodes()],
            'edges': [{'source': u, 'target': v} for u,v in G.edges()]
        }
        js = json.dumps(graph_dict)
        self._last_graph = js
        self.graph_pub.publish(String(data=js))

        # 7) Publish annotated image
        img_msg = self.bridge.cv2_to_imgmsg(color, 'bgr8')
        img_msg.header = color_msg.header
        self.img_pub.publish(img_msg)


def main():
    rclpy.init()
    node = VisionGraphNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
