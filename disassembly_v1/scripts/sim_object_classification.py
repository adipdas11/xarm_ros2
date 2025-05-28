#!/usr/bin/env python3

import os
import json
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge
import message_filters
from ultralytics import YOLO
import numpy as np
import networkx as nx
import cv2

class YOLOv11GraphNode(Node):
    def __init__(self):
        super().__init__('yolov11_graph_node')

        self.declare_parameter('mode', 'sim')
        mode = self.get_parameter('mode').get_parameter_value().string_value
        self.get_logger().info(f"Running in {mode} mode")

        self.model_path = '/home/adip/workspaces/image_processing_ws/HardDrive_Segmentation/runs/segment/train6/weights/best.pt'

        self.model = YOLO(self.model_path)
        self.model.fuse()
        self.model.tracker = 'bytetrack.yaml'

        self.bridge = CvBridge()
        self.intrinsics = None

        # Set camera topics based on mode
        if mode == 'real':
            self.cam_info_topic = '/camera/camera/color/camera_info'
            self.color_topic = '/camera/camera/color/image_raw'
            self.depth_topic = '/camera/camera/depth/image_rect_raw'
        else:
            self.cam_info_topic = '/xarm5/D435_1/camera_info'
            self.color_topic = '/xarm5/D435_1/color/image_raw'
            self.depth_topic = '/xarm5/D435_1/depth/image_rect_raw'

        cam_info_sub = message_filters.Subscriber(self, CameraInfo, self.cam_info_topic)
        color_sub    = message_filters.Subscriber(self, Image, self.color_topic)
        depth_sub    = message_filters.Subscriber(self, Image, self.depth_topic)

        ts = message_filters.ApproximateTimeSynchronizer([cam_info_sub, color_sub, depth_sub], 10, 0.1)
        ts.registerCallback(self.callback)

        self.image_pub = self.create_publisher(Image, '/yolov11/annotated_image', 10)
        self.graph_pub = self.create_publisher(String, '/yolov11/part_graph', 10)

        self.get_logger().info('YOLOv11 Graph Node with tracker initialized.')

    def callback(self, cam_info, color_msg, depth_msg):
        if self.intrinsics is None:
            fx = cam_info.k[0]; fy = cam_info.k[4]
            cx = cam_info.k[2]; cy = cam_info.k[5]
            self.intrinsics = (fx, fy, cx, cy)
            self.get_logger().info(f"Intrinsics: fx={fx}, fy={fy}, cx={cx}, cy={cy}")

        color_cv = self.bridge.imgmsg_to_cv2(color_msg, 'bgr8')
        depth_cv = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')
        depth_np = np.array(depth_cv, dtype=np.float32) / 1000.0  # Convert mm to meters

        results = self.model.track(color_cv, persist=True)[0]

        detections = []
        if results.masks is not None and len(results.boxes.cls) > 0:
            masks = results.masks.data.cpu().numpy()
            boxes = results.boxes.xyxy.cpu().numpy()
            classes = results.boxes.cls.cpu().numpy().astype(int)
            track_ids = results.boxes.id.cpu().numpy().astype(int) if results.boxes.id is not None else np.arange(len(boxes))
            names = results.names

            for i, cls_id in enumerate(classes):
                class_name = names[int(cls_id)]
                if class_name.lower() == 'label':
                    continue

                x1, y1, x2, y2 = boxes[i].astype(int)
                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)

                z = float(depth_np[cy, cx])
                fx, fy, cx0, cy0 = self.intrinsics
                X = (cx - cx0) * z / fx
                Y = (cy - cy0) * z / fy
                Z = z

                track_id = f"part_{track_ids[i]}"

                detections.append({
                    'id': track_id,
                    'class': class_name,
                    'bbox': (x1, y1, x2, y2),
                    'centroid': (cx, cy),
                    'position': (X, Y, Z)
                })

                cv2.rectangle(color_cv, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.circle(color_cv, (cx, cy), 4, (0, 0, 255), -1)
                # label = f"{class_name}_{track_ids[i]}: ({cx}, {cy})"
                label = f"{class_name}_{track_ids[i]}: ({X:.2f}, {Y:.2f}, {Z:.2f})"
                cv2.putText(color_cv, label, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        G = nx.Graph()
        for det in detections:
            G.add_node(det['id'], label=f"{det['class']}_{det['id'].split('_')[1]}", position=det['position'])

        for i in range(len(detections)):
            for j in range(i + 1, len(detections)):
                d1, d2 = detections[i], detections[j]
                if np.linalg.norm(np.array(d1['position']) - np.array(d2['position'])) < 0.05:
                    G.add_edge(d1['id'], d2['id'])

        graph_msg = {
            'nodes': [{'id': nid, 'label': attr['label'], 'position': attr['position']}
                      for nid, attr in G.nodes(data=True)],
            'edges': [{'source': u, 'target': v} for u, v in G.edges()]
        }

        gmsg = String()
        gmsg.data = json.dumps(graph_msg)
        self.graph_pub.publish(gmsg)

        img_msg = self.bridge.cv2_to_imgmsg(color_cv, 'bgr8')
        self.image_pub.publish(img_msg)

        self.get_logger().info('Published annotated image and part graph.')


def main(args=None):
    rclpy.init(args=args)
    node = YOLOv11GraphNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
