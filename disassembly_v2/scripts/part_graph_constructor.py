#!/usr/bin/env python3
import os
import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge
import message_filters
from ultralytics import YOLO
import numpy as np
import networkx as nx
import cv2
from cv2 import dilate

class PartGraphConstructor(Node):
    def __init__(self):
        super().__init__('part_graph_constructor')

        # --- Parameters ---
        self.declare_parameter('model_path',
            '/home/adip/workspaces/image_processing_ws/HardDrive_Segmentation/' +
            'runs/segment/train/weights/best.pt')
        self.declare_parameter('track_dist_threshold', 0.02)
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.track_dist = self.get_parameter('track_dist_threshold').get_parameter_value().double_value

        # Load YOLOv11 model
        self.get_logger().info(f'Loading YOLO model from {model_path}...')
        self.model = YOLO(model_path)

        # CvBridge for ROS<->CV
        self.bridge = CvBridge()

        # Camera intrinsics storage
        self.intrinsics = None  # (fx, fy, cx, cy)

        # Tracking state: id -> pos, id -> class_name
        self.tracked_parts = {}
        self.tracked_labels = {}
        self.next_id = 0

        # Subscribers for RGB-D and camera info
        cam_info_sub = message_filters.Subscriber(self, CameraInfo,
                                                  '/camera/camera/color/camera_info')
        color_sub    = message_filters.Subscriber(self, Image,
                                                  '/camera/camera/color/image_raw')
        depth_sub    = message_filters.Subscriber(self, Image,
                                                  '/camera/camera/depth/image_rect_raw')
        ts = message_filters.ApproximateTimeSynchronizer(
            [cam_info_sub, color_sub, depth_sub], queue_size=10, slop=0.1)
        ts.registerCallback(self.callback)

        # Publishers: graph JSON and annotated image
        self.graph_pub = self.create_publisher(String, 'part_graph', 10)
        self.image_pub = self.create_publisher(Image, 'part_graph_image', 10)

        self.get_logger().info('PartGraphConstructor initialized.')

    def callback(self, cam_info: CameraInfo, color_msg: Image, depth_msg: Image):
        # Initialize intrinsics
        if self.intrinsics is None:
            fx = cam_info.k[0]; fy = cam_info.k[4]
            cx = cam_info.k[2]; cy = cam_info.k[5]
            self.intrinsics = (fx, fy, cx, cy)
            self.get_logger().info(f'Intrinsics set: fx={fx}, fy={fy}, cx={cx}, cy={cy}')

        # Convert images to CV
        color_cv = self.bridge.imgmsg_to_cv2(color_msg, 'bgr8')
        depth_cv = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')
        depth_np = np.array(depth_cv, dtype=np.float32) / 1000.0

        # YOLO inference
        results = self.model(color_cv)[0]
        masks = results.masks.data.cpu().numpy() if results.masks is not None else None
        boxes = results.boxes.xyxy.cpu().numpy()
        classes = results.boxes.cls.cpu().numpy().astype(int)
        names = results.names

        detections = []
        # Detect & track
        for idx, box in enumerate(boxes):
            class_name = names[classes[idx]]
            # ignore detections of class 'label'
            if class_name == 'label':
                continue

            # get mask or fallback bbox mask
            if masks is not None:
                mask = masks[idx].astype(bool)
            else:
                x1,y1,x2,y2 = map(int, box)
                mask = np.zeros(color_cv.shape[:2], bool)
                mask[y1:y2, x1:x2] = True

            ys, xs = np.where(mask)
            if ys.size == 0:
                continue
            cy2d, cx2d = int(ys.mean()), int(xs.mean())
            z = float(depth_np[cy2d, cx2d])
            fx, fy, cx, cy = self.intrinsics
            X = (cx2d - cx) * z / fx
            Y = (cy2d - cy) * z / fy
            Z = z

            # assign track ID based on proximity
            assigned_id = None
            for pid, pos in self.tracked_parts.items():
                if np.linalg.norm(np.array(pos) - np.array((X, Y, Z))) < self.track_dist:
                    assigned_id = pid
                    break
            if assigned_id is None:
                assigned_id = f'part_{self.next_id}'
                self.next_id += 1

            # update tracking
            self.tracked_parts[assigned_id] = (X, Y, Z)
            self.tracked_labels[assigned_id] = class_name
            detections.append({
                'id': assigned_id,
                'pos': (X, Y, Z),
                'mask': mask,
                'bbox': box.astype(int),
                'centroid': (cx2d, cy2d)
            })

        # Build adjacency graph
        G = nx.Graph()
        for det in detections:
            pid = det['id']
            # label for graph: <class>_<id index>
            idx = pid.split('_')[1]
            node_label = f"{self.tracked_labels[pid]}_{idx}"
            G.add_node(pid, label=node_label, position=det['pos'])

        ker = np.ones((5,5), np.uint8)
        for i in range(len(detections)):
            for j in range(i+1, len(detections)):
                mi = detections[i]['mask']; mj = detections[j]['mask']
                yi, xi = np.where(mi); yj, xj = np.where(mj)
                if not yi.size or not yj.size:
                    continue
                if (xi.max()<xj.min() or xj.max()<xi.min() or
                    yi.max()<yj.min() or yj.max()<yi.min()):
                    continue
                if np.any(dilate(mi.astype(np.uint8), ker).astype(bool) & mj):
                    G.add_edge(detections[i]['id'], detections[j]['id'])

        # Publish graph JSON
        graph = {'nodes': [], 'edges': []}
        for nid, attr in G.nodes(data=True):
            graph['nodes'].append({'id': nid, 'label': attr['label'], 'position': attr['position']})
        for u, v in G.edges():
            graph['edges'].append({'source': u, 'target': v})
        gmsg = String(); gmsg.data = json.dumps(graph)
        self.graph_pub.publish(gmsg)

        # Annotate image with tracked names
        for det in detections:
            x1,y1,x2,y2 = det['bbox']
            pid = det['id']
            idx = pid.split('_')[1]
            lbl = f"{self.tracked_labels[pid]}_{idx}"
            # draw box and label
            cv2.rectangle(color_cv, (x1,y1), (x2,y2), (0,255,0), 2)
            cv2.putText(color_cv, lbl, (x1, y1-6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)

        # Publish annotated image
        img_msg = self.bridge.cv2_to_imgmsg(color_cv, 'bgr8')
        self.image_pub.publish(img_msg)

        self.get_logger().info(f'Published graph & annotated image with tracked names.')


def main(args=None):
    rclpy.init(args=args)
    node = PartGraphConstructor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
