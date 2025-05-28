#!/usr/bin/env python3

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

# import the generated service type
from disassembly_xarm5.srv import GetPartGraph


class YOLOv11GraphNode(Node):
    def __init__(self):
        super().__init__('yolov11_graph_node')

        # sim / real mode
        self.declare_parameter('mode', 'sim')
        mode = self.get_parameter('mode').get_parameter_value().string_value
        self.get_logger().info(f"Running in {mode} mode")

        # topics
        if mode == 'real':
            self.cam_info_topic = '/camera/camera/color/camera_info'
            self.color_topic    = '/camera/camera/color/image_raw'
            self.depth_topic    = '/camera/camera/depth/image_rect_raw'
        else:
            self.cam_info_topic = '/xarm5_d435/camera_info'
            self.color_topic    = '/xarm5_d435/color/image_raw'
            self.depth_topic    = '/xarm5_d435/depth/image_rect_raw'

        # model + tracker
        self.model = YOLO(
            '/home/adip/workspaces/image_processing_ws/HardDrive_Segmentation/'
            'runs/segment/train6/weights/best.pt')
        self.model.fuse()
        self.model.tracker = 'bytetrack.yaml'

        # bridge + intrinsics
        self.bridge = CvBridge()
        self.intrinsics = None

        # synchronized subscribers
        cam_info_sub = message_filters.Subscriber(self, CameraInfo, self.cam_info_topic)
        color_sub    = message_filters.Subscriber(self, Image,      self.color_topic)
        depth_sub    = message_filters.Subscriber(self, Image,      self.depth_topic)
        ts = message_filters.ApproximateTimeSynchronizer(
            [cam_info_sub, color_sub, depth_sub], 10, 0.1)
        ts.registerCallback(self.callback)

        # publishers
        self.image_pub = self.create_publisher(Image,  '/yolov11/annotated_image', 10)
        self.graph_pub = self.create_publisher(String, '/yolov11/part_graph',        10)

        # cached JSON string of last graph
        self._last_graph = json.dumps({'nodes': [], 'edges': []})

        # offer the service
        self._srv = self.create_service(
            GetPartGraph,
            'get_part_graph',
            self.handle_get_graph
        )
        self.get_logger().info("✅ /get_part_graph service ready")

        self.get_logger().info('YOLOv11 Graph Node initialized.')

    def handle_get_graph(self, request, response):
        """
        Service callback: return the last cached graph JSON.
        """
        response.json_graph = self._last_graph
        return response

    def callback(self, cam_info, color_msg, depth_msg):
        # set intrinsics once
        if self.intrinsics is None:
            fx = cam_info.k[0]; fy = cam_info.k[4]
            cx = cam_info.k[2]; cy = cam_info.k[5]
            self.intrinsics = (fx, fy, cx, cy)
            self.get_logger().info(f"Intrinsics: fx={fx}, fy={fy}, cx={cx}, cy={cy}")

        # convert images
        color_cv = self.bridge.imgmsg_to_cv2(color_msg, 'bgr8')
        depth_cv = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')

        results = self.model.track(color_cv, persist=True)[0]

        detections = []
        if results.boxes and len(results.boxes.cls) > 0:
            boxes     = results.boxes.xyxy.cpu().numpy()
            classes   = results.boxes.cls.cpu().numpy().astype(int)
            ids       = (results.boxes.id.cpu().numpy().astype(int)
                         if results.boxes.id is not None else np.arange(len(boxes)))
            names     = results.names
            fx, fy, cx0, cy0 = self.intrinsics

            for i, cls_id in enumerate(classes):
                cls_name = names[int(cls_id)]
                if cls_name.lower() == 'label':
                    continue

                x1, y1, x2, y2 = boxes[i].astype(int)
                cx_px = (x1 + x2)//2
                cy_px = (y1 + y2)//2

                raw = depth_cv[cy_px, cx_px]
                z = float(raw) / 1000.0 if depth_cv.dtype == np.uint16 else float(raw)
                if z <= 0 or np.isnan(z):
                    continue

                X = (cx_px - cx0)*z/fx
                Y = (cy_px - cy0)*z/fy
                tid = f"part_{ids[i]}"

                detections.append({
                    'id':       tid,
                    'class':    cls_name,
                    'bbox':     (int(x1),int(y1),int(x2),int(y2)),
                    'centroid': (int(cx_px),int(cy_px)),
                    'position': (X, Y, z)
                })

                # draw
                cv2.rectangle(color_cv,(x1,y1),(x2,y2),(0,0,255),2)
                cv2.circle(color_cv,(cx_px,cy_px),4,(0,0,255),-1)
                label = f"{cls_name}_{ids[i]}:({X:.2f},{Y:.2f},{z:.2f})"
                cv2.putText(color_cv,label,(x1,y1-10),
                            cv2.FONT_HERSHEY_SIMPLEX,0.8,(0,0,255),2)

        # build the graph
        G = nx.Graph()
        for det in detections:
            G.add_node(det['id'],
                       label=f"{det['class']}_{det['id'].split('_')[1]}",
                       position=det['position'])
        for i in range(len(detections)):
            for j in range(i+1, len(detections)):
                d1, d2 = detections[i], detections[j]
                if np.linalg.norm(np.array(d1['position'])-np.array(d2['position']))<0.05:
                    G.add_edge(d1['id'], d2['id'])

        graph_dict = {
            'nodes': [{'id': nid,
                       'label': attr['label'],
                       'position': attr['position']}
                      for nid, attr in G.nodes(data=True)],
            'edges': [{'source': u, 'target': v} for u, v in G.edges()]
        }

        # cache and publish
        js = json.dumps(graph_dict)
        self._last_graph = js

        gmsg = String(data=js)
        self.graph_pub.publish(gmsg)

        # log labels
        labels = [n['label'] for n in graph_dict['nodes']]
        self.get_logger().info(f"📋 Labels: {labels}")

        # publish annotated image
        img_msg = self.bridge.cv2_to_imgmsg(color_cv,'bgr8')
        img_msg.header = color_msg.header
        self.image_pub.publish(img_msg)


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
