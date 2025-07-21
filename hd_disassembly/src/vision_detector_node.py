#!/usr/bin/env python3
"""
vision_tracker_node.py

A combined vision and tracking node for ROS2 that:
  - Loads YOLO segmentation model and camera parameters
  - Subscribes to RGB and depth image topics and camera info
  - Runs YOLO inference, draws bounding boxes and segmentation masks
  - Publishes the annotated image for RViz visualization (/vision_debug/image)
  - Projects detections to 3D using depth + intrinsics
  - Tracks parts per class with persistent IDs
  - Publishes TrackedPart messages (/tracked_parts)
  - Publishes JSON summary of all tracks (/vision_tracks)
"""
import os
import yaml
import json
import warnings

# Silence only the Axes3D import warning from matplotlib
warnings.filterwarnings(
    "ignore",
    category=UserWarning,
    message=".*Unable to import Axes3D.*"
)

# Use headless matplotlib backend
import matplotlib
matplotlib.use("Agg")

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
import numpy as np
import cv2

from hd_disassembly.msg import TrackedPart
from geometry_msgs.msg import PointStamped

class VisionTrackerNode(Node):
    def __init__(self):
        super().__init__('vision_tracker_node')
        self.get_logger().info("🚀 VisionTrackerNode starting up…")

        # Load config
        pkg = get_package_share_directory('hd_disassembly')
        cfg_file = os.path.join(pkg, 'config', 'disassembler_params.yaml')
        with open(cfg_file, 'r') as f:
            cfg = yaml.safe_load(f)

        # Parameters
        self.declare_parameter('vision.model_path', cfg['vision']['model_path'])
        self.declare_parameter('vision.ignore_classes', cfg['vision']['ignore_classes'])
        cam = cfg['vision']['camera']
        self.declare_parameter('vision.camera.rgb_topic',   cam['rgb_topic'])
        self.declare_parameter('vision.camera.depth_topic', cam['depth_topic'])
        self.declare_parameter('vision.camera.info_topic',  cam['info_topic'])
        self.declare_parameter('vision.camera.frame_id',    cam['frame_id'])
        self.declare_parameter('tracker.iou_threshold',     cfg['tracker']['iou_threshold'])
        self.declare_parameter('tracker.max_lost',          cfg['tracker']['max_lost'])

        # Get parameter values
        model_path = self.get_parameter('vision.model_path').value
        ignore_cls = self.get_parameter('vision.ignore_classes').value
        self.rgb_topic    = self.get_parameter('vision.camera.rgb_topic').value
        self.depth_topic  = self.get_parameter('vision.camera.depth_topic').value
        self.info_topic   = self.get_parameter('vision.camera.info_topic').value
        self.camera_frame = self.get_parameter('vision.camera.frame_id').value
        self.iou_thres    = self.get_parameter('tracker.iou_threshold').value
        self.max_lost     = self.get_parameter('tracker.max_lost').value
        self.ignore       = set(ignore_cls)

        # Tracking state
        self.tracks  = {}  # label -> {id -> {'bbox','lost','pos'}}
        self.next_id = {}  # label -> next ID

        # Setup CV and model
        self.bridge = CvBridge()
        try:
            self.model = YOLO(model_path)
            self.get_logger().info(f"✅ Loaded YOLO model: {model_path}")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to load model: {e}")
            raise

        self.K = None
        self.depth = None

        # Subscribers
        self.create_subscription(CameraInfo, self.info_topic, self.info_cb,   10)
        self.create_subscription(Image,      self.depth_topic, self.depth_cb,  10)
        self.create_subscription(Image,      self.rgb_topic,   self.color_cb,  10)

        # Publishers
        self.debug_pub   = self.create_publisher(Image,      'vision_debug/image', 10)
        self.json_pub    = self.create_publisher(String,     'vision_tracks',      10)
        self.tracked_pub = self.create_publisher(TrackedPart, 'tracked_parts',     10)

        self.get_logger().info("✅ VisionTrackerNode ready.")

    def info_cb(self, msg: CameraInfo):
        # Camera intrinsics
        self.K = np.array(msg.k).reshape((3,3))
        self.get_logger().info("🔧 Camera intrinsics set")

    def depth_cb(self, msg: Image):
        try:
            raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except CvBridgeError as e:
            self.get_logger().error(f"Depth conversion error: {e}")
            return
        if raw.dtype == np.uint16:
            self.depth = raw.astype(np.float32) * 0.001
        else:
            self.depth = raw.astype(np.float32)

    def color_cb(self, msg: Image):
        # Ensure we have intrinsics and depth
        if self.K is None or self.depth is None:
            return

        # Convert color image
        try:
            raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except CvBridgeError as e:
            self.get_logger().error(f"Color conversion error: {e}")
            return
        color = raw if msg.encoding=='rgb8' else cv2.cvtColor(raw, cv2.COLOR_BGR2RGB)

        # Run inference
        results = self.model(color)[0]

        # Annotate image (boxes & masks)
        annotated = results.plot()  # draws bboxes + masks
        try:
            ann_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='rgb8')
            ann_msg.header = msg.header
            self.debug_pub.publish(ann_msg)
        except CvBridgeError as e:
            self.get_logger().error(f"Annotated image conversion failed: {e}")

        # Collect detections and project to 3D
        dets3d = {}
        h, w = color.shape[:2]
        for box, cls in zip(results.boxes.xyxy, results.boxes.cls):
            label = self.model.names[int(cls)]
            if label in self.ignore:
                continue
            x1,y1,x2,y2 = map(int, box)
            cx, cy = (x1+x2)//2, (y1+y2)//2
            z = float(self.depth[cy, cx])
            if z<=0 or np.isnan(z):
                continue
            fx,fy = self.K[0,0], self.K[1,1]
            cx0,cy0 = self.K[0,2], self.K[1,2]
            X = (cx-cx0)*z/fx; Y = (cy-cy0)*z/fy
            dets3d.setdefault(label, []).append(((x1,y1,x2,y2),(X,Y,z)))

        # Track and publish
        tracks_out = []
        for label, dets in dets3d.items():
            self.tracks.setdefault(label, {});
            self.next_id.setdefault(label, 0)
            new_tracks, assigned = {}, set()
            for bbox,pos in dets:
                # IOU matching
                best, biou = None, self.iou_thres
                for tid,data in self.tracks[label].items():
                    i = self.iou(bbox, data['bbox'])
                    if i>biou: biou, best = i, tid
                if best is None:
                    best = self.next_id[label]; self.next_id[label]+=1
                new_tracks[best] = {'bbox':bbox,'lost':0,'pos':pos}
                assigned.add(best)
                # Publish TrackedPart
                tp = TrackedPart(); tp.header=msg.header; tp.track_id=best; tp.part_label=label
                p=PointStamped(); p.header=msg.header; p.point.x,p.point.y,p.point.z=pos
                tp.position=p; self.tracked_pub.publish(tp)
                tracks_out.append({'id':best,'part':label,'position':{'x':pos[0],'y':pos[1],'z':pos[2]}})
            # prune
            for tid,data in self.tracks[label].items():
                if tid not in assigned:
                    if data['lost']+1<self.max_lost:
                        new_tracks[tid]={'bbox':data['bbox'],'lost':data['lost']+1,'pos':data['pos']}
            self.tracks[label]=new_tracks
        # Publish JSON track list
        msgj = String(); msgj.data=json.dumps(tracks_out); self.json_pub.publish(msgj)

    @staticmethod
    def iou(A, B):
        xA, yA = max(A[0],B[0]), max(A[1],B[1])
        xB, yB = min(A[2],B[2]), min(A[3],B[3])
        inter = max(0, xB-xA)*max(0, yB-yA)
        aA=(A[2]-A[0])*(A[3]-A[1]); aB=(B[2]-B[0])*(B[3]-B[1])
        return inter/(aA+aB-inter+1e-8)


def main(args=None):
    rclpy.init(args=args)
    node = VisionTrackerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node(); rclpy.shutdown()

if __name__=='__main__':
    main()