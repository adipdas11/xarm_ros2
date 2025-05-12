#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
import numpy as np

from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import (
    Detection2DArray, Detection3DArray,
    Detection2D, Detection3D,
    ObjectHypothesisWithPose
)
from geometry_msgs.msg import Pose, PoseWithCovariance

from ultralytics import YOLO
import message_filters

class Simple3DTracker(Node):
    def __init__(self):
        super().__init__('simple_3d_tracker')
        self.bridge = CvBridge()

        # pick sim vs real
        self.declare_parameter('device', 'isaac')
        device = self.get_parameter('device').get_parameter_value().string_value
        self.get_logger().info(f"📡 Running in '{device}' mode")

        if device == 'isaac':
            rgb_topic         = '/xarm5/D435_1/color/image_raw'
            depth_topic       = '/xarm5/D435_1/depth/image_rect_raw'
            camera_info_topic = '/xarm5/D435_1/camera_info'
        else:
            rgb_topic         = '/camera/camera/color/image_raw'
            depth_topic       = '/camera/camera/depth/image_rect_raw'
            camera_info_topic = '/camera/camera/depth/camera_info'

        # YOLO model
        self.model = YOLO(
            '/home/adip/workspaces/image_processing_ws/'
            'HardDrive_Segmentation/runs/segment/train/weights/best.pt'
        )
        self.get_logger().info("✅ YOLO model loaded")

        # intrinsics
        self.fx = self.fy = self.cx = self.cy = None
        self.intrinsics_received = False
        self.create_subscription(
            CameraInfo, camera_info_topic,
            self.camera_info_cb, 10
        )

        # publishers
        self.image_pub      = self.create_publisher(Image,           '/simple_3d/annotated_image',  10)
        self.d2d_pub        = self.create_publisher(Detection2DArray, '/simple_3d/detections_2d',   10)
        self.d3d_pub        = self.create_publisher(Detection3DArray, '/simple_3d/detections_3d',   10)

        # sync rgb + depth
        rgb_sub   = message_filters.Subscriber(self, Image, rgb_topic)
        depth_sub = message_filters.Subscriber(self, Image, depth_topic)
        ats = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=10, slop=0.1)
        ats.registerCallback(self.image_callback)

    def camera_info_cb(self, msg: CameraInfo):
        if not self.intrinsics_received:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.intrinsics_received = True
            self.get_logger().info(
                f"✅ Intrinsics: fx={self.fx:.1f}, fy={self.fy:.1f}, "
                f"cx={self.cx:.1f}, cy={self.cy:.1f}"
            )

    def image_callback(self, rgb_msg: Image, depth_msg: Image):
        if not self.intrinsics_received:
            return

        # convert ROS→OpenCV
        frame     = self.bridge.imgmsg_to_cv2(rgb_msg,   desired_encoding='bgr8')
        depth_raw = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

        # to metres
        if depth_raw.dtype == np.uint16:
            depth_m = depth_raw.astype(np.float32) / 1000.0
        else:
            depth_m = depth_raw.astype(np.float32)

        annotated  = frame.copy()
        det2d_arr  = Detection2DArray(header=rgb_msg.header)
        det3d_arr  = Detection3DArray(header=rgb_msg.header)

        # run YOLO
        res   = self.model.track(frame, task="segment", persist=True, tracker="botsort.yaml")[0]
        boxes = res.boxes
        if boxes is None or len(boxes.xyxy) == 0:
            return

        xyxy    = boxes.xyxy
        cls_ids = boxes.cls
        confs   = boxes.conf if hasattr(boxes, 'conf') else [0.0]*len(xyxy)
        ids     = boxes.id   if boxes.id is not None else list(range(len(xyxy)))

        H, W = frame.shape[:2]

        for i in range(len(xyxy)):
            x1, y1, x2, y2 = map(int, xyxy[i])
            cx_pix = (x1 + x2) / 2.0
            cy_pix = (y1 + y2) / 2.0

            # clamp
            px = np.clip(int(cx_pix), 0, W-1)
            py = np.clip(int(cy_pix), 0, H-1)
            z  = float(depth_m[py, px])

            # back‐project
            x = (cx_pix - self.cx) * z / self.fx
            y = (cy_pix - self.cy) * z / self.fy

            cls   = int(cls_ids[i])
            label = self.model.names[cls]
            tid   = int(ids[i])
            tag   = f"{label}_{tid}"
            conf  = float(confs[i])

            self.get_logger().info(
                f"🔲 {tag}: x={x:.3f}m, y={y:.3f}m, z={z:.3f}m"
            )

            # draw
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0,255,0), 2)
            cv2.circle(annotated, (px, py), 4, (0,255,0), -1)
            cv2.putText(
                annotated,
                f"{tag}: ({x:.2f},{y:.2f},{z:.2f})",
                (x1, y1-10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0,255,0),
                2
            )

            # Detection2D
            d2 = Detection2D(header=rgb_msg.header)
            d2.bbox.center.position.x = cx_pix
            d2.bbox.center.position.y = cy_pix
            d2.bbox.size_x = float(x2 - x1)
            d2.bbox.size_y = float(y2 - y1)
            hypo2 = ObjectHypothesisWithPose()
            hypo2.hypothesis.class_id = tag
            hypo2.hypothesis.score    = conf
            d2.results.append(hypo2)
            det2d_arr.detections.append(d2)

            # Detection3D
            d3 = Detection3D(header=rgb_msg.header)
            # build a PoseWithCovariance and assign .pose
            pwc = PoseWithCovariance()
            pwc.pose = Pose()
            pwc.pose.position.x = x
            pwc.pose.position.y = y
            pwc.pose.position.z = z
            # leave covariance as zeros
            hypo3 = ObjectHypothesisWithPose()
            hypo3.hypothesis.class_id = tag
            hypo3.hypothesis.score    = conf
            hypo3.pose = pwc
            d3.results.append(hypo3)
            det3d_arr.detections.append(d3)

        # publish
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8'))
        self.d2d_pub.publish(det2d_arr)
        self.d3d_pub.publish(det3d_arr)

def main(args=None):
    rclpy.init(args=args)
    node = Simple3DTracker()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
