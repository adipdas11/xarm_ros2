#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose
from ultralytics import YOLO
import message_filters

CLASS_COLORS = {
    0: (255, 0, 0), 1: (0, 255, 0), 2: (0, 0, 255), 3: (255, 255, 0),
    4: (255, 0, 255), 5: (0, 255, 255), 6: (128, 0, 128),
    7: (255, 165, 0), 8: (0, 128, 128), 9: (128, 128, 0),
    10: (0, 0, 128), 11: (0, 128, 0)
}

class ComponentTracker(Node):
    def __init__(self):
        super().__init__('component_tracker')
        self.bridge = CvBridge()

        # Declare parameter to switch between 'isaac' (sim) and 'real' modes
        self.declare_parameter('device', 'isaac')
        device = self.get_parameter('device').get_parameter_value().string_value
        self.get_logger().info(f"📡 Running in '{device}' mode.")

        # Set topic names and resize flag based on mode
        if device == 'isaac':
            camera_info_topic = '/xarm5/D435_1/camera_info'
            rgb_topic          = '/xarm5/D435_1/color/image_raw'
            depth_topic        = '/xarm5/D435_1/depth/image_rect_raw'
            self.isaac_mode    = True
        else:
            camera_info_topic = '/camera/camera/depth/camera_info'
            rgb_topic          = '/camera/camera/color/image_raw'
            depth_topic        = '/camera/camera/depth/image_rect_raw'
            self.isaac_mode    = False

        # Load YOLO model
        self.model = YOLO('/home/adip/workspaces/image_processing_ws/HardDrive_Segmentation/runs/segment/train/weights/best.pt')
        self.get_logger().info("✅ YOLOv11 model loaded.")

        # Camera intrinsics
        self.fx = self.fy = self.cx = self.cy = None
        self.intrinsics_received = False

        # Track ID mapping
        self.classwise_id_map = {}
        self.classwise_next_id = {}

        # Publishers
        self.image_pub     = self.create_publisher(Image,           '/yolov11/tracked_image',    10)
        self.pose_only_pub = self.create_publisher(Image,           '/yolov11/pose_overlay_image',10)
        self.detection_pub = self.create_publisher(Detection3DArray,'/yolov11/part_detections',   10)

        # Subscriptions
        self.create_subscription(CameraInfo, camera_info_topic, self.camera_info_callback, 10)
        rgb_sub   = message_filters.Subscriber(self, Image, rgb_topic)
        depth_sub = message_filters.Subscriber(self, Image, depth_topic)
        ts = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub], 10, 0.1)
        ts.registerCallback(self.image_callback)

    def camera_info_callback(self, msg):
        if not self.intrinsics_received:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.intrinsics_received = True
            self.get_logger().info(f"✅ Intrinsics: fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}")

    def image_callback(self, rgb_msg, depth_msg):
        if not self.intrinsics_received:
            self.get_logger().warn("Waiting for camera intrinsics...")
            return

        # Convert ROS → OpenCV
        rgb   = self.bridge.imgmsg_to_cv2(rgb_msg,   desired_encoding='bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

        # If running in simulation (Isaac), down‐sample from 720×1280 → 480×640
        if self.isaac_mode:
            rgb   = cv2.resize(rgb,   (640, 480), interpolation=cv2.INTER_AREA)
            depth = cv2.resize(depth, (640, 480), interpolation=cv2.INTER_NEAREST)

        # Perform detection + tracking
        results = self.model.track(rgb, task="segment", persist=True, tracker="botsort.yaml")[0]
        if results.masks is None or results.boxes is None:
            return

        annotated    = rgb.copy()
        pose_overlay = rgb.copy()
        masks        = results.masks.data.cpu().numpy()
        boxes        = results.boxes

        detection_array = Detection3DArray()
        detection_array.header.frame_id = "camera_color_optical_frame"
        detection_array.header.stamp    = self.get_clock().now().to_msg()

        H, W = rgb.shape[:2]

        for i in range(len(masks)):
            cls     = int(boxes.cls[i])
            label   = self.model.names[cls]
            conf    = float(boxes.conf[i])
            orig_id = int(boxes.id[i]) if boxes.id is not None else -1

            # Remap track IDs per class
            if label not in self.classwise_id_map:
                self.classwise_id_map[label]     = {}
                self.classwise_next_id[label]    = 1
            if orig_id not in self.classwise_id_map[label]:
                self.classwise_id_map[label][orig_id] = self.classwise_next_id[label]
                self.classwise_next_id[label]         += 1
            remap_id    = self.classwise_id_map[label][orig_id]
            track_label = f"{label}_{remap_id}"

            color = CLASS_COLORS.get(cls % len(CLASS_COLORS), (200,200,200))

            # Resize mask to full image resolution
            raw_mask = (masks[i] > 0.5).astype(np.uint8)
            full_mask = cv2.resize(raw_mask, (W, H), interpolation=cv2.INTER_NEAREST).astype(bool)

            # Annotate mask overlay
            mask_overlay = np.zeros_like(annotated, dtype=np.uint8)
            mask_overlay[full_mask] = color
            annotated = cv2.addWeighted(annotated, 1.0, mask_overlay, 0.4, 0)

            # Draw bounding box + label
            x1,y1,x2,y2 = map(int, boxes.xyxy[i])
            cv2.rectangle(annotated, (x1,y1), (x2,y2), color, 2)
            cv2.putText(annotated, f"{track_label} ({conf:.2f})", (x1, y1-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            # Compute 3D centroid
            ys, xs = np.where(full_mask)
            zs = depth[ys, xs].astype(np.float32) / 1000.0
            valid = zs > 0
            xs, ys, zs = xs[valid], ys[valid], zs[valid]
            if len(zs) == 0:
                continue
            x3d = (xs - self.cx) * zs / self.fx
            y3d = (ys - self.cy) * zs / self.fy
            centroid = np.mean(np.vstack((x3d, y3d, zs)).T, axis=0)

            self.get_logger().info(f"📍 {track_label}: x={centroid[0]:.3f}, y={centroid[1]:.3f}, z={centroid[2]:.3f}")
            cv2.putText(pose_overlay,
                        f"{track_label}: ({centroid[0]:.2f}, {centroid[1]:.2f}, {centroid[2]:.2f})",
                        (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            # Populate Detection3D message
            pose = Pose()
            pose.position.x = float(centroid[0])
            pose.position.y = float(centroid[1])
            pose.position.z = float(centroid[2])
            pose.orientation.w = 1.0

            hypo = ObjectHypothesisWithPose()
            hypo.hypothesis.class_id = track_label
            hypo.pose.pose = pose

            detection = Detection3D()
            detection.header = detection_array.header
            detection.results.append(hypo)
            detection_array.detections.append(detection)

        # Publish outputs
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(annotated,    encoding='bgr8'))
        self.pose_only_pub.publish(self.bridge.cv2_to_imgmsg(pose_overlay,encoding='bgr8'))
        self.detection_pub.publish(detection_array)

def main(args=None):
    rclpy.init(args=args)
    node = ComponentTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
