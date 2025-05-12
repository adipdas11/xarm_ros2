#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header
from ultralytics import YOLO
import struct

import message_filters

CLASS_COLORS = {
    0: (255, 0, 0), 1: (0, 255, 0), 2: (0, 0, 255), 3: (255, 255, 0),
    4: (255, 0, 255), 5: (0, 255, 255), 6: (128, 0, 128),
    7: (255, 165, 0), 8: (0, 128, 128), 9: (128, 128, 0),
    10: (0, 0, 128), 11: (0, 128, 0)
}

class SimpleYoloNode(Node):
    def __init__(self):
        super().__init__('simple_yolo_tracker_node')
        self.bridge = CvBridge()

        # Load YOLO model
        self.model = YOLO('/home/adip/workspaces/image_processing_ws/HardDrive_Segmentation/runs/segment/train/weights/best.pt')
        self.get_logger().info("✅ YOLO model loaded.")

        # Class-wise tracking
        self.classwise_id_map = {}
        self.classwise_next_id = {}

        # Subscriptions
        self.rgb_sub = message_filters.Subscriber(self, Image, '/camera/camera/color/image_raw')
        self.depth_sub = message_filters.Subscriber(self, Image, '/camera/camera/depth/image_rect_raw')
        self.sync = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], 10, 0.1)
        self.sync.registerCallback(self.image_callback)

        # Publishers
        self.image_pub = self.create_publisher(Image, '/yolov11/tracked_image', 10)
        self.pcd_pub = self.create_publisher(PointCloud2, '/yolov11/segmented_pointcloud', 10)

    def image_callback(self, rgb_msg, depth_msg):
        rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        results = self.model.track(rgb, task="segment", persist=True, tracker="botsort.yaml")[0]

        if results.masks is None:
            self.get_logger().warn("No masks detected.")
            return

        annotated = rgb.copy()
        masks = results.masks.data.cpu().numpy()
        boxes = results.boxes
        points, colors = [], []

        for i in range(len(masks)):
            conf = float(boxes.conf[i])
            cls = int(boxes.cls[i].item())
            label = self.model.names[cls]
            orig_id = int(boxes.id[i]) if boxes.id is not None else -1

            # Tracking remap
            if label not in self.classwise_id_map:
                self.classwise_id_map[label] = {}
                self.classwise_next_id[label] = 1
            if orig_id not in self.classwise_id_map[label]:
                self.classwise_id_map[label][orig_id] = self.classwise_next_id[label]
                self.classwise_next_id[label] += 1
            remap_id = self.classwise_id_map[label][orig_id]

            # Color
            color = CLASS_COLORS.get(cls % len(CLASS_COLORS), (255, 255, 255))
            mask = masks[i]
            colored_mask = np.zeros_like(annotated)
            colored_mask[mask > 0.5] = color
            annotated = cv2.addWeighted(annotated, 1.0, colored_mask, 0.4, 0)

            x1, y1, x2, y2 = map(int, boxes.xyxy[i])
            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
            cv2.putText(annotated, f"{label}_{remap_id}", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            # Point cloud from mask
            ys, xs = np.where(mask > 0.5)
            if len(xs) == 0:
                continue
            zs = depth[ys, xs].astype(np.float32) / 1000.0
            valid = zs > 0
            xs, ys, zs = xs[valid], ys[valid], zs[valid]
            if len(zs) == 0:
                continue

            fx = fy = 385.46
            cx, cy = 326.39, 236.97
            x3d = (xs - cx) * zs / fx
            y3d = (ys - cy) * zs / fy
            part_points = np.vstack((x3d, y3d, zs)).T
            points.extend(part_points)

            part_colors = np.tile(np.array(color[::-1]) / 255.0, (len(part_points), 1))
            colors.extend(part_colors)

        # Publish image
        img_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        self.image_pub.publish(img_msg)

        # Publish point cloud
        if points:
            cloud_msg = self.to_pointcloud2(points, colors, frame_id="camera_color_optical_frame")
            self.pcd_pub.publish(cloud_msg)

    def to_pointcloud2(self, points, colors, frame_id):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        data = []
        for pt, color in zip(points, colors):
            r, g, b = (color * 255).astype(np.uint8)
            rgb = struct.unpack('f', struct.pack('I', (r << 16) | (g << 8) | b))[0]
            data.append([pt[0], pt[1], pt[2], rgb])
        pc_array = np.array(data, dtype=np.float32)
        msg = PointCloud2(
            header=header,
            height=1,
            width=len(pc_array),
            fields=fields,
            is_bigendian=False,
            point_step=16,
            row_step=16 * len(pc_array),
            is_dense=True,
            data=pc_array.tobytes()
        )
        return msg

def main(args=None):
    rclpy.init(args=args)
    node = SimpleYoloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
