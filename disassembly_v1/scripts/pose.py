#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import open3d as o3d
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from ultralytics import YOLO
from scipy.spatial.transform import Rotation as R
import message_filters

class LidPoseNode(Node):
    def __init__(self):
        super().__init__('lid_pose_node')
        self.bridge = CvBridge()

        # Load YOLO model
        self.model = YOLO('/home/adip/workspaces/image_processing_ws/HardDrive_Segmentation/runs/segment/train/weights/best.pt')
        self.get_logger().info("✅ YOLOv11 loaded")

        # Intrinsics
        self.fx = self.fy = self.cx = self.cy = None
        self.intrinsics_received = False
        self.create_subscription(CameraInfo, '/camera/camera/depth/camera_info', self.camera_info_callback, 10)

        # Subscribers
        rgb_sub = message_filters.Subscriber(self, Image, '/camera/camera/color/image_raw')
        depth_sub = message_filters.Subscriber(self, Image, '/camera/camera/depth/image_rect_raw')
        self.sync = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub], 10, 0.1)
        self.sync.registerCallback(self.image_callback)

        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/lid_pose', 10)
        self.marker_pub = self.create_publisher(Marker, '/lid_marker', 10)

    def camera_info_callback(self, msg):
        if self.intrinsics_received:
            return
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.intrinsics_received = True
        self.get_logger().info(f"✅ Intrinsics received: fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}")

    def image_callback(self, rgb_msg, depth_msg):
        if not self.intrinsics_received:
            self.get_logger().warn("Waiting for intrinsics...")
            return

        rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        results = self.model(rgb, task="segment", verbose=False)[0]

        if results.masks is None:
            return

        masks = results.masks.data.cpu().numpy()
        boxes = results.boxes

        for i in range(len(masks)):
            cls_id = int(boxes.cls[i])
            label = self.model.names[cls_id]

            if label.lower() != "lid":
                continue

            mask = masks[i]
            ys, xs = np.where(mask > 0.5)
            if len(xs) < 10:
                continue
            zs = depth[ys, xs].astype(np.float32) / 1000.0
            valid = zs > 0
            xs, ys, zs = xs[valid], ys[valid], zs[valid]
            if len(zs) < 10:
                continue

            x3d = (xs - self.cx) * zs / self.fx
            y3d = (ys - self.cy) * zs / self.fy
            points = np.vstack((x3d, y3d, zs)).T
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)

            try:
                obb = pcd.get_oriented_bounding_box()
            except:
                self.get_logger().warn("⚠️ Skipping lid: OBB failed")
                continue

            pose = PoseStamped()
            pose.header.frame_id = "camera_color_optical_frame"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = obb.center
            quat = R.from_matrix(obb.R).as_quat()
            pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = quat

            self.pose_pub.publish(pose)
            self.publish_marker(pose)

    def publish_marker(self, pose):
        frame_id = pose.header.frame_id
        timestamp = pose.header.stamp
        position = pose.pose.position
        orientation = pose.pose.orientation

        # Convert quaternion to rotation matrix
        from scipy.spatial.transform import Rotation as R
        rot = R.from_quat([orientation.x, orientation.y, orientation.z, orientation.w])
        rot_matrix = rot.as_matrix()

        axes = ['x', 'y', 'z']
        colors = {
            'x': (1.0, 0.0, 0.0),  # Red
            'y': (0.0, 1.0, 0.0),  # Green
            'z': (0.0, 0.0, 1.0)   # Blue
        }

        axis_length = 0.1
        for i, axis in enumerate(axes):
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = timestamp
            marker.ns = "lid_pose_axes"
            marker.id = i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.scale.x = axis_length
            marker.scale.y = 0.01
            marker.scale.z = 0.01
            r, g, b = colors[axis]
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            marker.color.a = 1.0

            # Start and end of the arrow
            start = np.array([position.x, position.y, position.z])
            direction = rot_matrix[:, i] * axis_length
            end = start + direction

            from geometry_msgs.msg import Point
            marker.points.append(Point(x=start[0], y=start[1], z=start[2]))
            marker.points.append(Point(x=end[0], y=end[1], z=end[2]))

            self.marker_pub.publish(marker)



def main(args=None):
    rclpy.init(args=args)
    node = LidPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
