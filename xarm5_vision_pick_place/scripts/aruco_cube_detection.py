#!/usr/bin/env python3

import sys
import time
import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from visualization_msgs.msg import Marker

import tf2_ros
import tf2_geometry_msgs  # registers PoseStamped transformations
from scipy.spatial.transform import Rotation as SciRot


class ArucoDetectNode(Node):
    def __init__(self):
        super().__init__('aruco_detect_node')

        # declare and read 'mode' parameter
        self.declare_parameter('mode', 'sim')
        mode = self.get_parameter('mode').value
        if mode not in ('sim', 'real'):
            self.get_logger().error(f"Invalid mode '{mode}', must be 'sim' or 'real'")
            rclpy.shutdown()
            sys.exit(1)

        # select camera topics based on mode
        if mode == 'sim':
            cam_info_topic = '/xarm5/D435_1/camera_info'
            color_topic    = '/xarm5/D435_1/color/image_raw'
            depth_topic    = '/xarm5/D435_1/depth/image_rect_raw'
        else:  # real
            cam_info_topic = '/camera/camera/color/camera_info'
            color_topic    = '/camera/camera/color/image_raw'
            depth_topic    = '/camera/camera/depth/image_rect_raw'

        self.get_logger().info(f"ArucoDetectNode running in '{mode}' mode")
        self.get_logger().info(f"Subscribing to: {cam_info_topic}, {color_topic}, {depth_topic}")

        # Constants
        self.cube_marker_length = 0.03
        self.box_marker_length  = 0.10
        self.camera_frame       = 'camera_color_optical_frame'
        self.base_frame         = 'link_base'

        # TF2: listener for camera→base, and broadcaster for aruco→base
        self.tf_buffer      = tf2_ros.Buffer()
        self.tf_listener    = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # CV bridge
        self.bridge = CvBridge()

        # Camera intrinsics
        self.camera_matrix = None
        self.dist_coeffs   = None

        # latest depth image
        self.latest_depth = None

        # ArUco
        self.aruco_dict   = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()

        # Subscribers
        self.create_subscription(CameraInfo,
                                 cam_info_topic,
                                 self.camera_info_callback, 10)
        self.create_subscription(Image,
                                 color_topic,
                                 self.image_callback, 10)
        self.create_subscription(Image,
                                 depth_topic,
                                 self.depth_callback, 10)

        # Publishers
        self.image_pub  = self.create_publisher(Image,  '/aruco/detected_image', 10)
        self.marker_pub = self.create_publisher(Marker,'/aruco/cube_marker',     10)

        self.published_ids = set()
        self.get_logger().info('ArucoDetectNode initialized.')

    def camera_info_callback(self, msg: CameraInfo):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs   = np.array(msg.d)
            self.get_logger().info('Camera intrinsics set.')

    def depth_callback(self, msg: Image):
        try:
            # get raw depth, keep units (float32 meters or uint16 mm)
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.latest_depth = depth_image
        except CvBridgeError as e:
            self.get_logger().error(f"Depth conversion failed: {e}")

    def delete_markers(self, ids_to_delete):
        for mid in ids_to_delete:
            m = Marker()
            m.header.frame_id = self.base_frame
            m.header.stamp    = rclpy.time.Time().to_msg()
            m.ns      = 'aruco_cube'
            m.id      = int(mid)
            m.action  = Marker.DELETE
            self.marker_pub.publish(m)
        self.published_ids.clear()

    def image_callback(self, img_msg: Image):
        if self.camera_matrix is None:
            return

        # get color image
        cv_img = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')
        gray   = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)

        detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        corners, ids, _ = detector.detectMarkers(gray)

        if ids is None or len(ids) == 0:
            self.delete_markers(self.published_ids.copy())
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_img, 'bgr8'))
            return

        ids = ids.flatten()
        current_ids = set()
        cv2.aruco.drawDetectedMarkers(cv_img, corners, ids)

        for idx, marker_id in enumerate(ids):
            if marker_id in (0,1,2):
                length = self.cube_marker_length
                color_map = {0:(1,1,0),1:(1,0,0),2:(0,1,0)}
            elif marker_id in (4,5,6):
                length = self.box_marker_length
                color_map = {4:(1,1,0),5:(1,0,0),6:(0,1,0)}
            else:
                continue

            # pixel centroid
            pts = corners[idx][0]  # shape (4,2)
            u, v = int(pts[:,0].mean()), int(pts[:,1].mean())

            # estimate XY from PnP
            single = [corners[idx]]
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                single, length, self.camera_matrix, self.dist_coeffs)
            x_pnp, y_pnp, z_pnp = tvecs[0][0]

            # override Z from depth if available
            if self.latest_depth is not None:
                try:
                    z_depth = float(self.latest_depth[v, u])
                    z = z_depth / 1000.0
                except Exception:
                    z = z_pnp
            else:
                z = z_pnp

            # draw axes with PnP z for feedback
            cv2.drawFrameAxes(cv_img, self.camera_matrix, self.dist_coeffs,
                              rvecs[0][0], tvecs[0][0], length*0.5)

            # build PoseStamped in camera frame
            ps = PoseStamped()
            ps.header.stamp    = rclpy.time.Time().to_msg()
            ps.header.frame_id = self.camera_frame
            ps.pose.position.x = x_pnp
            ps.pose.position.y = y_pnp
            ps.pose.position.z = z

            # orientation from PnP
            R_mat,_ = cv2.Rodrigues(rvecs[0][0])
            qx,qy,qz,qw = SciRot.from_matrix(R_mat).as_quat()
            ps.pose.orientation.x = qx
            ps.pose.orientation.y = qy
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw

            # transform into base frame
            try:
                ps_base = self.tf_buffer.transform(
                    ps, self.base_frame,
                    timeout=Duration(seconds=1.0))
            except Exception as e:
                self.get_logger().error(f"TF error: {e}")
                continue

            # broadcast TF
            tf_msg = TransformStamped()
            tf_msg.header.stamp    = ps_base.header.stamp
            tf_msg.header.frame_id = self.base_frame
            tf_msg.child_frame_id  = f'aruco_{marker_id}'
            tf_msg.transform.translation.x = ps_base.pose.position.x
            tf_msg.transform.translation.y = ps_base.pose.position.y
            tf_msg.transform.translation.z = ps_base.pose.position.z
            tf_msg.transform.rotation      = ps_base.pose.orientation
            self.tf_broadcaster.sendTransform(tf_msg)

            # publish visualization marker
            m = Marker()
            m.header.frame_id = self.base_frame
            m.header.stamp    = ps_base.header.stamp
            m.ns      = 'aruco_cube'
            m.id      = int(marker_id)
            m.type    = Marker.CUBE
            m.action  = Marker.ADD
            m.pose    = ps_base.pose
            if marker_id in (0,1,2):
                m.scale.x = m.scale.y = m.scale.z = length
            else:
                m.scale.x = m.scale.y = 0.15
                m.scale.z = 0.01
            rgb = color_map[marker_id]
            m.color.r = float(rgb[0])
            m.color.g = float(rgb[1])
            m.color.b = float(rgb[2])
            m.color.a = 0.8
            self.marker_pub.publish(m)

            current_ids.add(marker_id)

        removed = self.published_ids - current_ids
        if removed:
            self.delete_markers(removed)
        self.published_ids = current_ids

        # publish annotated image
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_img, 'bgr8'))

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
