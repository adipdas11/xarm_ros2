#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker
import tf2_ros
import tf2_geometry_msgs  # registers PoseStamped support
from geometry_msgs.msg import PoseStamped, TransformStamped
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
        self.get_logger().info(f"ArucoDetectNode running in '{mode}' mode")

        # select camera topics based on mode
        if mode == 'sim':
            cam_info_topic = '/xarm5/D435_1/camera_info'
            color_topic    = '/xarm5/D435_1/color/image_raw'
            depth_topic    = '/xarm5/D435_1/depth/image_rect_raw'
        else:
            cam_info_topic = '/camera/camera/color/camera_info'
            color_topic    = '/camera/camera/color/image_raw'
            depth_topic    = '/camera/camera/depth/image_rect_raw'

        # QoS for sensor streams (best‐effort)
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        # TF2 setup
        self.tf_buffer      = tf2_ros.Buffer()
        self.tf_listener    = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # CV Bridge
        self.bridge = CvBridge()

        # camera intrinsics + latest depth image
        self.camera_matrix = None
        self.dist_coeffs   = None
        self.latest_depth  = None

        # visualization parameters
        self.cube_marker_length = 0.03
        self.box_marker_length  = 0.10
        self.camera_frame       = 'camera_color_optical_frame'
        self.base_frame         = 'link_base'

        # ArUco detector
        self.aruco_dict   = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()

        # subscriptions
        self.create_subscription(
            CameraInfo, cam_info_topic,
            self.camera_info_callback, qos_profile=sensor_qos
        )
        self.create_subscription(
            Image, color_topic,
            self.color_callback, qos_profile=sensor_qos
        )
        self.create_subscription(
            Image, depth_topic,
            self.depth_callback, qos_profile=sensor_qos
        )

        # publishers
        self.image_pub  = self.create_publisher(Image, '/aruco/detected_image', 10)
        self.marker_pub = self.create_publisher(Marker, '/aruco/cube_marker', 10)

        self.published_ids = set()
        self.get_logger().info('ArucoDetectNode initialized.')

    def camera_info_callback(self, msg: CameraInfo):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs   = np.array(msg.d)
            self.get_logger().info('Camera intrinsics set.')

    def depth_callback(self, msg: Image):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.latest_depth = depth
        except CvBridgeError as e:
            self.get_logger().error(f"Depth conversion failed: {e}")

    def color_callback(self, msg: Image):
        # wait until intrinsics are set
        if self.camera_matrix is None:
            return
        # skip empty frames
        if msg.width == 0 or msg.height == 0 or not msg.data:
            self.get_logger().warn('Empty color image—skipping')
            return
        # manual conversion from raw bytes to numpy BGR image
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint8)
            arr = arr.reshape((msg.height, msg.width, 3))
        except Exception as e:
            self.get_logger().error(f"Failed to reshape color data: {e}")
            return
        if msg.encoding == 'rgb8':
            cv_img = arr[..., ::-1].copy()
        elif msg.encoding == 'bgr8':
            cv_img = arr.copy()
        else:
            self.get_logger().error(f"Unsupported encoding: {msg.encoding}")
            return

        # ArUco detection
        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        corners, ids, _ = detector.detectMarkers(gray)
        if ids is None or len(ids) == 0:
            self._delete_markers(self.published_ids.copy())
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_img, 'bgr8'))
            return

        ids = ids.flatten()
        current_ids = set()
        cv2.aruco.drawDetectedMarkers(cv_img, corners, ids)

        for idx, marker_id in enumerate(ids):
            # select size/color
            if marker_id in (0,1,2):
                length = self.cube_marker_length
                color_map = {0:(1,1,0),1:(1,0,0),2:(0,1,0)}
            elif marker_id in (4,5,6):
                length = self.box_marker_length
                color_map = {4:(1,1,0),5:(1,0,0),6:(0,1,0)}
            else:
                continue

            # pixel centroid
            pts = corners[idx][0]
            u, v = int(pts[:,0].mean()), int(pts[:,1].mean())

            # pose from PnP
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                [corners[idx]], length, self.camera_matrix, self.dist_coeffs
            )
            x, y, z_pnp = tvecs[0][0]

            # depth override
            z = z_pnp
            if self.latest_depth is not None:
                raw = self.latest_depth[v, u]
                z_raw = float(raw)/1000.0 if self.latest_depth.dtype == np.uint16 else float(raw)
                if z_raw > 0 and np.isfinite(z_raw):
                    z = z_raw

            cv2.drawFrameAxes(
                cv_img, self.camera_matrix, self.dist_coeffs,
                rvecs[0][0], tvecs[0][0], length*0.5
            )

            # build PoseStamped
            ps = PoseStamped()
            ps.header.stamp    = rclpy.time.Time().to_msg()
            ps.header.frame_id = self.camera_frame
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = z
            R_mat, _ = cv2.Rodrigues(rvecs[0][0])
            qx, qy, qz, qw = SciRot.from_matrix(R_mat).as_quat()
            ps.pose.orientation.x = qx
            ps.pose.orientation.y = qy
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw

            # TF broadcast
            try:
                ps_base = self.tf_buffer.transform(
                    ps, self.base_frame, timeout=Duration(seconds=1.0)
                )
            except Exception as e:
                self.get_logger().error(f"TF error: {e}")
                continue

            tf_msg = TransformStamped()
            tf_msg.header = ps_base.header
            tf_msg.child_frame_id = f'aruco_{marker_id}'
            tf_msg.transform.translation.x = ps_base.pose.position.x
            tf_msg.transform.translation.y = ps_base.pose.position.y
            tf_msg.transform.translation.z = ps_base.pose.position.z
            tf_msg.transform.rotation      = ps_base.pose.orientation
            self.tf_broadcaster.sendTransform(tf_msg)

            # publish marker
            m = Marker()
            m.header.frame_id = self.base_frame
            m.header.stamp    = ps_base.header.stamp
            m.ns              = 'aruco_cube'
            m.id              = int(marker_id)
            m.type            = Marker.CUBE
            m.action          = Marker.ADD
            m.pose            = ps_base.pose
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

        # delete old
        removed = self.published_ids - current_ids
        if removed:
            self._delete_markers(removed)
        self.published_ids = current_ids

        # publish annotated image
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_img, 'bgr8'))

    def _delete_markers(self, ids_to_delete):
        for mid in ids_to_delete:
            m = Marker()
            m.header.frame_id = self.base_frame
            m.header.stamp    = rclpy.time.Time().to_msg()
            m.ns              = 'aruco_cube'
            m.id              = int(mid)
            m.action          = Marker.DELETE
            self.marker_pub.publish(m)
        self.published_ids.clear()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()