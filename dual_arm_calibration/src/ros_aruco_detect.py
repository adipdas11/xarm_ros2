#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS2 node: Detect a 4x4 ArUco marker with ID 0 (size ~100 mm by default)
- Subscribes to a color image and camera info
- Detects ArUco markers from a chosen 4x4 dictionary (default DICT_4X4_50)
- If ID 0 is found, draws its outline and (if intrinsics are known) the pose axes
- Publishes the annotated image to /aruco/detected_image for RViz visualization

Tested with rclpy + OpenCV (contrib) 4.x. Requires cv_bridge.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge

import tf2_ros
from rclpy.time import Time
from rclpy.duration import Duration

import cv2
import numpy as np


class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')

        # --- Parameters ---
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        # Common 4x4 dicts: DICT_4X4_50, DICT_4X4_100, DICT_4X4_250, DICT_4X4_1000
        self.declare_parameter('aruco_dictionary', 'DICT_4X4_50')
        # Marker length in meters (100 mm default)
        self.declare_parameter('marker_length_m', 0.10)
        self.declare_parameter('depth_topic', '/camera/camera/depth/image_rect_raw')
        self.declare_parameter('camera_frame', 'R_camera_color_optical_frame')
        self.declare_parameter('base_frame', 'R_link_base')

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        dict_name = self.get_parameter('aruco_dictionary').get_parameter_value().string_value
        self.marker_length_m = float(self.get_parameter('marker_length_m').value)
        depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value

        # --- ArUco setup ---
        try:
            dict_id = getattr(cv2.aruco, dict_name)
        except AttributeError:
            self.get_logger().warn(f"Unknown aruco_dictionary '{dict_name}', falling back to DICT_4X4_50")
            dict_id = cv2.aruco.DICT_4X4_50
        self.dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
        try:
            self.parameters = cv2.aruco.DetectorParameters_create()
        except AttributeError:
            # OpenCV >= 4.7 has different API; fallback handled later if needed
            self.parameters = cv2.aruco.DetectorParameters()

        # --- Camera model cache ---
        self.camera_matrix = None
        self.dist_coeffs = None

        # --- Depth cache ---
        self.depth_image = None
        self.depth_encoding = None
        self.depth_stamp = None

        # --- TF ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- ROS interfaces ---
        sensor_qos = QoSProfile(depth=10)
        sensor_qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        sensor_qos.history = QoSHistoryPolicy.KEEP_LAST

        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(Image, image_topic, self.image_cb, sensor_qos)
        self.depth_sub = self.create_subscription(Image, depth_topic, self.depth_cb, sensor_qos)
        self.camera_info_sub = self.create_subscription(CameraInfo, camera_info_topic, self.camera_info_cb, 10)

        self.image_pub = self.create_publisher(Image, '/aruco/detected_image', 10)
        self.pose_cam_pub = self.create_publisher(PoseStamped, '/aruco/pose_camera', 10)
        self.pose_base_pub = self.create_publisher(PoseStamped, '/aruco/pose_base', 10)
        self.marker_pub = self.create_publisher(Marker, '/aruco/marker', 10)

        self.get_logger().info(
            f"ArucoDetectorNode up. Subscribing to {image_topic} and {camera_info_topic}."
            f"Dictionary: {dict_name}, marker_length_m: {self.marker_length_m:.3f}."
            f"Annotated images -> /aruco/detected_image"
            f"Poses -> /aruco/pose_camera (frame: {self.camera_frame}), /aruco/pose_base (frame: {self.base_frame})"
            f"Marker -> /aruco/marker (frame: {self.base_frame})"
        )

    # --- Callbacks ---
    def camera_info_cb(self, msg: CameraInfo):
        # Build intrinsics from CameraInfo
        k = np.array(msg.k).reshape((3, 3))
        d = np.array(msg.d, dtype=np.float64).reshape(-1, 1) if msg.d else np.zeros((5, 1))
        self.camera_matrix = k
        self.dist_coeffs = d

    def depth_cb(self, msg: Image):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().warn(f"depth cv_bridge conversion failed: {e}")
            return
        if img is None or img.size == 0:
            return
        self.depth_image = img
        self.depth_encoding = (msg.encoding or '').lower()
        self.depth_stamp = msg.header.stamp

    def image_cb(self, msg: Image):
        # Convert using passthrough first so we can handle various encodings safely
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"cv_bridge conversion failed: {e}")
            return

        if cv_image is None or cv_image.size == 0:
            self.get_logger().warn(
                f"Empty image received: encoding={msg.encoding}, size=({msg.width}x{msg.height}), step={msg.step}"
            )
            return

        # Normalize to a BGR image for downstream OpenCV ops
        enc = msg.encoding.lower() if msg.encoding else ''
        try:
            if enc in ('bgr8',):
                frame = cv_image
            elif enc == 'rgb8':
                frame = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            elif enc in ('mono8', '8uc1'):
                frame = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
            elif enc in ('mono16', '16uc1', '32fc1'):
                # Depth-like image; scale to 8-bit for visualization only
                img = cv_image.astype(np.float32)
                maxv = float(np.max(img)) if np.max(img) > 0 else 1.0
                img = (img / maxv) * 255.0
                img8 = img.astype(np.uint8)
                frame = cv2.cvtColor(img8, cv2.COLOR_GRAY2BGR)
            else:
                # Fallback: try assuming it's already 3-channel
                if len(cv_image.shape) == 3 and cv_image.shape[2] == 3:
                    frame = cv_image.copy()
                else:
                    frame = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
        except Exception as e:
            self.get_logger().error(f"Image encoding handling failed ({enc}): {e}")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect markers (OpenCV 4.x compatibility)
        try:
            corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.parameters)
        except AttributeError:
            # Newer API (OpenCV 4.7+): use ArucoDetector class
            detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)
            corners, ids, rejected = detector.detectMarkers(gray)

        if ids is not None and len(ids) > 0:
            # Draw all detected markers lightly
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            # Focus on ID 0 only
            idx = np.where(ids.flatten() == 0)[0]
            if idx.size > 0:
                for i in idx:
                    c = corners[i]
                    # Thicker outline for ID 0
                    pts = c[0].astype(int)
                    for j in range(4):
                        p1 = tuple(pts[j])
                        p2 = tuple(pts[(j + 1) % 4])
                        cv2.line(frame, p1, p2, (0, 255, 0), 3)

                # Pose estimation for orientation (rvec) and fallback translation
                rvecs, tvecs = None, None
                if self.camera_matrix is not None and self.dist_coeffs is not None and self.marker_length_m > 0:
                    try:
                        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                            [corners[i] for i in idx], self.marker_length_m, self.camera_matrix, self.dist_coeffs
                        )
                        for rvec, tvec in zip(rvecs, tvecs):
                            cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, self.marker_length_m * 0.5)
                    except Exception as e:
                        self.get_logger().warn(f"Pose estimation failed: {e}")

                # Compute centroid pixel (u,v) of first ID 0 marker
                i0 = int(idx[0])
                c0 = corners[i0][0]
                u = float(np.mean(c0[:, 0]))
                v = float(np.mean(c0[:, 1]))

                # Depth-based Z (meters) from registered depth image
                Z = self.get_depth_at(u, v)
                if Z is None and tvecs is not None:
                    Z = float(tvecs[0][0][2])  # fallback if depth missing

                # Compute X,Y from pinhole using color intrinsics
                if self.camera_matrix is not None and Z is not None:
                    fx = float(self.camera_matrix[0, 0])
                    fy = float(self.camera_matrix[1, 1])
                    cx = float(self.camera_matrix[0, 2])
                    cy = float(self.camera_matrix[1, 2])
                    X = (u - cx) * Z / fx
                    Y = (v - cy) * Z / fy

                    # Orientation from rvecs if available, else identity
                    if rvecs is not None:
                        R_cam_marker, _ = cv2.Rodrigues(rvecs[0])
                    else:
                        R_cam_marker = np.eye(3)

                    q_cam = self.mat_to_quat(R_cam_marker)

                    # Publish/log pose in camera frame
                    pose_cam = PoseStamped()
                    pose_cam.header = msg.header
                    pose_cam.header.frame_id = self.camera_frame or msg.header.frame_id
                    pose_cam.pose.position.x = float(X)
                    pose_cam.pose.position.y = float(Y)
                    pose_cam.pose.position.z = float(Z)
                    pose_cam.pose.orientation.x = q_cam[0]
                    pose_cam.pose.orientation.y = q_cam[1]
                    pose_cam.pose.orientation.z = q_cam[2]
                    pose_cam.pose.orientation.w = q_cam[3]
                    self.pose_cam_pub.publish(pose_cam)

                    # Transform to base frame using TF
                    try:
                        tf = self.tf_buffer.lookup_transform(
                            self.base_frame,
                            pose_cam.header.frame_id,
                            Time(),  # latest
                            timeout=Duration(seconds=0.2)
                        )
                        p_base, q_base = self.apply_transform(tf, np.array([X, Y, Z]), q_cam)

                        pose_base = PoseStamped()
                        pose_base.header.stamp = msg.header.stamp
                        pose_base.header.frame_id = self.base_frame
                        pose_base.pose.position.x = float(p_base[0])
                        pose_base.pose.position.y = float(p_base[1])
                        pose_base.pose.position.z = float(p_base[2])
                        pose_base.pose.orientation.x = q_base[0]
                        pose_base.pose.orientation.y = q_base[1]
                        pose_base.pose.orientation.z = q_base[2]
                        pose_base.pose.orientation.w = q_base[3]
                        self.pose_base_pub.publish(pose_base)

                        # Publish RViz Marker at transformed pose
                        self.publish_marker(pose_base, marker_id=0)

                        self.get_logger().info(
                            f"ID 0 pose (base={self.base_frame}): p=[{p_base[0]:.3f}, {p_base[1]:.3f}, {p_base[2]:.3f}] m, "
                            f"q=[{q_base[0]:.3f}, {q_base[1]:.3f}, {q_base[2]:.3f}, {q_base[3]:.3f}]"
                        )
                    except Exception as e:
                        self.get_logger().warn(f"TF lookup/apply failed ({self.camera_frame} -> {self.base_frame}): {e}")
        else:
            # Optional: annotate no detection
            cv2.putText(frame, "ArUco ID 0 not found", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2, cv2.LINE_AA)
            # Remove marker from RViz if previously shown
            self.delete_marker(marker_id=0)

        # Publish annotated image continuously so RViz updates
        try:
            out_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            out_msg.header = msg.header  # keep timestamp/frame_id
            self.image_pub.publish(out_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish annotated image: {e}")

    # --- Helpers ---
    def get_depth_at(self, u: float, v: float, window: int = 5):
        """Return median depth (meters) around pixel (u,v) using a square window (odd size)."""
        if self.depth_image is None:
            return None
        h, w = self.depth_image.shape[:2]
        uc = int(round(u))
        vc = int(round(v))
        r = max(1, window // 2)
        u0, u1 = np.clip([uc - r, uc + r + 1], 0, w)
        v0, v1 = np.clip([vc - r, vc + r + 1], 0, h)
        patch = self.depth_image[v0:v1, u0:u1]
        if patch.size == 0:
            return None
        enc = (self.depth_encoding or '').lower()
        depth_m = None
        try:
            if enc in ('16uc1', 'mono16'):
                # millimeters -> meters
                patch = patch.astype(np.float32)
                patch_valid = patch[(patch > 0) & (patch < 100000)]
                if patch_valid.size == 0:
                    return None
                depth_m = float(np.median(patch_valid) / 1000.0)
            elif enc in ('32fc1',):
                patch = patch.astype(np.float32)
                patch_valid = patch[np.isfinite(patch) & (patch > 0)]
                if patch_valid.size == 0:
                    return None
                depth_m = float(np.median(patch_valid))
            else:
                # Unknown encoding: try best-effort
                patch = patch.astype(np.float32)
                patch_valid = patch[np.isfinite(patch) & (patch > 0)]
                if patch_valid.size == 0:
                    return None
                # Assume meters
                depth_m = float(np.median(patch_valid))
        except Exception:
            return None
        return depth_m

    @staticmethod
    def mat_to_quat(R: np.ndarray):
        """Convert 3x3 rotation matrix to quaternion (x,y,z,w)."""
        m = R
        t = np.trace(m)
        if t > 0:
            s = np.sqrt(t + 1.0) * 2
            qw = 0.25 * s
            qx = (m[2, 1] - m[1, 2]) / s
            qy = (m[0, 2] - m[2, 0]) / s
            qz = (m[1, 0] - m[0, 1]) / s
        else:
            if (m[0, 0] > m[1, 1]) and (m[0, 0] > m[2, 2]):
                s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
                qx = 0.25 * s
                qy = (m[0, 1] + m[1, 0]) / s
                qz = (m[0, 2] + m[2, 0]) / s
                qw = (m[2, 1] - m[1, 2]) / s
            elif m[1, 1] > m[2, 2]:
                s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
                qx = (m[0, 1] + m[1, 0]) / s
                qy = 0.25 * s
                qz = (m[1, 2] + m[2, 1]) / s
                qw = (m[0, 2] - m[2, 0]) / s
            else:
                s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
                qx = (m[0, 2] + m[2, 0]) / s
                qy = (m[1, 2] + m[2, 1]) / s
                qz = 0.25 * s
                qw = (m[1, 0] - m[0, 1]) / s
        return np.array([float(qx), float(qy), float(qz), float(qw)])

    @staticmethod
    def quat_to_mat(q):
        x, y, z, w = q
        xx, yy, zz = x*x, y*y, z*z
        xy, xz, yz = x*y, x*z, y*z
        wx, wy, wz = w*x, w*y, w*z
        R = np.array([
            [1 - 2*(yy + zz), 2*(xy - wz),     2*(xz + wy)],
            [2*(xy + wz),     1 - 2*(xx + zz), 2*(yz - wx)],
            [2*(xz - wy),     2*(yz + wx),     1 - 2*(xx + yy)]
        ], dtype=np.float64)
        return R

    def apply_transform(self, tf: TransformStamped, p_cam: np.ndarray, q_cam: np.ndarray):
        # tf is base <- camera
        t = tf.transform.translation
        q = tf.transform.rotation
        q_base_cam = np.array([q.x, q.y, q.z, q.w])
        R_base_cam = self.quat_to_mat(q_base_cam)
        p_base = R_base_cam @ p_cam + np.array([t.x, t.y, t.z])

        # Orientation composition: q_base_marker = q_base_cam * q_cam
        qb = q_base_cam
        qc = q_cam
        q_base = np.array([
            qb[3]*qc[0] + qb[0]*qc[3] + qb[1]*qc[2] - qb[2]*qc[1],
            qb[3]*qc[1] - qb[0]*qc[2] + qb[1]*qc[3] + qb[2]*qc[0],
            qb[3]*qc[2] + qb[0]*qc[1] - qb[1]*qc[0] + qb[2]*qc[3],
            qb[3]*qc[3] - qb[0]*qc[0] - qb[1]*qc[1] - qb[2]*qc[2]
        ])
        return p_base, q_base

    def publish_marker(self, pose: PoseStamped, marker_id: int = 0):
        m = Marker()
        m.header.stamp = pose.header.stamp
        m.header.frame_id = self.base_frame
        m.ns = 'aruco'
        m.id = int(marker_id)
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose = pose.pose
        # Use marker physical size for x,y; make it a thin slab for z
        m.scale.x = float(self.marker_length_m)
        m.scale.y = float(self.marker_length_m)
        m.scale.z = float(max(0.002, self.marker_length_m * 0.05))
        m.color.r = 0.0
        m.color.g = 1.0
        m.color.b = 0.0
        m.color.a = 0.8
        m.lifetime = Duration(seconds=0.3).to_msg()  # auto-fade if updates stop
        self.marker_pub.publish(m)

    def delete_marker(self, marker_id: int = 0):
        m = Marker()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.base_frame
        m.ns = 'aruco'
        m.id = int(marker_id)
        m.action = Marker.DELETE
        self.marker_pub.publish(m)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
