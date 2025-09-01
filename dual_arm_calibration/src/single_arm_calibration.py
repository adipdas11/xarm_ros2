#!/usr/bin/env python3
# single_arm_calibration.py  ✨
# 🤖 xArm: HOME ➜ start pose
# 📷 RGB + Depth + CameraInfo (Best-Effort QoS) with /compressed fallback
# 🧩 ArUco 4x4 ID=0 (size=0.10 m)
# 🖼 Annotated image + cuboid Marker in RViz
# 🔁 TF: R_camera_color_optical_frame ➜ R_link_base (future-extrapolation safe)
# 🛡 Robust cv_bridge conversions
# 🕒 Action: detect ➜ wait 5s ➜ go to flipped-Z ArUco pose (+45 mm) ➜ wait 10s ➜ return to start pose

import math
import numpy as np
import rclpy
from rclpy.node import Node

from xarm.wrapper import XArmAPI

from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from visualization_msgs.msg import Marker

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.duration import Duration
from cv_bridge import CvBridge
import cv2
from message_filters import Subscriber, ApproximateTimeSynchronizer

# TF2
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs  # registers geometry types with TF2

# Quaternion helpers
try:
    from tf_transformations import quaternion_multiply, quaternion_from_euler
except Exception:
    def quaternion_from_euler(roll, pitch, yaw):
        cr = math.cos(roll / 2.0); sr = math.sin(roll / 2.0)
        cp = math.cos(pitch / 2.0); sp = math.sin(pitch / 2.0)
        cy = math.cos(yaw / 2.0); sy = math.sin(yaw / 2.0)
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return (qx, qy, qz, qw)

    def quaternion_multiply(q1, q2):
        x1, y1, z1, w1 = q1; x2, y2, z2, w2 = q2
        x = w1*x2 + x1*w2 + y1*z2 - z1*y2
        y = w1*y2 - x1*z2 + y1*w2 + z1*x2
        z = w1*z2 + x1*y2 - y1*x2 + z1*w2
        w = w1*w2 - x1*x2 - y1*y2 - z1*z2
        return (x, y, z, w)


def quat_to_rpy(qx, qy, qz, qw):
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (qw * qy - qz * qx)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


class XArmGoToPoseNode(Node):
    def __init__(self):
        super().__init__('xarm_go_to_pose')

        # ----------------- xArm params -----------------
        self.declare_parameter('robot_port', '192.168.1.239')
        self.declare_parameter('tcp_speed', 100.0)   # mm/s
        self.declare_parameter('tcp_acc', 2000.0)    # mm/s^2
        self.declare_parameter('wait', True)
        self.declare_parameter('go_home_first', True)

        # ----------------- Camera params ----------------
        self.declare_parameter('color_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/camera/depth/image_rect_raw')
        self.declare_parameter('caminfo_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('camera_optical_frame', 'R_camera_color_optical_frame')  # 🔁 new

        # ----------------- ArUco params ----------------
        self.declare_parameter('aruco_target_id', 0)                 # 🎯 ID
        self.declare_parameter('aruco_marker_length_m', 0.10)        # 📏 100 mm
        self.declare_parameter('aruco_dictionary', 'DICT_4X4_50')
        self.declare_parameter('publish_annotated_topic', '/aruco/detected/image')
        self.declare_parameter('publish_pose_topic', '/aruco/detected/pose')

        # ----------------- TF params ----------------
        self.declare_parameter('target_base_frame', 'R_link_base')   # 🔁 new default
        self.declare_parameter('tcp_frame', 'R_link_tcp')            # 🔁 for reference/logs

        # Resolve params
        port = self.get_parameter('robot_port').value
        self.tcp_speed = float(self.get_parameter('tcp_speed').value)
        self.tcp_acc = float(self.get_parameter('tcp_acc').value)
        self.wait_motion = bool(self.get_parameter('wait').value)
        go_home_first = bool(self.get_parameter('go_home_first').value)

        self.color_topic = self.get_parameter('color_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.caminfo_topic = self.get_parameter('caminfo_topic').value

        self.camera_optical_frame = self.get_parameter('camera_optical_frame').value
        self.target_base_frame = self.get_parameter('target_base_frame').value
        self.tcp_frame = self.get_parameter('tcp_frame').value

        # Topic presence heads-up
        topic_names = [t[0] for t in self.get_topic_names_and_types()]
        for t in [self.color_topic, self.depth_topic, self.caminfo_topic]:
            if t not in topic_names:
                self.get_logger().warn(f'📋 Configured topic "{t}" not found in ROS graph!')

        # ---------- Connect and move the arm ----------
        self.get_logger().info(f'🤖 Connecting to xArm at {port}…')
        try:
            self.arm = XArmAPI(port=port, is_radian=False, protocol_type=3)
        except TypeError:
            self.arm = XArmAPI(port=port, is_radian=False, protocol=3)
        self.get_logger().info('✅ Connected.')

        self._prepare_controller()

        if go_home_first:
            self.get_logger().info('🏠 Moving to HOME pose…')
            code = self.arm.move_gohome(is_radian=False, wait=True)
            if code != 0:
                self.get_logger().warn(f'HOME returned code {code} (err={self.arm.error_code}, warn={self.arm.warn_code})')

        # ➜ Start pose
        self.start_xyz_m = (0.4, -0.3, 0.2)
        qx, qy, qz, qw = 0.90, -0.42, -0.01, -0.002
        roll_r, pitch_r, yaw_r = quat_to_rpy(qx, qy, qz, qw)
        self.start_rpy_deg = tuple(map(math.degrees, (roll_r, pitch_r, yaw_r)))

        x, y, z = self.start_xyz_m
        roll, pitch, yaw = self.start_rpy_deg
        self.get_logger().info(
            f'➡️ Start pose XYZ(mm)=({x*1000:.1f},{y*1000:.1f},{z*1000:.1f}) '
            f'RPY(deg)=({roll:.2f},{pitch:.2f},{yaw:.2f})'
        )
        code = self.arm.set_position(
            x=x*1000.0, y=y*1000.0, z=z*1000.0,
            roll=roll, pitch=pitch, yaw=yaw,
            speed=self.tcp_speed, mvacc=self.tcp_acc, is_radian=False, wait=self.wait_motion
        )
        if code != 0:
            self.get_logger().error(f'❌ set_position code={code} (err={self.arm.error_code}, warn={self.arm.warn_code})')
        else:
            self.get_logger().info('✅ Start pose reached.')

        # ----------------- ArUco setup -----------------
        self.bridge = CvBridge()
        self.K = None
        self.D = None
        self.frame_id = None  # will use camera_optical_frame

        dict_name = self.get_parameter('aruco_dictionary').value
        aruco_dict_id = getattr(cv2.aruco, dict_name, cv2.aruco.DICT_4X4_50)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_id)

        self._use_new_detector = hasattr(cv2.aruco, 'ArucoDetector')
        if self._use_new_detector:
            params = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, params)
        else:
            try:
                params = cv2.aruco.DetectorParameters_create()
            except AttributeError:
                params = cv2.aruco.DetectorParameters()
            self.detector = None
        self.aruco_params = params

        self.target_id = int(self.get_parameter('aruco_target_id').value)
        self.marker_len = float(self.get_parameter('aruco_marker_length_m').value)

        # Publishers
        self.image_pub = self.create_publisher(Image, self.get_parameter('publish_annotated_topic').value, 10)
        self.pose_pub = self.create_publisher(PoseStamped, self.get_parameter('publish_pose_topic').value, 10)
        self.pose_base_pub = self.create_publisher(PoseStamped, '/aruco/detected/pose_base', 10)
        self.marker_pub = self.create_publisher(Marker, '/aruco/marker_cube', 10)

        # ----------------- TF buffer/listener -----------------
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        # ----------------- Subscribers (Best-Effort QoS) -----------------
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)

        self.color_sub = Subscriber(self, Image, self.color_topic, qos_profile=sensor_qos)
        self.depth_sub = Subscriber(self, Image, self.depth_topic, qos_profile=sensor_qos)
        self.info_sub  = Subscriber(self, CameraInfo, self.caminfo_topic, qos_profile=sensor_qos)

        self.ts = ApproximateTimeSynchronizer([self.color_sub, self.depth_sub, self.info_sub],
                                              queue_size=10, slop=0.15)
        self.ts.registerCallback(self._on_rgbd)

        # Optional: compressed fallback subscriber
        self.compressed_color_topic = self.color_topic + '/compressed'
        self.last_compressed = None
        if self.compressed_color_topic in topic_names:
            self.create_subscription(CompressedImage, self.compressed_color_topic, self._on_compressed_color, sensor_qos)
            self.get_logger().info(f'🧩 Compressed fallback enabled: {self.compressed_color_topic}')
        else:
            self.get_logger().info('🧩 Compressed fallback not available (topic not found).')

        # --- Motion control state for “detect → move → return” ---
        self.move_scheduled = False
        self.moved_to_marker = False
        self.return_scheduled = False
        self.pending_pose_base = None

        self.get_logger().info('🟢 ArUco detection initialized. Waiting for frames…')

    # ----------------- Compressed fallback handler -----------------
    def _on_compressed_color(self, msg: CompressedImage):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if img is not None:
                self.last_compressed = img
        except Exception:
            pass

    # ----------------- Safe img conversion helpers -----------------
    def _convert_color(self, msg: Image):
        if msg.width == 0 or msg.height == 0:
            raise RuntimeError('color image empty (zero width/height)')
        if not msg.data:
            raise RuntimeError('color image empty (no data)')
        expected_min = int(msg.height) * int(msg.step or 0)
        if expected_min <= 0 or len(msg.data) < expected_min:
            raise RuntimeError(f'color image malformed: len(data)={len(msg.data)} < h*step={expected_min}')
        enc = (msg.encoding or '').lower()
        try:
            if enc == 'bgr8':
                return self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            elif enc in ('rgb8', 'rgba8', 'bgra8', 'bayer_rggb8', 'bayer_bggr8', 'bayer_gbrg8', 'bayer_grbg8'):
                return self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            else:
                return self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            raise RuntimeError(f'cv_bridge color convert failed: {e}')

    def _convert_depth(self, msg: Image):
        if msg.width == 0 or msg.height == 0:
            raise RuntimeError('depth image empty (zero width/height)')
        if not msg.data:
            raise RuntimeError('depth image empty (no data)')
        expected_min = int(msg.height) * int(msg.step or 0)
        if expected_min <= 0 or len(msg.data) < expected_min:
            raise RuntimeError(f'depth image malformed: len(data)={len(msg.data)} < h*step={expected_min}')
        try:
            return self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            raise RuntimeError(f'cv_bridge depth convert failed: {e}')

    # ----------------- Cuboid Marker -----------------
    def _publish_cuboid_marker(self, pose_base: PoseStamped, ns='aruco_cube', color=(0.0, 1.0, 0.2, 0.9)):
        m = Marker()
        m.header = pose_base.header  # will be R_link_base
        m.ns = ns
        m.id = 0
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose = pose_base.pose
        m.scale.x = self.marker_len
        m.scale.y = self.marker_len
        m.scale.z = 0.02
        m.color.r, m.color.g, m.color.b, m.color.a = color
        m.lifetime = rclpy.duration.Duration(seconds=0.5).to_msg()
        self.marker_pub.publish(m)

    # ----------------- Camera callback -----------------
    def _on_rgbd(self, color_msg: Image, depth_msg: Image, info_msg: CameraInfo):
        # Cache intrinsics
        if self.K is None:
            k_array = np.array(info_msg.k, dtype=np.float64)
            if k_array.size != 9 or info_msg.width == 0 or info_msg.height == 0:
                self.get_logger().debug('⏳ Waiting for valid CameraInfo…')
                return
            self.K = k_array.reshape(3, 3)
            self.D = np.array(info_msg.d, dtype=np.float64).reshape(-1, 1) if len(info_msg.d) > 0 else np.zeros((5, 1))
            # Force the camera optical frame you requested:
            self.frame_id = self.camera_optical_frame
            self.get_logger().info(f'🔧 Intrinsics loaded.\nK=\n{self.K}\nD={self.D.ravel()} frame_id={self.frame_id}')

        # Convert images (with compressed fallback for color)
        try:
            cv_color = self._convert_color(color_msg)
        except RuntimeError as e:
            if self.last_compressed is not None:
                cv_color = self.last_compressed.copy()
            else:
                self.get_logger().debug(f'⏳ Waiting for non-empty color image… ({e})')
                return

        try:
            _ = self._convert_depth(depth_msg)
        except RuntimeError:
            pass

        # Detect markers
        if self._use_new_detector:
            corners, ids, _ = self.detector.detectMarkers(cv_color)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(cv_color, self.aruco_dict, parameters=self.aruco_params)

        annotated = cv_color.copy()
        if ids is None or len(ids) == 0:
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8'))
            except Exception:
                pass
            return

        ids = ids.flatten()
        cv2.aruco.drawDetectedMarkers(annotated, corners, ids)

        # Pose estimation
        try:
            rvecs, tvecs, _obj = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_len, self.K, self.D)
        except Exception as e:
            self.get_logger().warn(f'estimatePoseSingleMarkers failed: {e}')
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8'))
            except Exception:
                pass
            return

        for i, marker_id in enumerate(ids):
            rvec = rvecs[i].reshape(3, 1)
            tvec = tvecs[i].reshape(3, 1)  # meters

            try:
                cv2.aruco.drawAxis(annotated, self.K, self.D, rvec, tvec, 0.05)
            except Exception:
                pass

            if marker_id == self.target_id:
                pose_cam = PoseStamped()
                # Use your requested camera frame explicitly:
                pose_cam.header = Header(stamp=color_msg.header.stamp, frame_id=self.camera_optical_frame)

                # rvec ➜ quaternion
                R, _ = cv2.Rodrigues(rvec)
                qw = math.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2.0
                qx = (R[2, 1] - R[1, 2]) / (4.0 * qw + 1e-9)
                qy = (R[0, 2] - R[2, 0]) / (4.0 * qw + 1e-9)
                qz = (R[1, 0] - R[0, 1]) / (4.0 * qw + 1e-9)
                norm = math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz) or 1.0
                qw, qx, qy, qz = qw/norm, qx/norm, qy/norm, qz/norm

                tx, ty, tz = [float(v) for v in tvec.ravel()]
                pose_cam.pose.position.x = tx
                pose_cam.pose.position.y = ty
                pose_cam.pose.position.z = tz
                pose_cam.pose.orientation.x = qx
                pose_cam.pose.orientation.y = qy
                pose_cam.pose.orientation.z = qz
                pose_cam.pose.orientation.w = qw

                # Publish camera-frame pose
                self.pose_pub.publish(pose_cam)

                # Transform to base (R_link_base)
                try:
                    pose_base: PoseStamped = self.tf_buffer.transform(
                        pose_cam, self.target_base_frame, timeout=Duration(seconds=0.2)
                    )
                except Exception as e:
                    msg = str(e).lower()
                    if 'extrapolation' in msg:
                        pose_latest = PoseStamped()
                        pose_latest.header = Header(stamp=pose_cam.header.stamp, frame_id=pose_cam.header.frame_id)
                        pose_latest.pose = pose_cam.pose
                        pose_latest.header.stamp.sec = 0
                        pose_latest.header.stamp.nanosec = 0
                        pose_base = self.tf_buffer.transform(
                            pose_latest, self.target_base_frame, timeout=Duration(seconds=0.5)
                        )
                    else:
                        self.get_logger().warn(f'⛔ TF to {self.target_base_frame} failed: {e}')
                        continue

                # Ensure header shows base frame
                pose_base.header.frame_id = self.target_base_frame
                self.pose_base_pub.publish(pose_base)

                # Publish cuboid marker in R_link_base
                self._publish_cuboid_marker(pose_base)

                # Schedule motion
                if not self.move_scheduled and not self.moved_to_marker:
                    self.pending_pose_base = pose_base
                    self.move_scheduled = True
                    self.get_logger().info('⏱ Detected target. Will move to it in 5 seconds…')
                    self.create_timer(5.0, self._go_to_marker_pose_once)

        # Publish annotated image
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8'))
        except Exception:
            pass

    # ----------------- Motion schedulers -----------------
    def _go_to_marker_pose_once(self):
        if self.moved_to_marker:
            return
        self.moved_to_marker = True

        if self.pending_pose_base is None:
            self.get_logger().warn('❔ No pending pose to move to.')
            return

        x_mm, y_mm, z_mm, roll_deg, pitch_deg, yaw_deg = self._pose_to_mm_deg(self.pending_pose_base)

        self.get_logger().info(
            f'🤖 Moving to ArUco pose (flipped Z, +45 mm): '
            f'XYZ(mm)=({x_mm:.1f},{y_mm:.1f},{z_mm:.1f}) '
            f'RPY(deg)=({roll_deg:.2f},{pitch_deg:.2f},{yaw_deg:.2f}) in {self.target_base_frame}'
        )
        code = self.arm.set_position(
            x=x_mm, y=y_mm, z=z_mm,
            roll=roll_deg, pitch=pitch_deg, yaw=yaw_deg,
            speed=self.tcp_speed, mvacc=self.tcp_acc, is_radian=False, wait=True
        )
        if code != 0:
            self.get_logger().error(f'❌ Move to ArUco pose failed (code={code}, err={self.arm.error_code}, warn={self.arm.warn_code})')
        else:
            self.get_logger().info('✅ Reached ArUco pose.')

        if not self.return_scheduled:
            self.return_scheduled = True
            self.get_logger().info('⏱ Will return to start pose in 10 seconds…')
            self.create_timer(10.0, self._return_to_start_once)

    def _return_to_start_once(self):
        x, y, z = self.start_xyz_m
        roll, pitch, yaw = self.start_rpy_deg
        self.get_logger().info('↩️ Returning to start pose…')
        code = self.arm.set_position(
            x=x*1000.0, y=y*1000.0, z=z*1000.0,
            roll=roll, pitch=pitch, yaw=yaw,
            speed=self.tcp_speed, mvacc=self.tcp_acc, is_radian=False, wait=True
        )
        if code != 0:
            self.get_logger().error(f'❌ Return failed (code={code}, err={self.arm.error_code}, warn={self.arm.warn_code})')
        else:
            self.get_logger().info('✅ Back at start pose.')

    # ----------------- Pose ➜ robot command (mm & deg) with Z flip (+45 mm) -----------------
    def _pose_to_mm_deg(self, pose_stamped: PoseStamped):
        p = pose_stamped.pose.position
        q = pose_stamped.pose.orientation

        # Flip the end-effector Z axis relative to marker Z:
        flip_q = quaternion_from_euler(math.pi, 0.0, 0.0)  # 180° about X
        new_qx, new_qy, new_qz, new_qw = quaternion_multiply((q.x, q.y, q.z, q.w), flip_q)
        roll, pitch, yaw = quat_to_rpy(new_qx, new_qy, new_qz, new_qw)

        # Stop above by +45 mm in base Z (as in your working code)
        z_offset_mm = 45.0
        return (
            float(p.x) * 1000.0,
            float(p.y) * 1000.0,
            float(p.z) * 1000.0 + z_offset_mm,
            math.degrees(roll),
            math.degrees(pitch),
            math.degrees(yaw),
        )

    # ----------------- xArm helpers -----------------
    def _prepare_controller(self):
        try:
            if self.arm.has_error:
                self.get_logger().warn('🧹 has_error → clean_error()')
                self.arm.clean_error()
            if self.arm.has_warn:
                self.get_logger().warn('🧹 has_warn → clean_warn()')
                self.arm.clean_warn()
        except Exception as e:
            self.get_logger().warn(f'While clearing errors/warns: {e}')

        for name, fn in [
            ('motion_enable', lambda: self.arm.motion_enable(True)),
            ('set_mode(0)',   lambda: self.arm.set_mode(0)),
            ('set_state(0)',  lambda: self.arm.set_state(0)),
        ]:
            try:
                code = fn()
                code = code[0] if isinstance(code, tuple) else code
                if code != 0:
                    self.get_logger().warn(f'{name} returned code {code}')
            except Exception as e:
                self.get_logger().warn(f'{name} raised: {e}')


def main():
    rclpy.init()
    node = XArmGoToPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
