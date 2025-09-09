#!/usr/bin/env python3
# dual_arm_calibration_fsm.py  ✨
# Two xArms (RIGHT & LEFT) with a single finite-state machine to avoid timer loops.
#
# Sequence:
#  1) BOTH ➜ HOME
#  2) RIGHT ➜ start
#  3) Detect ArUco (only in this phase), TF ➜ {R_link_base, L_link_base}, store both (flip EE Z later)
#  4) Wait 5s
#  5) RIGHT ➜ target(+R_offset)  ➜ wait 5s ➜ RIGHT ➜ HOME
#  6) LEFT  ➜ start             ➜ wait 5s ➜ LEFT  ➜ target(+L_offset) ➜ wait 5s ➜ LEFT ➜ HOME
#  7) DONE 🎉
#
# Frames (RIGHT camera):
#   R_camera_color_optical_frame  →  R_link_base, L_link_base
#
# Logs + emojis for each step; robust TF; cv_bridge-safe; /compressed fallback.

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
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
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


class DualArmCalibrationFSM(Node):
    def __init__(self):
        super().__init__('dual_arm_calibration')

        # ----------------- Parameters -----------------
        # RIGHT arm
        self.declare_parameter('R.ip', '192.168.1.239')
        self.declare_parameter('R.base_frame', 'R_link_base')
        self.declare_parameter('R.tcp_frame',  'R_link_tcp')
        self.declare_parameter('R.camera_optical_frame', 'R_camera_color_optical_frame')
        self.declare_parameter('R.tcp_speed', 100.0)     # mm/s
        self.declare_parameter('R.tcp_acc',   2000.0)    # mm/s^2
        self.declare_parameter('R.offset_mm', 40.0)      # stop 5 cm above
        # 👉 Software bias (XYZ in mm, RPY in deg)
        self.declare_parameter('R.bias_xyz_mm', [0.6, 0.0, 0.0])
        self.declare_parameter('R.bias_rpy_deg', [0.0, 0.0, 0.0])

        # LEFT arm
        self.declare_parameter('L.ip', '192.168.1.195')
        self.declare_parameter('L.base_frame', 'L_link_base')
        self.declare_parameter('L.tcp_frame',  'L_link_tcp')
        self.declare_parameter('L.tcp_speed', 100.0)
        self.declare_parameter('L.tcp_acc',   2000.0)
        self.declare_parameter('L.offset_mm', 60.0)
        # 👉 Software bias (XYZ in mm, RPY in deg)
        self.declare_parameter('L.bias_xyz_mm', [0.0, 0.0, 0.0])
        self.declare_parameter('L.bias_rpy_deg', [0.0, 0.0, 0.0])

        # Camera topics (RIGHT camera)
        self.declare_parameter('color_topic',  '/camera/camera/color/image_raw')
        self.declare_parameter('depth_topic',  '/camera/camera/depth/image_rect_raw')
        self.declare_parameter('caminfo_topic','/camera/camera/color/camera_info')

        # ArUco
        self.declare_parameter('aruco_target_id', 0)
        self.declare_parameter('aruco_marker_length_m', 0.10)
        self.declare_parameter('aruco_dictionary', 'DICT_4X4_50')
        self.declare_parameter('publish_annotated_topic', '/aruco/detected/image')

        # Resolve params
        self.R_ip  = self.get_parameter('R.ip').value
        self.R_base = self.get_parameter('R.base_frame').value
        self.R_tcp  = self.get_parameter('R.tcp_frame').value
        self.R_cam  = self.get_parameter('R.camera_optical_frame').value
        self.R_speed = float(self.get_parameter('R.tcp_speed').value)
        self.R_acc   = float(self.get_parameter('R.tcp_acc').value)
        self.R_offset = float(self.get_parameter('R.offset_mm').value)

        self.L_ip  = self.get_parameter('L.ip').value
        self.L_base = self.get_parameter('L.base_frame').value
        self.L_tcp  = self.get_parameter('L.tcp_frame').value
        self.L_speed = float(self.get_parameter('L.tcp_speed').value)
        self.L_acc   = float(self.get_parameter('L.tcp_acc').value)
        self.L_offset = float(self.get_parameter('L.offset_mm').value)

        self.color_topic  = self.get_parameter('color_topic').value
        self.depth_topic  = self.get_parameter('depth_topic').value
        self.caminfo_topic= self.get_parameter('caminfo_topic').value

        # ----------------- Connect both arms -----------------
        self.get_logger().info(f'🤖 Connecting RIGHT xArm @ {self.R_ip} …')
        try:
            self.R_arm = XArmAPI(port=self.R_ip, is_radian=False, protocol_type=3)
        except TypeError:
            self.R_arm = XArmAPI(port=self.R_ip, is_radian=False, protocol=3)
        self.get_logger().info('✅ RIGHT connected.')

        self.get_logger().info(f'🤖 Connecting LEFT  xArm @ {self.L_ip} …')
        try:
            self.L_arm = XArmAPI(port=self.L_ip, is_radian=False, protocol_type=3)
        except TypeError:
            self.L_arm = XArmAPI(port=self.L_ip, is_radian=False, protocol=3)
        self.get_logger().info('✅ LEFT connected.')

        self._prepare_controller(self.R_arm, 'R')
        self._prepare_controller(self.L_arm, 'L')

        # ----------------- Start poses -----------------
        # RIGHT start
        self.R_start_xyz_m = (0.4, -0.3, 0.2)
        R_qx, R_qy, R_qz, R_qw = 0.90, -0.42, -0.01, -0.002
        R_roll, R_pitch, R_yaw = map(math.degrees, quat_to_rpy(R_qx, R_qy, R_qz, R_qw))
        self.R_start_rpy_deg = (R_roll, R_pitch, R_yaw)

        # LEFT start (provided)
        self.L_start_xyz_m = (0.4, 0.3, 0.4)
        L_qx, L_qy, L_qz, L_qw = 0.90, 0.42, 0.01, 0.002
        L_roll, L_pitch, L_yaw = map(math.degrees, quat_to_rpy(L_qx, L_qy, L_qz, L_qw))
        self.L_start_rpy_deg = (L_roll, L_pitch, L_yaw)

        # ----------------- ArUco + camera setup -----------------
        self.bridge = CvBridge()
        self.K = None
        self.D = None
        self.cam_frame = None  # enforced to R_camera_color_optical_frame

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

        self.target_id  = int(self.get_parameter('aruco_target_id').value)
        self.marker_len = float(self.get_parameter('aruco_marker_length_m').value)

        # Publishers
        self.image_pub = self.create_publisher(Image, self.get_parameter('publish_annotated_topic').value, 10)
        self.marker_pub_R = self.create_publisher(Marker, '/aruco/marker_cube_R', 10)
        self.marker_pub_L = self.create_publisher(Marker, '/aruco/marker_cube_L', 10)
        self.pose_R_pub   = self.create_publisher(PoseStamped, '/aruco/pose_R_base', 10)
        self.pose_L_pub   = self.create_publisher(PoseStamped, '/aruco/pose_L_base', 10)

        # TF
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        # Subscriptions
        sensor_qos = QoSProfile(depth=10,
                                reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST)

        self.color_sub = Subscriber(self, Image, self.color_topic, qos_profile=sensor_qos)
        self.depth_sub = Subscriber(self, Image, self.depth_topic, qos_profile=sensor_qos)
        self.info_sub  = Subscriber(self, CameraInfo, self.caminfo_topic, qos_profile=sensor_qos)

        self.ts = ApproximateTimeSynchronizer([self.color_sub, self.depth_sub, self.info_sub],
                                              queue_size=10, slop=0.15)
        self.ts.registerCallback(self._on_rgbd)

        # Compressed fallback
        self.compressed_color_topic = self.color_topic + '/compressed'
        self.last_compressed = None
        if any(t[0] == self.compressed_color_topic for t in self.get_topic_names_and_types()):
            self.create_subscription(CompressedImage, self.compressed_color_topic,
                                     self._on_compressed_color, sensor_qos)
            self.get_logger().info(f'🧩 Compressed fallback enabled: {self.compressed_color_topic}')

        # ================= FSM =================
        self.State = type('State', (), {
            'INIT': 0,
            'RIGHT_TO_START': 1,
            'DETECT_WAIT': 2,
            'RIGHT_MOVE': 3,
            'RIGHT_HOLD': 4,
            'RIGHT_HOME': 5,
            'LEFT_TO_START': 6,
            'LEFT_HOLD_START': 7,
            'LEFT_MOVE': 8,
            'LEFT_HOLD': 9,
            'LEFT_HOME': 10,
            'DONE': 11
        })
        self.state = self.State.INIT
        self.active_timers = []

        # Stored target poses (in respective bases)
        self.pose_R_base = None
        self.pose_L_base = None

        # Kick off sequence
        self._transition_to(self.State.INIT)

    # ------------- FSM utilities -------------
    def _cancel_timers(self):
        for t in self.active_timers:
            try:
                t.cancel()
            except Exception:
                pass
        self.active_timers = []

    def _set_timer(self, delay, cb):
        t = self.create_timer(delay, cb)
        self.active_timers.append(t)
        return t

    def _transition_to(self, new_state):
        self._cancel_timers()
        self.state = new_state

        if new_state == self.State.INIT:
            self.get_logger().info('🏠 Sending BOTH arms to HOME …')
            self._safe_home(self.R_arm, 'R')
            self._safe_home(self.L_arm, 'L')
            self._transition_to(self.State.RIGHT_TO_START)

        elif new_state == self.State.RIGHT_TO_START:
            self._right_to_start()
            self.get_logger().info('👀 Entering detection phase…')
            self._transition_to(self.State.DETECT_WAIT)

        elif new_state == self.State.DETECT_WAIT:
            self.get_logger().info('🟢 Waiting for ArUco detection (will move in 5s after detection)…')

        elif new_state == self.State.RIGHT_MOVE:
            self.get_logger().info('➡️ RIGHT moving to ArUco pose in R base (with offset)…')
            self._move_to_target(self.R_arm, 'R', self.pose_R_base, self.R_speed, self.R_acc, self.R_offset)
            self.get_logger().info('⏱ RIGHT waiting 5s at target…')
            self._set_timer(5.0, lambda: self._transition_to(self.State.RIGHT_HOME))

        elif new_state == self.State.RIGHT_HOME:
            self.get_logger().info('↩️ RIGHT going HOME…')
            self._safe_home(self.R_arm, 'R')
            self.get_logger().info('⏱ LEFT will go to start in 1s…')
            self._set_timer(1.0, lambda: self._transition_to(self.State.LEFT_TO_START))

        elif new_state == self.State.LEFT_TO_START:
            self._left_to_start()
            self.get_logger().info('⏱ LEFT waiting 5s at start…')
            self._set_timer(5.0, lambda: self._transition_to(self.State.LEFT_MOVE))

        elif new_state == self.State.LEFT_MOVE:
            if self.pose_L_base is None:
                self.get_logger().warn('❔ No stored L target yet; retrying in 0.5s…')
                self._set_timer(0.5, lambda: self._transition_to(self.State.LEFT_MOVE))
                return
            self.get_logger().info('➡️ LEFT moving to ArUco pose in L base (with offset)…')
            self._move_to_target(self.L_arm, 'L', self.pose_L_base, self.L_speed, self.L_acc, self.L_offset)
            self.get_logger().info('⏱ LEFT waiting 5s at target…')
            self._set_timer(5.0, lambda: self._transition_to(self.State.LEFT_HOME))

        elif new_state == self.State.LEFT_HOME:
            self.get_logger().info('↩️ LEFT going HOME…')
            self._safe_home(self.L_arm, 'L')
            self._transition_to(self.State.DONE)

        elif new_state == self.State.DONE:
            self.get_logger().info('🎉 Sequence complete!')

    # ------------- Arm helpers -------------
    def _prepare_controller(self, arm: XArmAPI, tag='?'):
        try:
            if arm.has_error:
                self.get_logger().warn(f'🧹 [{tag}] has_error → clean_error()'); arm.clean_error()
            if arm.has_warn:
                self.get_logger().warn(f'🧹 [{tag}] has_warn → clean_warn()'); arm.clean_warn()
        except Exception as e:
            self.get_logger().warn(f'[{tag}] While clearing errors/warns: {e}')
        for name, fn in [
            ('motion_enable', lambda: arm.motion_enable(True)),
            ('set_mode(0)',   lambda: arm.set_mode(0)),
            ('set_state(0)',  lambda: arm.set_state(0)),
        ]:
            try:
                code = fn(); code = code[0] if isinstance(code, tuple) else code
                if code != 0:
                    self.get_logger().warn(f'[{tag}] {name} returned code {code}')
            except Exception as e:
                self.get_logger().warn(f'[{tag}] {name} raised: {e}')

    def _safe_home(self, arm: XArmAPI, tag='?'):
        code = arm.move_gohome(is_radian=False, wait=True)
        if code != 0:
            self.get_logger().warn(f'🏠 [{tag}] HOME returned code {code} (err={arm.error_code}, warn={arm.warn_code})')

    def _right_to_start(self):
        x, y, z = self.R_start_xyz_m
        roll, pitch, yaw = self.R_start_rpy_deg
        self.get_logger().info(f'➡️ RIGHT ➜ start pose XYZ(mm)=({x*1000:.1f},{y*1000:.1f},{z*1000:.1f}) '
                               f'RPY(deg)=({roll:.2f},{pitch:.2f},{yaw:.2f})')
        code = self.R_arm.set_position(x=x*1000.0, y=y*1000.0, z=z*1000.0,
                                       roll=roll, pitch=pitch, yaw=yaw,
                                       speed=self.R_speed, mvacc=self.R_acc, is_radian=False, wait=True)
        if code != 0:
            self.get_logger().error(f'❌ [R] set_position failed (code={code}, err={self.R_arm.error_code}, warn={self.R_arm.warn_code})')
        else:
            self.get_logger().info('✅ RIGHT start reached.')

    def _left_to_start(self):
        x, y, z = self.L_start_xyz_m
        roll, pitch, yaw = self.L_start_rpy_deg
        self.get_logger().info(f'➡️ LEFT  ➜ start pose XYZ(mm)=({x*1000:.1f},{y*1000:.1f},{z*1000:.1f}) '
                               f'RPY(deg)=({roll:.2f},{pitch:.2f},{yaw:.2f})')
        code = self.L_arm.set_position(x=x*1000.0, y=y*1000.0, z=z*1000.0,
                                       roll=roll, pitch=pitch, yaw=yaw,
                                       speed=self.L_speed, mvacc=self.L_acc, is_radian=False, wait=True)
        if code != 0:
            self.get_logger().error(f'❌ [L] set_position failed (code={code}, err={self.L_arm.error_code}, warn={self.L_arm.warn_code})')
        else:
            self.get_logger().info('✅ LEFT start reached.')

    # ------------- Image helpers -------------
    def _on_compressed_color(self, msg: CompressedImage):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if img is not None:
                self.last_compressed = img
        except Exception:
            pass

    def _convert_color(self, msg: Image):
        if msg.width == 0 or msg.height == 0 or not msg.data:
            raise RuntimeError('color image empty')
        try:
            enc = (msg.encoding or '').lower()
            if enc == 'bgr8':
                return self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            return self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            raise RuntimeError(f'cv_bridge color convert failed: {e}')

    def _convert_depth(self, msg: Image):
        if msg.width == 0 or msg.height == 0 or not msg.data:
            raise RuntimeError('depth image empty')
        try:
            return self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            raise RuntimeError(f'cv_bridge depth convert failed: {e}')

    # ------------- Markers & TF -------------
    def _publish_cuboid(self, pub: rclpy.publisher.Publisher, pose: PoseStamped, ns: str, color=(0.0, 1.0, 0.2, 0.9)):
        m = Marker()
        m.header = pose.header
        m.ns = ns
        m.id = 0
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose = pose.pose
        m.scale.x = self.marker_len
        m.scale.y = self.marker_len
        m.scale.z = 0.02
        m.color.r, m.color.g, m.color.b, m.color.a = color
        m.lifetime = rclpy.duration.Duration(seconds=0.5).to_msg()
        pub.publish(m)

    def _transform_pose_robust(self, src_pose: PoseStamped, target_frame: str) -> PoseStamped:
        try:
            return self.tf_buffer.transform(src_pose, target_frame, timeout=Duration(seconds=0.2))
        except Exception:
            latest = PoseStamped()
            latest.header = Header(frame_id=src_pose.header.frame_id)
            latest.header.stamp.sec = 0
            latest.header.stamp.nanosec = 0
            latest.pose = src_pose.pose
            return self.tf_buffer.transform(latest, target_frame, timeout=Duration(seconds=0.5))

    # ------------- Pose → mm/deg with Z-flip & vertical offset -------------
    def _pose_to_mm_deg_with_flip_and_offset(self, pose_stamped: PoseStamped, offset_mm: float):
        p = pose_stamped.pose.position
        q = pose_stamped.pose.orientation
        flip_q = quaternion_from_euler(math.pi, 0.0, 0.0)  # flip EE Z
        qx, qy, qz, qw = quaternion_multiply((q.x, q.y, q.z, q.w), flip_q)
        roll, pitch, yaw = quat_to_rpy(qx, qy, qz, qw)
        return (
            float(p.x) * 1000.0,
            float(p.y) * 1000.0,
            float(p.z) * 1000.0 + float(offset_mm),
            math.degrees(roll),
            math.degrees(pitch),
            math.degrees(yaw),
        )

    # ---------- small param helper ----------
    def _get_vec_param(self, name: str):
        val = self.get_parameter(name).value
        if isinstance(val, (list, tuple)) and len(val) == 3:
            return float(val[0]), float(val[1]), float(val[2])
        return 0.0, 0.0, 0.0

    # ------------- Move primitive -------------
    def _move_to_target(self, arm: XArmAPI, tag: str, pose_base: PoseStamped, speed: float, acc: float, offset_mm: float):
        x_mm, y_mm, z_mm, roll_deg, pitch_deg, yaw_deg = self._pose_to_mm_deg_with_flip_and_offset(pose_base, offset_mm)

        # 👉 Apply per-arm software bias
        if tag == 'R':
            bx, by, bz = self._get_vec_param('R.bias_xyz_mm')
            br, bp, byaw = self._get_vec_param('R.bias_rpy_deg')
        else:  # 'L'
            bx, by, bz = self._get_vec_param('L.bias_xyz_mm')
            br, bp, byaw = self._get_vec_param('L.bias_rpy_deg')

        x_mm += bx; y_mm += by; z_mm += bz
        roll_deg += br; pitch_deg += bp; yaw_deg += byaw

        self.get_logger().info(f'🪛 [{tag}] bias applied: dXYZ(mm)=({bx:.2f},{by:.2f},{bz:.2f}), dRPY(deg)=({br:.2f},{bp:.2f},{byaw:.2f})')
        self.get_logger().info(
            f'🤖 [{tag}] set_position XYZ(mm)=({x_mm:.1f},{y_mm:.1f},{z_mm:.1f}) '
            f'RPY(deg)=({roll_deg:.2f},{pitch_deg:.2f},{yaw_deg:.2f})'
        )
        code = arm.set_position(x=x_mm, y=y_mm, z=z_mm,
                                roll=roll_deg, pitch=pitch_deg, yaw=yaw_deg,
                                speed=speed, mvacc=acc, is_radian=False, wait=True)
        if code != 0:
            self.get_logger().error(f'❌ [{tag}] move failed (code={code}, err={arm.error_code}, warn={arm.warn_code})')
        else:
            self.get_logger().info(f'✅ [{tag}] at target offset pose.')
            # explicit final command (with bias)
            self.get_logger().info(
                f'📝 [{tag}] final commanded pose (with bias): XYZ(mm)=({x_mm:.1f},{y_mm:.1f},{z_mm:.1f}) '
                f'RPY(deg)=({roll_deg:.2f},{pitch_deg:.2f},{yaw_deg:.2f})'
            )

    # ------------- RGBD callback (gated by FSM) -------------
    def _on_rgbd(self, color_msg: Image, depth_msg: Image, info_msg: CameraInfo):
        # Only process detections during DETECT_WAIT
        if self.state != self.State.DETECT_WAIT:
            return

        # Set intrinsics and enforce cam frame
        if self.K is None:
            k = np.array(info_msg.k, dtype=np.float64)
            if k.size != 9 or info_msg.width == 0 or info_msg.height == 0:
                return
            self.K = k.reshape(3, 3)
            self.D = np.array(info_msg.d, dtype=np.float64).reshape(-1, 1) if len(info_msg.d) > 0 else np.zeros((5, 1))
            self.cam_frame = self.R_cam
            self.get_logger().info(f'🔧 Intrinsics loaded.\nK=\n{self.K}\nD={self.D.ravel()} frame_id={self.cam_frame}')

        # Convert color (fallback)
        try:
            cv_color = self._convert_color(color_msg)
        except RuntimeError:
            if self.last_compressed is None:
                return
            cv_color = self.last_compressed.copy()

        try:
            _ = self._convert_depth(depth_msg)
        except RuntimeError:
            pass

        # Detect ArUco
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

        try:
            rvecs, tvecs, _obj = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_len, self.K, self.D)
        except Exception as e:
            self.get_logger().warn(f'estimatePoseSingleMarkers failed: {e}')
            return

        for i, marker_id in enumerate(ids):
            if marker_id != self.target_id:
                continue

            rvec = rvecs[i].reshape(3, 1)
            tvec = tvecs[i].reshape(3, 1)

            try:
                cv2.aruco.drawAxis(annotated, self.K, self.D, rvec, tvec, 0.05)
            except Exception:
                pass

            # Pose in camera frame (force RIGHT cam frame)
            pose_cam = PoseStamped()
            pose_cam.header = Header(stamp=color_msg.header.stamp, frame_id=self.cam_frame)
            Rm, _ = cv2.Rodrigues(rvec)
            qw = math.sqrt(max(0.0, 1.0 + Rm[0, 0] + Rm[1, 1] + Rm[2, 2])) / 2.0
            qx = (Rm[2, 1] - Rm[1, 2]) / (4.0 * qw + 1e-9)
            qy = (Rm[0, 2] - Rm[2, 0]) / (4.0 * qw + 1e-9)
            qz = (Rm[1, 0] - Rm[0, 1]) / (4.0 * qw + 1e-9)
            norm = math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz) or 1.0
            qx, qy, qz, qw = qx/norm, qy/norm, qz/norm, qw/norm
            tx, ty, tz = [float(v) for v in tvec.ravel()]
            pose_cam.pose.position.x = tx
            pose_cam.pose.position.y = ty
            pose_cam.pose.position.z = tz
            pose_cam.pose.orientation.x = qx
            pose_cam.pose.orientation.y = qy
            pose_cam.pose.orientation.z = qz
            pose_cam.pose.orientation.w = qw

            # TF to bases
            pose_R = None; pose_L = None
            try:
                pose_R = self._transform_pose_robust(pose_cam, self.R_base)
            except Exception as e:
                self.get_logger().warn(f'⛔ TF to {self.R_base} failed: {e}')
            try:
                pose_L = self._transform_pose_robust(pose_cam, self.L_base)
            except Exception as e:
                self.get_logger().warn(f'⛔ TF to {self.L_base} failed: {e}')

            if pose_R:
                self.pose_R_base = pose_R
                self.pose_R_pub.publish(pose_R)
                self._publish_cuboid(self.marker_pub_R, pose_R, 'aruco_cube_R', (0.0, 1.0, 0.2, 0.9))
                pr = pose_R.pose.position; qr = pose_R.pose.orientation
                self.get_logger().info(
                    f'📐 ArUco {marker_id} in {self.R_base}: xyz=({pr.x:.3f}, {pr.y:.3f}, {pr.z:.3f}) '
                    f'quat=({qr.x:.3f},{qr.y:.3f},{qr.z:.3f},{qr.w:.3f})'
                )
            if pose_L:
                self.pose_L_base = pose_L
                self.pose_L_pub.publish(pose_L)
                self._publish_cuboid(self.marker_pub_L, pose_L, 'aruco_cube_L', (0.2, 0.6, 1.0, 0.9))
                pl = pose_L.pose.position; ql = pose_L.pose.orientation
                self.get_logger().info(
                    f'📐 ArUco {marker_id} in {self.L_base}: xyz=({pl.x:.3f}, {pl.y:.3f}, {pl.z:.3f}) '
                    f'quat=({ql.x:.3f},{ql.y:.3f},{ql.z:.3f},{ql.w:.3f})'
                )

            # Advance FSM exactly once (first valid detection)
            if pose_R is not None:
                self.get_logger().info('⏱ RIGHT will move to target in 5 seconds…')
                self._set_timer(5.0, lambda: self._transition_to(self.State.RIGHT_MOVE))
                # Leave DETECT_WAIT; ignore subsequent detections
                self.state = self.State.RIGHT_MOVE
                break

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8'))
        except Exception:
            pass


def main():
    rclpy.init()
    node = DualArmCalibrationFSM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
