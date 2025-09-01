#!/usr/bin/env python3
"""
vision_tracker_node.py — RGB-only callback, robust MASK→3D for lids, minimal logs

- Subscribes:
    • RGB:   vision.camera.rgb_topic  (e.g. /camera/camera/color/image_raw)
    • DEPTH: vision.camera.depth_topic (must be aligned-to-color; can be different size)
    • INFO:  vision.camera.info_topic
- YOLO (seg) → annotated image /vision_debug/image
- 3D per detection:
    1) MASK median XYZ (resize mask → depth size, sample valid points, back-project)
    2) window median around bbox center on depth
    3) nearest-valid depth search
- Publishes:
    • /tracked_parts (hd_disassembly/TrackedPart) in vision.camera.frame_id
    • /vision_tracks (std_msgs/String JSON)
- Logs: startup + one “[DET] id=… label=… pos=(x,y,z) m” per kept detection
"""

import os, json, yaml
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from hd_disassembly.msg import TrackedPart


class VisionTrackerNode(Node):
    def __init__(self):
        super().__init__('vision_tracker_node')
        self.get_logger().info("🚀 VisionTrackerNode starting up…")

        # ---- Load config ----
        pkg = get_package_share_directory('hd_disassembly')
        cfg_file = os.path.join(pkg, 'config', 'disassembler_params.yaml')
        with open(cfg_file, 'r') as f:
            cfg = yaml.safe_load(f)

        cam                = cfg['vision']['camera']
        self.model_path    = cfg['vision']['model_path']
        self.ignore        = set(cfg['vision']['ignore_classes'])
        self.rgb_topic     = cam['rgb_topic']
        self.depth_topic   = cam['depth_topic']          # prefer aligned_depth_to_color
        self.info_topic    = cam['info_topic']
        self.camera_frame  = cam['frame_id']
        self.iou_thres     = float(cfg['tracker']['iou_threshold'])
        self.max_lost      = int(cfg['tracker']['max_lost'])

        # Depth sampling knobs
        self.window_k   = 7
        self.min_pts    = 10
        self.mask_min_pts = 80     # require more for a stable mask median
        self.mask_max_samples = 6000
        self.dilate_iters = 2       # try to fill mask holes on shiny/flats

        # State
        self.bridge = CvBridge()
        self.K = None
        self.fx = self.fy = self.cx = self.cy = None
        self.depth_m = None
        self._streams_logged = False

        # Tracking (per label, local IDs)
        self.tracks  = {}
        self.next_id = {}

        # Model
        try:
            self.model = YOLO(self.model_path)
            self.get_logger().info(f"✅ Loaded YOLO model: {self.model_path}")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to load model: {e}")
            raise

        # I/O
        self.create_subscription(CameraInfo, self.info_topic,  self.info_cb,   10)
        self.create_subscription(Image,      self.depth_topic, self.depth_cb,  10)
        self.create_subscription(Image,      self.rgb_topic,   self.color_cb,  10)

        self.debug_pub   = self.create_publisher(Image,       'vision_debug/image', 10)
        self.json_pub    = self.create_publisher(String,      'vision_tracks',      10)
        self.tracked_pub = self.create_publisher(TrackedPart, 'tracked_parts',      10)

        self.get_logger().info(f"🎥 RGB:   {self.rgb_topic}")
        self.get_logger().info(f"🌊 DEPTH: {self.depth_topic}  (must be aligned-to-color)")
        self.get_logger().info(f"ℹ️ INFO:  {self.info_topic}")
        self.get_logger().info(f"🧭 Output frame: {self.camera_frame}")
        self.get_logger().info("✅ VisionTrackerNode ready.")

    # ----- Callbacks -----
    def info_cb(self, msg: CameraInfo):
        try:
            K = np.array(msg.k, dtype=np.float64).reshape(3,3)
            self.K  = K
            self.fx, self.fy = float(K[0,0]), float(K[1,1])
            self.cx, self.cy = float(K[0,2]), float(K[1,2])
        except Exception:
            self.K = None

    def depth_cb(self, msg: Image):
        try:
            raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except CvBridgeError:
            return
        if raw.dtype == np.uint16:
            self.depth_m = raw.astype(np.float32) * 0.001
        else:
            self.depth_m = raw.astype(np.float32)

    # ----- Helpers -----
    @staticmethod
    def _bbox_center(b):
        x1,y1,x2,y2 = map(int, b)
        return int((x1+x2)//2), int((y1+y2)//2)

    def _window_median_depth(self, u, v, W_d, H_d, k=None, min_pts=None):
        k = k if k is not None else self.window_k
        min_pts = min_pts if min_pts is not None else self.min_pts
        u0 = max(0, u-k); u1 = min(W_d-1, u+k)
        v0 = max(0, v-k); v1 = min(H_d-1, v+k)
        patch = self.depth_m[v0:v1+1, u0:u1+1]
        good = np.isfinite(patch) & (patch > 0)
        if int(good.sum()) < min_pts:
            return None
        return float(np.median(patch[good]))

    def _nearest_valid_depth(self, u, v, W_d, H_d, max_r=15):
        for r in range(1, max_r+1):
            u0 = max(0, u-r); u1 = min(W_d-1, u+r)
            v0 = max(0, v-r); v1 = min(H_d-1, v+r)
            patch = self.depth_m[v0:v1+1, u0:u1+1]
            good = np.isfinite(patch) & (patch > 0)
            if good.any():
                return float(np.median(patch[good]))
        return None

    def _mask_median_xyz(self, mask_img_bool, su, sv, W_d, H_d):
        """
        mask_img_bool: mask at IMAGE size (H_img, W_img) → resize to depth size.
        su, sv: IMAGE→DEPTH scale factors.
        Returns (X,Y,Z) or None.
        """
        # Resize mask to depth size
        mask_d = cv2.resize(mask_img_bool.astype(np.uint8), (W_d, H_d),
                            interpolation=cv2.INTER_NEAREST).astype(bool)

        # Try a couple of dilations to fill typical TOF holes on shiny planar lids
        if self.dilate_iters > 0:
            kernel = np.ones((3,3), np.uint8)
            mask_d = cv2.dilate(mask_d.astype(np.uint8), kernel, iterations=self.dilate_iters).astype(bool)

        Z = self.depth_m
        valid = mask_d & np.isfinite(Z) & (Z > 0)
        n = int(valid.sum())
        if n < self.mask_min_pts:
            return None

        # Sample to cap cost
        if n > self.mask_max_samples:
            ys, xs = np.nonzero(valid)
            idx = np.random.choice(len(xs), size=self.mask_max_samples, replace=False)
            xs = xs[idx]; ys = ys[idx]
        else:
            ys, xs = np.nonzero(valid)

        # Map depth indices → image indices (approx) for back-projection with K
        u_img = xs.astype(np.float32) / float(su)
        v_img = ys.astype(np.float32) / float(sv)
        z_s   = Z[ys, xs].astype(np.float32)

        # Back-project
        Xs = (u_img - self.cx) * z_s / self.fx
        Ys = (v_img - self.cy) * z_s / self.fy

        return float(np.median(Xs)), float(np.median(Ys)), float(np.median(z_s))

    # ----- Main -----
    def color_cb(self, msg: Image):
        if self.K is None or self.depth_m is None:
            return

        # Color to BGR for YOLO/plotting
        try:
            cv = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except CvBridgeError:
            return
        enc = (msg.encoding or '').lower()
        color = cv if enc == 'bgr8' else cv2.cvtColor(cv, cv2.COLOR_RGB2BGR)

        H_img, W_img = color.shape[:2]
        H_d,   W_d   = self.depth_m.shape[:2]
        if not self._streams_logged:
            self._streams_logged = True
            self.get_logger().info(f"📏 Streams: color={W_img}x{H_img}, depth={W_d}x{H_d}")

        # IMAGE → DEPTH scaling
        su = W_d / float(W_img)
        sv = H_d / float(H_img)

        # Inference (quiet)
        res = self.model(color, verbose=False)[0]

        # Annotated image (quiet)
        try:
            ann = res.plot()  # BGR
            ann_msg = self.bridge.cv2_to_imgmsg(ann, encoding='bgr8')
            ann_msg.header = msg.header
            self.debug_pub.publish(ann_msg)
        except CvBridgeError:
            pass

        # Collect 3D detections (mask-first)
        dets3d = {}
        has_masks = getattr(res, "masks", None) is not None and res.masks is not None

        for i, (b, cls) in enumerate(zip(res.boxes.xyxy, res.boxes.cls)):
            label = self.model.names[int(cls)]
            if label in self.ignore:
                continue

            xyz = None
            # 1) Try mask median (best for lids)
            if has_masks and i < len(res.masks.data):
                mask = res.masks.data[i].cpu().numpy()  # (H_img, W_img) after Ultralytics postproc
                if mask.dtype != np.bool_:
                    mask = mask > 0.5
                xyz = self._mask_median_xyz(mask, su, sv, W_d, H_d)

            # 2) Window median at bbox center (fallback)
            if xyz is None:
                u_img, v_img = self._bbox_center(b)
                u = int(np.clip(round(u_img * su), 0, W_d-1))
                v = int(np.clip(round(v_img * sv), 0, H_d-1))
                Z = self._window_median_depth(u, v, W_d, H_d)
                if Z is None:
                    z0 = float(self.depth_m[v, u])
                    if np.isfinite(z0) and z0 > 0:
                        Z = z0
                    else:
                        Z = self._nearest_valid_depth(u, v, W_d, H_d)
                if Z is not None:
                    X = (u_img - self.cx) * Z / self.fx
                    Y = (v_img - self.cy) * Z / self.fy
                    xyz = (X, Y, Z)

            if xyz is None:
                continue

            dets3d.setdefault(label, []).append((tuple(map(int, b)), xyz))

        # Track + publish (per label)
        tracks_out = []
        for label, dets in dets3d.items():
            self.tracks.setdefault(label, {})
            self.next_id.setdefault(label, 0)
            new_tracks, assigned = {}, set()

            for bbox, (X, Y, Z) in dets:
                # per-class IOU match
                best, biou = None, self.iou_thres
                for tid, data in self.tracks[label].items():
                    iou = self.iou(bbox, data['bbox'])
                    if iou > biou:
                        biou, best = iou, tid

                if best is None:
                    best = self.next_id[label]
                    self.next_id[label] += 1

                new_tracks[best] = {'bbox': bbox, 'lost': 0, 'pos': (X, Y, Z)}
                assigned.add(best)

                # Publish TrackedPart in camera frame (matches calibration)
                p = PointStamped()
                p.header = msg.header
                p.header.frame_id = self.camera_frame
                p.point.x, p.point.y, p.point.z = float(X), float(Y), float(Z)

                tp = TrackedPart()
                tp.header = msg.header
                tp.track_id = best
                tp.part_label = label
                tp.position = p
                self.tracked_pub.publish(tp)

                # Minimal log per kept detection
                self.get_logger().info(f"[DET] id={best} label={label} pos=({X:.3f}, {Y:.3f}, {Z:.3f}) m")

                tracks_out.append({
                    'id': best,
                    'part': label,
                    'position': {'x': X, 'y': Y, 'z': Z},
                    'frame': self.camera_frame
                })

            # prune unmatched
            for tid, data in self.tracks[label].items():
                if tid not in assigned and data['lost'] + 1 < self.max_lost:
                    new_tracks[tid] = {'bbox': data['bbox'], 'lost': data['lost'] + 1, 'pos': data['pos']}
            self.tracks[label] = new_tracks

        # JSON summary
        if tracks_out:
            out = String()
            out.data = json.dumps(tracks_out)
            self.json_pub.publish(out)

    # ----- utils -----
    @staticmethod
    def iou(A, B):
        xA, yA = max(A[0], B[0]), max(A[1], B[1])
        xB, yB = min(A[2], B[2]), min(A[3], B[3])
        inter = max(0, xB - xA) * max(0, yB - yA)
        aA = (A[2]-A[0])*(A[3]-A[1]); aB = (B[2]-B[0])*(B[3]-B[1])
        return inter / (aA + aB - inter + 1e-8)


def main(args=None):
    rclpy.init(args=args)
    node = VisionTrackerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
