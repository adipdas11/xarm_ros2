#!/usr/bin/env python3
"""
vision_detector_node_detr.py — RF-DETR + depth back-projection + supervision overlays
(compat fixes for Supervision palette + annotate API)

- RF-DETR weights from vision.model_path
- Class names from COCO json at vision.coco_json_path
- Ignores class "screw" by default; you can allow-list with vision.class_names
- Depth must be aligned to color
"""

import os, json, yaml
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from hd_disassembly.msg import TrackedPart

from PIL import Image as PILImage
from rfdetr import RFDETRLarge

import supervision as sv


class VisionRFDetrNode(Node):
    def __init__(self):
        super().__init__('vision_tracker_node')
        self.get_logger().info("🚀 VisionTrackerNode (RF-DETR) starting…")

        # ---- Load config ----
        pkg = get_package_share_directory('hd_disassembly')
        cfg_file = os.path.join(pkg, 'config', 'disassembler_params.yaml')
        with open(cfg_file, 'r') as f:
            cfg = yaml.safe_load(f)

        vcfg              = cfg['vision']
        cam               = vcfg['camera']
        self.model_path   = vcfg['model_path']
        self.coco_json    = vcfg['coco_json_path']
        self.rgb_topic    = cam['rgb_topic']
        self.depth_topic  = cam['depth_topic']
        self.info_topic   = cam['info_topic']
        self.cfg_frame_id = cam['frame_id']
        self.conf_thres   = float(vcfg.get('conf_threshold', 0.30))

        self.keep_names   = set(vcfg.get('class_names', []) or [])
        self.ignore_names = set(vcfg.get('ignore_names', ['screw']))

        self.bridge = CvBridge()
        self.K = None
        self.fx = self.fy = self.cx = self.cy = None
        self.depth_m = None
        self.output_frame_id = self.cfg_frame_id
        self._streams_logged = False
        self._frame_warned   = False

        self.window_k  = 7
        self.min_pts   = 10

        self.tracks  = {}
        self.next_id = {}

        # ---- Names from COCO json ----
        self.id_to_name = self._load_coco_names(self.coco_json)
        self.get_logger().info(f"🗂 Loaded {len(self.id_to_name)} class names from COCO json")

        if not self.keep_names:
            self.get_logger().info("ℹ️ No class_names provided → not filtering by names. "
                                   "Set vision.class_names in YAML to enable name-based filtering.")
        else:
            self.get_logger().info(f"🎯 keep_names={sorted(self.keep_names)}")
        if self.ignore_names:
            self.get_logger().info(f"🚫 ignore_names={sorted(self.ignore_names)}")

        # ---- RF-DETR ----
        self.get_logger().info("⏳ Loading RF-DETR weights…")
        self.model = RFDETRLarge(pretrain_weights=self.model_path)
        try:
            self.model.optimize_for_inference()
        except Exception:
            pass
        self.get_logger().info(f"✅ Loaded RF-DETR weights: {self.model_path}")

        self.get_logger().info(f"🎥 RGB:   {self.rgb_topic}")
        self.get_logger().info(f"🌊 DEPTH: {self.depth_topic}  (must be aligned to color)")
        self.get_logger().info(f"ℹ️ INFO:  {self.info_topic}")
        self.get_logger().info(f"🧭 Output frame (configured): {self.cfg_frame_id}")
        self.get_logger().info(f"🎯 conf_threshold={self.conf_thres:.2f}")

        # ---- ROS I/O ----
        self.create_subscription(CameraInfo, self.info_topic,  self.info_cb,   10)
        self.create_subscription(Image,      self.depth_topic, self.depth_cb,  10)
        self.create_subscription(Image,      self.rgb_topic,   self.color_cb,  10)

        self.debug_pub   = self.create_publisher(Image,       'vision_debug/image', 10)
        self.json_pub    = self.create_publisher(String,      'vision_tracks',      10)
        self.tracked_pub = self.create_publisher(TrackedPart, 'tracked_parts',      20)

        # Annotators (lazy-init after first frame to size them correctly)
        self.box_annotator   = None
        self.label_annotator = None

        self.get_logger().info("✅ VisionTrackerNode ready.")

    # ---------- Callbacks ----------
    def info_cb(self, msg: CameraInfo):
        try:
            K = np.array(msg.k, dtype=np.float64).reshape(3,3)
            self.K  = K
            self.fx, self.fy = float(K[0,0]), float(K[1,1])
            self.cx, self.cy = float(K[0,2]), float(K[1,2])
        except Exception:
            self.K = None
            return

        cam_frame = msg.header.frame_id or self.cfg_frame_id
        if not self._frame_warned and cam_frame and cam_frame != self.cfg_frame_id:
            self.get_logger().info(
                f"🔁 Camera frame from CameraInfo differs ('{cam_frame}' vs '{self.cfg_frame_id}'). "
                f"Using '{cam_frame}' for outputs."
            )
            self._frame_warned = True
        self.output_frame_id = cam_frame or self.cfg_frame_id

    def depth_cb(self, msg: Image):
        try:
            raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except CvBridgeError:
            return
        self.depth_m = raw.astype(np.float32) * 0.001 if raw.dtype == np.uint16 else raw.astype(np.float32)

    def color_cb(self, msg: Image):
        if self.K is None or self.depth_m is None:
            return

        try:
            cvimg = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except CvBridgeError:
            return
        enc = (msg.encoding or '').lower()
        color_bgr = cvimg if enc == 'bgr8' else cv2.cvtColor(cvimg, cv2.COLOR_RGB2BGR)

        H_img, W_img = color_bgr.shape[:2]
        H_d,   W_d   = self.depth_m.shape[:2]

        if not self._streams_logged:
            self._streams_logged = True
            self.get_logger().info(f"📏 Streams: color={W_img}x{H_img}, depth={W_d}x{H_d}")

            # --- Supervision palette (robust across versions) ---
            palette = None
            # Try common APIs in order
            for candidate in (
                getattr(sv.ColorPalette, 'DEFAULT', None),
                getattr(sv.ColorPalette, 'default', None),  # some versions expose a function attr
            ):
                if candidate is not None:
                    palette = candidate if not callable(candidate) else candidate()
                    break
            if palette is None:
                # Fallback: custom palette
                palette = sv.ColorPalette.from_hex([
                    "#ffff00", "#ff9b00", "#ff66ff", "#3399ff", "#ff66b2", "#ff8080",
                    "#b266ff", "#9999ff", "#66ffff", "#33ff99", "#66ff66", "#99ff00"
                ])

            text_scale = sv.calculate_optimal_text_scale(resolution_wh=(W_img, H_img))
            thickness  = sv.calculate_optimal_line_thickness(resolution_wh=(W_img, H_img))

            self.box_annotator   = sv.BoxAnnotator(color=palette, thickness=thickness)
            # omit text_color for broader version compatibility
            self.label_annotator = sv.LabelAnnotator(color=palette, text_scale=text_scale)

        # IMAGE → DEPTH scale
        su  = W_d / float(W_img)
        svs = H_d / float(H_img)

        # -------- Inference --------
        try:
            pil_img = PILImage.fromarray(cv2.cvtColor(color_bgr, cv2.COLOR_BGR2RGB))
            det = self.model.predict(pil_img, threshold=self.conf_thres)
        except Exception as e:
            self.get_logger().warn(f"Inference error: {e}")
            return

        xyxy      = np.asarray(det.xyxy)
        conf      = np.asarray(det.confidence) if hasattr(det, 'confidence') else np.ones((len(xyxy),), dtype=np.float32)
        class_id  = np.asarray(det.class_id).astype(int) if hasattr(det, 'class_id') else np.zeros((len(xyxy),), dtype=int)

        sup_det = sv.Detections(xyxy=xyxy, confidence=conf, class_id=class_id)

        kept_indices = []
        dets3d_by_name = {}

        for i in range(len(xyxy)):
            cid  = int(class_id[i])
            name = self.id_to_name.get(cid, f"class_{cid}")
            if name in self.ignore_names:
                continue
            if self.keep_names and (name not in self.keep_names):
                continue

            x1, y1, x2, y2 = map(int, xyxy[i])
            u_img = int((x1 + x2) // 2)
            v_img = int((y1 + y2) // 2)
            u = int(np.clip(round(u_img * su), 0, W_d-1))
            v = int(np.clip(round(v_img * svs), 0, H_d-1))

            Z = self._window_median_depth(u, v, W_d, H_d)
            if Z is None:
                z0 = float(self.depth_m[v, u])
                if np.isfinite(z0) and z0 > 0:
                    Z = z0
                else:
                    Z = self._nearest_valid_depth(u, v, W_d, H_d)
            if Z is None or not np.isfinite(Z) or Z <= 0:
                continue

            X = (u_img - self.cx) * Z / self.fx
            Y = (v_img - self.cy) * Z / self.fy

            bbox = (int(x1), int(y1), int(x2), int(y2))
            dets3d_by_name.setdefault(name, []).append((bbox, (float(X), float(Y), float(Z)), float(conf[i]), cid))
            kept_indices.append(i)

        # -------- Track + publish --------
        tracks_out = []
        for name, lst in dets3d_by_name.items():
            self.tracks.setdefault(name, {})
            self.next_id.setdefault(name, 0)
            new_tracks, assigned = {}, set()

            for bbox, (X, Y, Z), c, cid in lst:
                best, biou = None, 0.0
                for tid, data in self.tracks[name].items():
                    iou = self._iou(bbox, data['bbox'])
                    if iou > biou and iou > 0.1:
                        biou, best = iou, tid
                if best is None:
                    best = self.next_id[name]
                    self.next_id[name] += 1

                new_tracks[best] = {'bbox': bbox, 'lost': 0, 'pos': (X, Y, Z), 'conf': c}
                assigned.add(best)

                p = PointStamped()
                p.header = msg.header
                p.header.frame_id = self.output_frame_id
                p.point.x, p.point.y, p.point.z = float(X), float(Y), float(Z)

                tp = TrackedPart()
                tp.header = msg.header
                tp.track_id = int(best)
                tp.part_label = name
                tp.position = p
                self.tracked_pub.publish(tp)

                self.get_logger().info(f"[DET] id={best} label={name} pos=({X:.3f}, {Y:.3f}, {Z:.3f}) m")

                tracks_out.append({
                    'id': int(best),
                    'part': name,
                    'confidence': float(c),
                    'class_id': int(cid),
                    'position': {'x': float(X), 'y': float(Y), 'z': float(Z)},
                    'frame': self.output_frame_id
                })

            for tid, data in self.tracks[name].items():
                if tid not in assigned and data['lost'] + 1 < 10:
                    new_tracks[tid] = {'bbox': data['bbox'], 'lost': data['lost'] + 1, 'pos': data['pos'], 'conf': data.get('conf', 0.0)}
            self.tracks[name] = new_tracks

        # -------- RViz overlay --------
        if kept_indices and self.box_annotator is not None:
            sub = sup_det[kept_indices]
            labels = []
            for i in range(len(sub)):
                cid = int(sub.class_id[i])
                nm  = self.id_to_name.get(cid, f"class_{cid}")
                cf  = float(sub.confidence[i]) if sub.confidence is not None else 1.0
                labels.append(f"{nm} {cf:.2f}")

            vis = color_bgr.copy()
            vis = self.box_annotator.annotate(scene=vis, detections=sub)
            vis = self.label_annotator.annotate(scene=vis, detections=sub, labels=labels)

            try:
                dbg = self.bridge.cv2_to_imgmsg(vis, encoding='bgr8')
                dbg.header = msg.header
                dbg.header.frame_id = self.output_frame_id
                self.debug_pub.publish(dbg)
            except CvBridgeError:
                pass

        if tracks_out:
            out = String()
            out.data = json.dumps(tracks_out)
            self.json_pub.publish(out)

    # ---------- Helpers ----------
    def _load_coco_names(self, json_path: str):
        try:
            with open(json_path, 'r') as f:
                data = json.load(f)
            return {int(c['id']): str(c['name']) for c in data.get('categories', [])}
        except Exception as e:
            self.get_logger().warn(f"Could not load COCO categories from {json_path}: {e}")
            return {}

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

    @staticmethod
    def _iou(A, B):
        xA = max(A[0], B[0]); yA = max(A[1], B[1])
        xB = min(A[2], B[2]); yB = min(A[3], B[3])
        inter = max(0, xB - xA) * max(0, yB - yA)
        aA = max(0, A[2]-A[0]) * max(0, A[3]-A[1])
        aB = max(0, B[2]-B[0]) * max(0, B[3]-B[1])
        denom = (aA + aB - inter)
        return (inter / denom) if denom > 1e-8 else 0.0


def main(args=None):
    rclpy.init(args=args)
    node = VisionRFDetrNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
