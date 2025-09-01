#!/usr/bin/env python3
"""
vision_detector_yolo_sahi.py — YOLOv11 + SAHI tiled inference, robust MASK→3D

- Subscribes:
    RGB:   vision.camera.rgb_topic
    DEPTH: vision.camera.depth_topic (aligned-to-color)
    INFO:  vision.camera.info_topic
- Inference:
    * Preferred: SAHI tiled prediction with Ultralytics model (better small-object recall)
    * Fallback:  plain YOLO predict if SAHI is missing/disabled
- 3D per detection:
    1) MASK median XYZ (if mask available; best for lids)
    2) window median around bbox center
    3) nearest-valid depth search
- Publishes:
    /tracked_parts (hd_disassembly/TrackedPart) in camera frame
    /vision_tracks (std_msgs/String JSON)
    /vision_debug/image (annotated)
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

# ---- Optional SAHI imports (lazy/fallback if missing) ----
_SAHI_AVAILABLE = True
try:
    import torch
    from sahi.predict import get_sliced_prediction
    from sahi import AutoDetectionModel
except Exception:
    _SAHI_AVAILABLE = False
    torch = None


class VisionTrackerNode(Node):
    def __init__(self):
        super().__init__('vision_tracker_node')
        self.get_logger().info("🚀 VisionTrackerNode (YOLOv11 + SAHI) starting…")

        # ---- Load config ----
        pkg = get_package_share_directory('hd_disassembly')
        cfg_file = os.path.join(pkg, 'config', 'disassembler_params.yaml')
        with open(cfg_file, 'r') as f:
            cfg = yaml.safe_load(f)

        cam                 = cfg['vision']['camera']
        self.model_path     = cfg['vision']['model_path']
        self.ignore         = set(cfg['vision'].get('ignore_classes', []))
        self.rgb_topic      = cam['rgb_topic']
        self.depth_topic    = cam['depth_topic']
        self.info_topic     = cam['info_topic']
        self.camera_frame   = cam['frame_id']

        # Detection thresholds
        self.conf_thres     = float(cfg['vision'].get('conf_threshold', 0.30))
        self.iou_nms        = float(cfg['vision'].get('iou_nms', 0.45))

        # SAHI (tiled) inference knobs
        sahi_cfg            = cfg['vision'].get('sahi', {})
        self.sahi_enabled   = bool(sahi_cfg.get('enabled', True))
        self.sahi_tile      = int(sahi_cfg.get('tile', 640))
        self.sahi_overlap   = float(sahi_cfg.get('overlap', 0.20))
        self.get_logger().info(f"🧩 SAHI: enabled={self.sahi_enabled}, tile={self.sahi_tile}, overlap={self.sahi_overlap}")

        # Tracker knobs
        self.iou_thres      = float(cfg['tracker']['iou_threshold'])
        self.max_lost       = int(cfg['tracker']['max_lost'])

        # Depth sampling knobs
        self.window_k         = 7
        self.min_pts          = 10
        self.mask_min_pts     = 80
        self.mask_max_samples = 6000
        self.dilate_iters     = 2

        # State
        self.bridge = CvBridge()
        self.K = None
        self.fx = self.fy = self.cx = self.cy = None
        self.depth_m = None
        self._streams_logged = False

        # Tracking (per label, local IDs)
        self.tracks  = {}
        self.next_id = {}

        # Model (Ultralytics)
        try:
            self.model = YOLO(self.model_path)
            self.names = getattr(self.model.model, "names", getattr(self.model, "names", {}))
            self.get_logger().info(f"✅ Loaded YOLO model: {self.model_path}")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to load YOLO model: {e}")
            raise

        # SAHI wrapper (optional)
        self.sahi_model = None
        if self.sahi_enabled:
            if not _SAHI_AVAILABLE:
                self.get_logger().warn("⚠️ SAHI not installed → falling back to plain YOLO. `pip install sahi` to enable tiling.")
                self.sahi_enabled = False
            else:
                try:
                    device = "cuda:0" if (torch and torch.cuda.is_available()) else "cpu"
                    self.sahi_model = AutoDetectionModel.from_pretrained(
                        model_type="ultralytics",
                        model_path=self.model_path,
                        confidence_threshold=self.conf_thres,
                        device=device,
                    )
                except Exception as e:
                    self.get_logger().warn(f"⚠️ SAHI init failed ({e}) → using plain YOLO.")
                    self.sahi_enabled = False

        # I/O
        self.create_subscription(CameraInfo, self.info_topic,  self.info_cb,   10)
        self.create_subscription(Image,      self.depth_topic, self.depth_cb,  10)
        self.create_subscription(Image,      self.rgb_topic,   self.color_cb,  10)

        self.debug_pub   = self.create_publisher(Image,       'vision_debug/image', 10)
        self.json_pub    = self.create_publisher(String,      'vision_tracks',      10)
        self.tracked_pub = self.create_publisher(TrackedPart, 'tracked_parts',      10)

        self.get_logger().info(f"🎥 RGB:   {self.rgb_topic}")
        self.get_logger().info(f"🌊 DEPTH: {self.depth_topic}  (must be aligned to color)")
        self.get_logger().info(f"ℹ️ INFO:  {self.info_topic}")
        self.get_logger().info(f"🧭 Output frame: {self.camera_frame}")
        self.get_logger().info(f"🎯 conf={self.conf_thres:.2f}, iou_nms={self.iou_nms:.2f}")
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

        if msg.header.frame_id and msg.header.frame_id != self.camera_frame:
            self.get_logger().info(
                f"🔁 Camera frame from CameraInfo differs ('{msg.header.frame_id}' vs '{self.camera_frame}'). "
                f"Using '{msg.header.frame_id}' for outputs."
            )
            self.camera_frame = msg.header.frame_id

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
        mask_d = cv2.resize(mask_img_bool.astype(np.uint8), (W_d, H_d),
                            interpolation=cv2.INTER_NEAREST).astype(bool)
        if self.dilate_iters > 0:
            kernel = np.ones((3,3), np.uint8)
            mask_d = cv2.dilate(mask_d.astype(np.uint8), kernel, iterations=self.dilate_iters).astype(bool)
        Z = self.depth_m
        valid = mask_d & np.isfinite(Z) & (Z > 0)
        n = int(valid.sum())
        if n < self.mask_min_pts:
            return None
        if n > self.mask_max_samples:
            ys, xs = np.nonzero(valid)
            idx = np.random.choice(len(xs), size=self.mask_max_samples, replace=False)
            xs = xs[idx]; ys = ys[idx]
        else:
            ys, xs = np.nonzero(valid)
        u_img = xs.astype(np.float32) / float(su)
        v_img = ys.astype(np.float32) / float(sv)
        z_s   = Z[ys, xs].astype(np.float32)
        Xs = (u_img - self.cx) * z_s / self.fx
        Ys = (v_img - self.cy) * z_s / self.fy
        return float(np.median(Xs)), float(np.median(Ys)), float(np.median(z_s))

    # ----- Inference wrappers -----
    def infer_sahi(self, color_bgr):
        """
        SAHI tiled prediction. Returns:
        boxes_xyxy[N,4], classes[N], scores[N], masks_list or None, labels[N] (strings)
        """
        res = get_sliced_prediction(
            image=color_bgr,
            detection_model=self.sahi_model,
            slice_height=self.sahi_tile,
            slice_width=self.sahi_tile,
            overlap_height_ratio=self.sahi_overlap,
            overlap_width_ratio=self.sahi_overlap,
            postprocess_type="NMS",
            postprocess_match_metric="IOU",
            postprocess_match_threshold=self.iou_nms,
        )

        boxes, classes, scores, masks, labels = [], [], [], [], []
        for obj in res.object_prediction_list:
            # bbox
            x1, y1, x2, y2 = obj.bbox.to_xyxy()
            boxes.append([x1, y1, x2, y2])

            # --- robust category extraction across SAHI versions ---
            cls_id = getattr(obj, "category_id", None)
            name   = getattr(obj, "category_name", None)
            if cls_id is None or name is None:
                cat = getattr(obj, "category", None)
                if cat is not None:
                    cls_id = getattr(cat, "id", cls_id)
                    name   = getattr(cat, "name", name)
            if cls_id is None:
                cls_id = -1
            if name is None:
                # fallback to Ultralytics names if possible
                name = self.names.get(cls_id, f"class_{cls_id}") if isinstance(self.names, dict) else f"class_{cls_id}"

            classes.append(int(cls_id))
            labels.append(str(name))

            # score
            sc = getattr(obj, "score", None)
            val = getattr(sc, "value", sc) if sc is not None else 0.0
            try:
                scores.append(float(val))
            except Exception:
                scores.append(0.0)

            # mask (robust)
            m = None
            try:
                if getattr(obj, "mask", None) is not None:
                    m = getattr(obj.mask, "bool_mask", None)
                    if m is None:
                        # try common converters
                        if hasattr(obj.mask, "to_bool_mask"):
                            bm = obj.mask.to_bool_mask()
                            m = getattr(bm, "mask", None) or getattr(bm, "bool_mask", None)
                        elif hasattr(obj.mask, "numpy"):
                            m = obj.mask.numpy()
                        elif hasattr(obj.mask, "array"):
                            m = obj.mask.array
                if m is not None:
                    m = (np.asarray(m) > 0).astype(np.uint8)
            except Exception:
                m = None
            masks.append(m)

        boxes   = np.array(boxes, dtype=np.float32) if boxes else np.zeros((0,4), dtype=np.float32)
        classes = np.array(classes, dtype=int) if classes else np.zeros((0,), dtype=int)
        scores  = np.array(scores, dtype=np.float32) if scores else np.zeros((0,), dtype=np.float32)
        # keep masks as list; set to None if none present
        masks   = masks if any(m is not None for m in masks) else None
        return boxes, classes, scores, masks, labels

    def infer_yolo_plain(self, color_bgr):
        """
        Plain Ultralytics predict (no tiling). Returns same tuple as infer_sahi.
        """
        results = self.model.predict(
            color_bgr,
            conf=self.conf_thres,
            iou=self.iou_nms,
            verbose=False,
            stream=False,
            retina_masks=True,
        )
        res = results[0]
        boxes   = res.boxes.xyxy.cpu().numpy() if res.boxes is not None else np.zeros((0,4))
        classes = res.boxes.cls.cpu().numpy().astype(int) if res.boxes is not None else np.zeros((0,), dtype=int)
        scores  = res.boxes.conf.cpu().numpy().astype(np.float32) if res.boxes is not None else np.zeros((0,), dtype=np.float32)
        masks   = None
        if hasattr(res, 'masks') and res.masks is not None and res.masks.data is not None:
            masks = [m.cpu().numpy().astype(np.uint8) for m in res.masks.data]  # list of HxW
        # labels from Ultralytics names
        labels = [self.names.get(int(c), f"class_{int(c)}") if isinstance(self.names, dict) else f"class_{int(c)}"
                  for c in classes]
        return boxes, classes, scores, masks, labels

    # ----- Main -----
    def color_cb(self, msg: Image):
        if self.K is None or self.depth_m is None:
            return

        # Color image (BGR)
        try:
            cvimg = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except CvBridgeError:
            return
        enc = (msg.encoding or '').lower()
        color = cvimg if enc == 'bgr8' else cv2.cvtColor(cvimg, cv2.COLOR_RGB2BGR)

        H_img, W_img = color.shape[:2]
        H_d,   W_d   = self.depth_m.shape[:2]
        if not self._streams_logged:
            self._streams_logged = True
            self.get_logger().info(f"📏 Streams: color={W_img}x{H_img}, depth={W_d}x{H_d}")

        su = W_d / float(W_img)
        sv = H_d / float(H_img)

        # ---------- Inference ----------
        try:
            if self.sahi_enabled and self.sahi_model is not None:
                boxes_xyxy, classes, scores, masks, labels = self.infer_sahi(color)
            else:
                boxes_xyxy, classes, scores, masks, labels = self.infer_yolo_plain(color)
        except Exception as e:
            self.get_logger().error(f"❌ Inference failed: {e}")
            return

        # ---------- Annotated image ----------
        ann = color.copy()
        for i, b in enumerate(boxes_xyxy):
            x1,y1,x2,y2 = [int(v) for v in b]
            label = labels[i] if i < len(labels) else f"class_{int(classes[i])}"
            score = float(scores[i]) if i < len(scores) else 0.0

            if label in self.ignore:
                continue

            if masks is not None and i < len(masks) and masks[i] is not None:
                m = masks[i].astype(bool)
                try:
                    cnts, _ = cv2.findContours(m.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    cv2.drawContours(ann, cnts, -1, (0,255,0), 2)
                    ann[m] = ann[m] * 0.6 + np.array([0,255,0], dtype=np.uint8) * 0.4
                except Exception:
                    pass

            cv2.rectangle(ann, (x1,y1), (x2,y2), (255,0,0), 2)
            cv2.putText(ann, f"{label} {score:.2f}", (x1, max(0,y1-6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,0,0), 2, cv2.LINE_AA)

        try:
            ann_msg = self.bridge.cv2_to_imgmsg(ann, encoding='bgr8')
            ann_msg.header = msg.header
            self.debug_pub.publish(ann_msg)
        except CvBridgeError:
            pass

        # ---------- Depth→3D per detection ----------
        dets3d = {}
        for i, b in enumerate(boxes_xyxy):
            x1,y1,x2,y2 = b
            label = labels[i] if i < len(labels) else f"class_{int(classes[i])}"
            if label in self.ignore:
                continue

            xyz = None
            # 1) mask median if available
            if masks is not None and i < len(masks) and masks[i] is not None:
                mask = masks[i].astype(bool)
                xyz = self._mask_median_xyz(mask, su, sv, W_d, H_d)

            # 2) window median at bbox center
            if xyz is None:
                u_img, v_img = self._bbox_center([x1,y1,x2,y2])
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

            dets3d.setdefault(label, []).append(([int(x1),int(y1),int(x2),int(y2)], xyz))

        # ---------- Track + publish ----------
        tracks_out = []
        for label, dets in dets3d.items():
            self.tracks.setdefault(label, {})
            self.next_id.setdefault(label, 0)
            new_tracks, assigned = {}, set()

            for bbox, (X, Y, Z) in dets:
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

                self.get_logger().info(f"[DET] id={best} label={label} pos=({X:.3f}, {Y:.3f}, {Z:.3f}) m")
                tracks_out.append({'id': best, 'part': label,
                                   'position': {'x': X, 'y': Y, 'z': Z},
                                   'frame': self.camera_frame})

            for tid, data in self.tracks[label].items():
                if tid not in assigned and data['lost'] + 1 < self.max_lost:
                    new_tracks[tid] = {'bbox': data['bbox'], 'lost': data['lost'] + 1, 'pos': data['pos']}
            self.tracks[label] = new_tracks

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
