#!/usr/bin/env python3
"""
tool_cam_rfdetr_node.py — RFDETR (supervision) detector on ROS2 camera topic

Publishes:
  • ~/xy_dir     (std_msgs/Float32MultiArray) -> [x_cmd, y_cmd] in {-step_mag, 0, +step_mag}
  • ~/annotated  (sensor_msgs/Image)

ROS params:
  tool_camera.rgb_topic   (string, default "/tool_camera")
  prefer_compressed       (bool,   default False)   # kept for compatibility, not used

  model.weights           (string, default "/path/to/checkpoint_best_total.pth")
  model.threshold         (double, default 0.5)
  model.infer_scale       (double, default 1.0)     # e.g. 0.75 or 0.5 for speed
  perf.frame_skip         (int,    default 0)       # run detector every N+1 frames

  viz.crosshair_x         (int,    default 340)
  viz.crosshair_y         (int,    default 145)
  viz.tolerance_px        (int,    default 10)
  viz.crosshair_len_px    (int,    default 30)
  viz.crosshair_thickness (int,    default 2)
  viz.line_thickness      (int,    default 2)

  axis.swap_xy            (bool,   default True)   # your last snippet defaults
  axis.invert_x           (bool,   default True)
  axis.invert_y           (bool,   default True)
  axis.step_mag_x         (double, default 0.5)    # output magnitude for X
  axis.step_mag_y         (double, default 0.5)    # output magnitude for Y
"""

import time
from typing import Optional, Tuple
from contextlib import nullcontext

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data  # for the camera subscriber only

from std_msgs.msg import Header, Float32MultiArray
from sensor_msgs.msg import Image

# RFDETR + supervision
from PIL import Image as PILImage
import supervision as sv
from rfdetr import RFDETRLarge  # adjust import if needed

try:
    import torch
    HAVE_TORCH = True
except Exception:
    HAVE_TORCH = False


# ------------------------ helpers ------------------------

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def draw_axes_top_left(img, origin=(20, 20), axis_len=50, color_x=(0, 0, 255), color_y=(0, 255, 0)):
    ox, oy = origin
    cv2.arrowedLine(img, (ox, oy), (ox + axis_len, oy), color_x, 2, cv2.LINE_AA, tipLength=0.3)
    cv2.putText(img, "+X", (ox + axis_len + 6, oy + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_x, 1, cv2.LINE_AA)
    cv2.arrowedLine(img, (ox, oy), (ox, oy + axis_len), color_y, 2, cv2.LINE_AA, tipLength=0.3)
    cv2.putText(img, "+Y", (ox - 6, oy + axis_len + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_y, 1, cv2.LINE_AA)

def draw_crosshair(img, x, y, size=20, color=(0, 255, 255), thickness=2):
    h, w = img.shape[:2]
    x = int(clamp(x, 0, w - 1)); y = int(clamp(y, 0, h - 1))
    cv2.line(img, (x - size, y), (x + size, y), color, thickness, cv2.LINE_AA)
    cv2.line(img, (x, y - size), (x, y + size), color, thickness, cv2.LINE_AA)
    cv2.circle(img, (x, y), max(1, thickness), color, -1, cv2.LINE_AA)

def nearest_detection_center(detections: sv.Detections, tx: int, ty: int) -> Tuple[Optional[int], Optional[int], Optional[int]]:
    if detections is None or len(detections) == 0:
        return None, None, None
    xyxy = np.asarray(detections.xyxy)
    cx = (xyxy[:, 0] + xyxy[:, 2]) * 0.5
    cy = (xyxy[:, 1] + xyxy[:, 3]) * 0.5
    d2 = (cx - tx) ** 2 + (cy - ty) ** 2
    idx = int(np.argmin(d2))
    return int(round(cx[idx])), int(round(cy[idx])), idx


# ------------------------ ROS2 node ------------------------

class ToolCamRFDETR(Node):
    def __init__(self):
        # keep SAME node name to preserve ~ topic FQDNs
        super().__init__('tool_cam_rfdetr')

        # --- params ---
        self.declare_parameter('tool_camera.rgb_topic', '/tool_camera')
        self.declare_parameter('prefer_compressed', False)  # compatibility only

        self.declare_parameter('model.weights', '/home/adip/workspaces/image_processing_ws/Model_Training/rfdetr/output_2/checkpoint_best_total.pth')
        self.declare_parameter('model.threshold', 0.5)
        self.declare_parameter('model.infer_scale', 1.0)
        self.declare_parameter('perf.frame_skip', 0)

        self.declare_parameter('viz.crosshair_x', 340)
        self.declare_parameter('viz.crosshair_y', 142)
        self.declare_parameter('viz.tolerance_px', 10)
        self.declare_parameter('viz.crosshair_len_px', 30)
        self.declare_parameter('viz.crosshair_thickness', 2)
        self.declare_parameter('viz.line_thickness', 2)

        # your last defaults were True for these:
        self.declare_parameter('axis.swap_xy', False)
        self.declare_parameter('axis.invert_x', True)
        self.declare_parameter('axis.invert_y', False)

        # NEW: step magnitudes (tunable)
        self.declare_parameter('axis.step_mag_x', 0.05)
        self.declare_parameter('axis.step_mag_y', 0.05)

        # read params
        self.rgb_topic   = self.get_parameter('tool_camera.rgb_topic').value
        self.weights     = self.get_parameter('model.weights').value
        self.threshold   = float(self.get_parameter('model.threshold').value)
        self.infer_scale = float(self.get_parameter('model.infer_scale').value)
        self.frame_skip  = int(self.get_parameter('perf.frame_skip').value)

        self.cross_x     = int(self.get_parameter('viz.crosshair_x').value)
        self.cross_y     = int(self.get_parameter('viz.crosshair_y').value)
        self.tol_px      = int(self.get_parameter('viz.tolerance_px').value)
        self.cross_len   = int(self.get_parameter('viz.crosshair_len_px').value)
        self.cross_thk   = int(self.get_parameter('viz.crosshair_thickness').value)
        self.line_thk    = int(self.get_parameter('viz.line_thickness').value)

        self.swap_xy     = bool(self.get_parameter('axis.swap_xy').value)
        self.invert_x    = bool(self.get_parameter('axis.invert_x').value)
        self.invert_y    = bool(self.get_parameter('axis.invert_y').value)

        self.step_mag_x  = float(self.get_parameter('axis.step_mag_x').value)
        self.step_mag_y  = float(self.get_parameter('axis.step_mag_y').value)

        # publishers — DEFAULT QoS (RELIABLE) to match subscribers
        self.pub_dir   = self.create_publisher(Float32MultiArray, '~/xy_dir', 10)
        self.pub_annot = self.create_publisher(Image, '~/annotated', 10)

        # subscriber — use sensor-data QoS for the camera
        self.sub_img  = self.create_subscription(Image, self.rgb_topic, self._img_cb, qos_profile_sensor_data)

        # model
        self.model = RFDETRLarge(pretrain_weights=self.weights)
        if hasattr(self.model, "optimize_for_inference"):
            self.model.optimize_for_inference()

        # Torch niceties when applicable (don’t force CUDA if wrapper isn't nn.Module)
        if HAVE_TORCH:
            try:
                if isinstance(self.model, torch.nn.Module):
                    self.model.eval()
                    if hasattr(torch.backends, "cudnn"):
                        torch.backends.cudnn.benchmark = True
                    if torch.cuda.is_available():
                        try:
                            self.model.to("cuda")
                            self.get_logger().info("⚡ Using CUDA for inference")
                        except Exception as e:
                            self.get_logger().warning(f"CUDA move failed (nn.Module), staying on CPU: {e}")
                else:
                    self.get_logger().info("Model wrapper is not a torch.nn.Module — running on CPU.")
            except Exception as e:
                self.get_logger().warning(f"Torch setup skipped: {e}")

        self.box_annotator = None
        self._frame_ctr = 0
        self._last_detections = sv.Detections(xyxy=np.empty((0, 4), dtype=np.float32))
        self._prev_dir = (None, None, "init")
        self._last_log_time = 0.0

        self.get_logger().info(
            f"🧠 RFDETR loaded | tol={self.tol_px} | frame_skip={self.frame_skip} | "
            f"infer_scale={self.infer_scale} | swap_xy={self.swap_xy} | "
            f"invert_x={self.invert_x} | invert_y={self.invert_y} | "
            f"step_mag_x={self.step_mag_x} | step_mag_y={self.step_mag_y}"
        )

    # ---------------- callbacks ----------------

    def _img_cb(self, msg: Image):
        bgr = self._image_msg_to_bgr(msg)
        self._handle_frame(msg.header, bgr)

    def _image_msg_to_bgr(self, msg: Image) -> np.ndarray:
        h, w, step = int(msg.height), int(msg.width), int(msg.step)
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        row = buf.reshape(h, step)
        # Assuming interleaved 3-channel (bgr8/rgb8); we treat as BGR in OpenCV.
        return row[:, : w * 3].reshape(h, w, 3)

    # ---------------- inference ----------------

    def _run_inference(self, bgr: np.ndarray) -> sv.Detections:
        """
        Run the model on (optionally) downscaled image and rescale detections back.
        """
        scale = float(self.infer_scale) if self.infer_scale > 0 else 1.0
        img_for_model = bgr
        if scale != 1.0:
            img_for_model = cv2.resize(
                bgr, None, fx=scale, fy=scale, interpolation=cv2.INTER_LINEAR
            )

        # Convert only when running inference
        rgb = cv2.cvtColor(img_for_model, cv2.COLOR_BGR2RGB)
        pil = PILImage.fromarray(rgb)

        ctx = torch.inference_mode() if HAVE_TORCH else nullcontext()
        with ctx:
            det_small = self.model.predict(pil, threshold=self.threshold)

        if len(det_small) == 0 or scale == 1.0:
            return det_small

        # Rescale boxes back to original coordinates
        xyxy = np.asarray(det_small.xyxy, dtype=np.float32) / scale

        # Preserve optional fields when present
        kwargs = {}
        for k in ("class_id", "confidence", "tracker_id", "data"):
            if hasattr(det_small, k):
                kwargs[k] = getattr(det_small, k)
        det = sv.Detections(xyxy=xyxy, **kwargs)
        return det

    # ---------------- core ----------------

    def _handle_frame(self, header: Header, bgr: np.ndarray):
        H, W = bgr.shape[:2]
        if self.box_annotator is None:
            self._thickness = sv.calculate_optimal_line_thickness((W, H))
            self.box_annotator = sv.BoxAnnotator(
                color=sv.ColorPalette.from_hex(["#ffff00"]),
                thickness=self._thickness
            )

        # --- frame skipping: run detector only every (frame_skip + 1) frames ---
        run_detector = True
        if self.frame_skip > 0:
            run_detector = (self._frame_ctr % (self.frame_skip + 1) == 0)
        self._frame_ctr += 1

        if run_detector:
            detections = self._run_inference(bgr)
            self._last_detections = detections
        else:
            detections = self._last_detections

        # --- annotate & compute directions every frame (cheap path) ---
        annotated = bgr.copy()
        if len(detections) > 0:
            annotated = self.box_annotator.annotate(annotated, detections)

        # crosshair + tolerance ring
        draw_crosshair(annotated, self.cross_x, self.cross_y,
                       size=self.cross_len, thickness=self.cross_thk)
        cv2.circle(annotated, (self.cross_x, self.cross_y),
                   max(1, self.tol_px), (255, 255, 0), 2, cv2.LINE_AA)

        x_dir = y_dir = 0
        status = "no_detection"
        dx = dy = 0
        if len(detections) > 0:
            det_cx, det_cy, _ = nearest_detection_center(detections, self.cross_x, self.cross_y)
            if det_cx is not None:
                cv2.line(annotated, (det_cx, det_cy), (self.cross_x, self.cross_y), (0, 0, 255), self.line_thk)
                dx, dy = det_cx - self.cross_x, det_cy - self.cross_y
                if abs(dx) <= self.tol_px and abs(dy) <= self.tol_px:
                    status = "aligned"
                else:
                    x_dir = 1 if dx > self.tol_px else (-1 if dx < -self.tol_px else 0)
                    y_dir = 1 if dy > self.tol_px else (-1 if dy < -self.tol_px else 0)
                    status = "seeking"

        # Apply swaps/inversions
        if self.swap_xy:
            x_dir, y_dir = y_dir, x_dir
            dx, dy = dy, dx
        if self.invert_x:
            x_dir = -x_dir
        if self.invert_y:
            y_dir = -y_dir

        # Scale to float outputs (tunable magnitudes)
        x_cmd = float(x_dir) * self.step_mag_x
        y_cmd = float(y_dir) * self.step_mag_y

        # publish directions (~/xy_dir) — Float32MultiArray
        dir_msg = Float32MultiArray()
        dir_msg.data = [x_cmd, y_cmd]
        self.pub_dir.publish(dir_msg)

        now = time.time()
        if (x_cmd, y_cmd, status) != self._prev_dir or (now - self._last_log_time) > 1.0:
            self.get_logger().info(
                f"[dir] x_cmd={x_cmd:.3f} y_cmd={y_cmd:.3f} | dx={dx} dy={dy} | tol={self.tol_px} | {status}"
            )
            self._prev_dir = (x_cmd, y_cmd, status)
            self._last_log_time = now

        draw_axes_top_left(annotated)
        self._publish_bgr_as_image(header, annotated)

    def _publish_bgr_as_image(self, header: Header, bgr: np.ndarray):
        msg = Image()
        msg.header = header
        msg.height, msg.width = bgr.shape[:2]
        msg.encoding = 'bgr8'
        msg.step = msg.width * 3
        msg.data = bgr.tobytes()
        # same topic name: ~/annotated — DEFAULT QoS (RELIABLE)
        self.pub_annot.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ToolCamRFDETR()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
