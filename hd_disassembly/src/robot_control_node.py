#!/usr/bin/env python3
"""
robot_control_step8_dual.py
Step 8 — Full dual-arm flow with TOOL XY auto-align + peck + unscrew, and MANIP pick flow.

Updates in this version (per your request):
- TOOL probing remains POSITION-STEP based and can stop on τ3 or xy_dir non-zero (unchanged from your last file).
- MANIP probing is reverted to the OLD VELOCITY-CONTROL method: descend until τ3 ≥ probe threshold.
  * No "max depth" limit applies to MANIP probing.
- All other logic (peck-first-then-nudge, τ1/τ3 pre-moves, unscrew routine, parking, return TOOL to START) is unchanged.
"""

import os
import time
import math
import json
import yaml
from typing import Optional, List, Dict, Any, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory

from xarm.wrapper import XArmAPI

from hd_disassembly.srv import GetPlan, TransformPoint
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32MultiArray, Int8


# ---------------- helpers ----------------
def quaternion_to_euler(x, y, z, w):
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


def _pose_from_cfg(block, key):
    p = block[key]["position"]
    q = block[key]["orientation"]
    pos_m = (float(p["x"]), float(p["y"]), float(p["z"]))
    quat = (float(q["x"]), float(q["y"]), float(q["z"]), float(q["w"]))
    return pos_m, quat


# ---------------- node ----------------
class Step8Control(Node):
    def __init__(self):
        super().__init__("dual_arm_step8")
        self.get_logger().info("🤖 Step 8: dual-arm (TOOL align+peck+unscrew, MANIP pick) — TOOL step-probe; MANIP VC-probe")

        # --- load YAML ---
        pkg_share = get_package_share_directory("hd_disassembly")
        cfg_path = os.path.join(pkg_share, "config", "disassembler_params.yaml")
        with open(cfg_path, "r") as f:
            cfg = yaml.safe_load(f) or {}

        rcfg = cfg.get("robot", {})
        vcfg = cfg.get("vision", {})
        cam = vcfg.get("camera", {})
        tcfg = rcfg.get("tool", {})
        mcfg = rcfg.get("manip", {})

        # --- IPs / frame ---
        self.declare_parameter("robot.tool.ip", tcfg.get("ip", "192.168.1.239"))
        self.declare_parameter("robot.manip.ip", mcfg.get("ip", "192.168.1.195"))
        tool_ip = str(self.get_parameter("robot.tool.ip").value)
        manip_ip = str(self.get_parameter("robot.manip.ip").value)

        self.declare_parameter("vision.frame_id", cam.get("frame_id", "R_camera_color_optical_frame"))
        self.vision_frame = str(self.get_parameter("vision.frame_id").value)

        # --- poses ---
        tool_start_pos, tool_start_q = _pose_from_cfg(tcfg, "start_pose")
        tool_ideal_pos, tool_ideal_q = _pose_from_cfg(tcfg, "ideal_pose")
        manip_start_pos, manip_start_q = _pose_from_cfg(mcfg, "start_pose")
        manip_ideal_pos, manip_ideal_q = _pose_from_cfg(mcfg, "ideal_pose")

        self.tool_start_m = tool_start_pos
        self.tool_start_rpy = quaternion_to_euler(*tool_start_q)
        self.tool_ideal_m = tool_ideal_pos
        self.tool_rpy = quaternion_to_euler(*tool_ideal_q)

        self.manip_start_m = manip_start_pos
        self.manip_ideal_m = manip_ideal_pos
        self.manip_rpy = quaternion_to_euler(*manip_ideal_q)

        # --- motion & probing thresholds ---
        stop_h_m = float(rcfg.get("stop_height", 0.10))       # for MANIP hover-above
        probe_mps = float(rcfg.get("probing_speed", 0.0255))  # m/s
        thr = rcfg.get("effort_threshold", {"probing": -6.0, "unscrew": -6.0})
        probe_thresh = float(thr.get("probing", -6.0))
        unscrew_thresh = float(thr.get("unscrew", -6.0))

        self.declare_parameter("robot.move_speed_mm_s", 50.0)
        self.declare_parameter("robot.stop_height_mm", stop_h_m * 1000.0)
        self.declare_parameter("robot.probe_speed_mm_s", probe_mps * 1000.0)  # used for retract speeds etc.
        self.declare_parameter("robot.probe_thresh_nm", probe_thresh)
        self.declare_parameter("robot.unscrew_thresh_nm", unscrew_thresh)
        self.declare_parameter("robot.lift_after_grip_mm", 20.0)

        # Behavior toggles (default: keep other arm at IDEAL)
        self.declare_parameter("park_other_arm_home", False)
        self.declare_parameter("manip.hover_only", False)

        # Per-arm probing signs
        self.declare_parameter("probe.prefer_tool_frame", True)
        self.declare_parameter("probe.tool_z_sign", 1)
        self.declare_parameter("probe.manip_z_sign", 1)
        self.declare_parameter("probe.base_z_sign", 1)
        self.declare_parameter("probe.retract_after_mm", 0.0)

        # TOOL probing (position-step) params
        self.declare_parameter("probe.step_mm", 20.0)
        self.declare_parameter("probe.wait_after_step_s", 1.0)
        self.declare_parameter("probe.max_total_mm", 60.0)

        # TOOL hover fixed Z options
        self.declare_parameter("hover.fixed_z_mm.tool", 30.0)
        self.declare_parameter("hover.fixed_z_mm.manip", 200.0)

        # Alignment (TOOL)
        self.declare_parameter("detector.xy_topic", "/tool_cam_rfdetr/xy_dir")
        self.declare_parameter("align.speed_mm_s", 2.0)
        self.declare_parameter("align.zero_hold_updates", 3)
        self.declare_parameter("align.dir_timeout_s", 2.0)
        self.declare_parameter("align.stale_limit", 10)
        self.declare_parameter("align.max_total_mm", 150.0)

        # Peck (TOOL)
        self.declare_parameter("align.peck_step_mm", 1.0)
        self.declare_parameter("align.peck_speed_mm_s", 5.0)
        self.declare_parameter("align.peck_max_total_mm", 30.0)
        self.declare_parameter("align.peck_pause_s", 0.05)
        self.declare_parameter("align.j1_stop_nm", -4.0)  # stop peck if τ1 <= this

        # Nudge-on-zero (TOOL)
        self.declare_parameter("align.nudge_step_mm", -0.5)   # –Y
        self.declare_parameter("align.nudge_speed_mm_s", 4.0)
        self.declare_parameter("align.nudge_max_total_mm", 5.0)

        # Unscrew pre-moves
        self.declare_parameter("unscrew.pre_retract_z_mm", 0.5)     # when τ3 triggers
        self.declare_parameter("unscrew.pre_shift_y_mm", 0.5)       # when τ1 triggers (+Y)
        self.declare_parameter("unscrew.pre_move_speed_mm_s", 5.0)

        # Read params
        self.move_speed = float(self.get_parameter("robot.move_speed_mm_s").value)
        self.stop_height_mm = float(self.get_parameter("robot.stop_height_mm").value)
        self.probe_speed_mm = float(self.get_parameter("robot.probe_speed_mm_s").value)
        self.probe_thresh_nm = float(self.get_parameter("robot.probe_thresh_nm").value)
        self.unscrew_thresh = float(self.get_parameter("robot.unscrew_thresh_nm").value)
        self.lift_after_grip = float(self.get_parameter("robot.lift_after_grip_mm").value)

        self.park_other_home = bool(self.get_parameter("park_other_arm_home").value)
        self.manip_hover_only = bool(self.get_parameter("manip.hover_only").value)

        self.prefer_tool_frame = bool(self.get_parameter("probe.prefer_tool_frame").value)
        self.tool_z_sign = int(self.get_parameter("probe.tool_z_sign").value)
        self.manip_z_sign = int(self.get_parameter("probe.manip_z_sign").value)
        self.base_z_sign = int(self.get_parameter("probe.base_z_sign").value)
        self.retract_after_mm = float(self.get_parameter("probe.retract_after_mm").value)

        self.probe_step_mm = float(self.get_parameter("probe.step_mm").value)
        self.probe_wait_after_step_s = float(self.get_parameter("probe.wait_after_step_s").value)
        self.probe_max_total_mm = float(self.get_parameter("probe.max_total_mm").value)

        self.hover_z_tool_mm = float(self.get_parameter("hover.fixed_z_mm.tool").value)
        self.hover_z_manip_mm = float(self.get_parameter("hover.fixed_z_mm.manip").value)

        self.xy_dir_topic = str(self.get_parameter("detector.xy_topic").value)
        self.align_speed = float(self.get_parameter("align.speed_mm_s").value)
        self.zero_hold_updates = int(self.get_parameter("align.zero_hold_updates").value)
        self.dir_timeout_s = float(self.get_parameter("align.dir_timeout_s").value)
        self.stale_limit = int(self.get_parameter("align.stale_limit").value)
        self.max_total_mm = float(self.get_parameter("align.max_total_mm").value)

        self.peck_step_mm = float(self.get_parameter("align.peck_step_mm").value)
        self.peck_speed_mm_s = float(self.get_parameter("align.peck_speed_mm_s").value)
        self.peck_max_total_mm = float(self.get_parameter("align.peck_max_total_mm").value)
        self.peck_pause_s = float(self.get_parameter("align.peck_pause_s").value)
        self.j1_stop_nm = float(self.get_parameter("align.j1_stop_nm").value)

        self.nudge_step_mm = float(self.get_parameter("align.nudge_step_mm").value)
        self.nudge_speed_mm_s = float(self.get_parameter("align.nudge_speed_mm_s").value)
        self.nudge_max_total_mm = float(self.get_parameter("align.nudge_max_total_mm").value)

        self.unscrew_pre_retract_z_mm = float(self.get_parameter("unscrew.pre_retract_z_mm").value)
        self.unscrew_pre_shift_y_mm = float(self.get_parameter("unscrew.pre_shift_y_mm").value)
        self.unscrew_pre_move_speed = float(self.get_parameter("unscrew.pre_move_speed_mm_s").value)

        self.get_logger().info(
            f"""
🧭 IPs: TOOL={tool_ip} | MANIP={manip_ip} | vision_frame={self.vision_frame}
⚙️ speeds: move={self.move_speed:.1f} mm/s
   TOOL probe: step={self.probe_step_mm:.1f} mm, wait={self.probe_wait_after_step_s:.1f}s, max_total={self.probe_max_total_mm:.1f} mm
   MANIP probe: velocity-control until τ3≥{self.probe_thresh_nm:.2f} (no max-depth)
🧱 thresholds: probe τ3≥{self.probe_thresh_nm:.2f}, j1_stop≤{self.j1_stop_nm:.2f}, unscrew τ3≥{self.unscrew_thresh:.2f}
🅿️ other arm parked: {"HOME" if self.park_other_home else "IDEAL"}
TOOL hoverZ={self.hover_z_tool_mm:.1f} mm | MANIP hoverZ={self.hover_z_manip_mm:.1f} mm
XY topic={self.xy_dir_topic} | align v={self.align_speed:.1f} | peck step={self.peck_step_mm:.2f} @ {self.peck_speed_mm_s:.1f}
"""
        )

        # --- connect arms ---
        self.arm_tool = self._connect_arm(tool_ip, "TOOL")
        self.arm_manip = self._connect_arm(manip_ip, "MANIP")

        # screwdriver topic
        self.tool_pub = self.create_publisher(Int8, "/tool_cmd", 10)

        # --- services ---
        self.plan_cli = self.create_client(GetPlan, "get_plan")
        self.tf_cli_tool = self.create_client(TransformPoint, "transform_point_tool")
        self.tf_cli_man = self.create_client(TransformPoint, "transform_point_manip")
        for name, cli in [
            ("get_plan", self.plan_cli),
            ("transform_point_tool", self.tf_cli_tool),
            ("transform_point_manip", self.tf_cli_man),
        ]:
            while not cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn(f"⏳ Waiting for {name}…")
            self.get_logger().info(f"✅ {name} ready")

        # --- XY dir subscriber (TOOL alignment) ---
        self._dx_mm: float = 0.0
        self._dy_mm: float = 0.0
        self._xy_count: int = 0
        self.create_subscription(Float32MultiArray, self.xy_dir_topic, self._xy_cb, qos_profile_sensor_data)

        # --- Kickoff: TOOL to START, MANIP to IDEAL ---
        self._go_pose(self.arm_tool, self.tool_start_m, self.tool_start_rpy, "TOOL")
        self._go_pose(self.arm_manip, self.manip_ideal_m, self.manip_rpy, "MANIP")

        # --- run pipeline across full plan ---
        try:
            plan = self._fetch_plan()
            fixed = self._transform_and_print(plan)
            self._run_sequence(fixed)
            self.get_logger().info("✅ Disassembly plan complete.")
        except Exception as e:
            self.get_logger().error(f"❌ Step error: {e}")

    # ---------------- xy_dir callback ----------------
    def _xy_cb(self, msg: Float32MultiArray):
        data = list(msg.data or [])
        dx = float(data[0]) if len(data) >= 1 else 0.0
        dy = float(data[1]) if len(data) >= 2 else 0.0
        self._dx_mm = dx
        self._dy_mm = dy
        self._xy_count += 1
        if self._xy_count <= 5 or abs(dx) > 0.0 or abs(dy) > 0.0:
            self.get_logger().info(f"[{self.xy_dir_topic}] Δ(mm)=({dx:.3f},{dy:.3f}) | count={self._xy_count}")

    # ---------------- torque helper ----------------
    def _read_taus(self, arm: XArmAPI) -> Tuple[Optional[float], Optional[float]]:
        try:
            code, torq = arm.get_joints_torque()
            if code == 0 and isinstance(torq, (list, tuple)) and len(torq) >= 3:
                return float(torq[0]), float(torq[2])
        except Exception:
            pass
        return None, None

    # ---------------- plan helpers ----------------
    def _fetch_plan(self) -> List[Dict[str, Any]]:
        req = GetPlan.Request()
        req.mode = "snapshot"
        req.dedup_tol_m = 0.015
        fut = self.plan_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        res = fut.result()
        if not res:
            raise RuntimeError("get_plan returned no response")
        if not getattr(res, "ok", False):
            raise RuntimeError(f"get_plan failed: {getattr(res, 'message', '(no message)')}")
        plan = json.loads(res.plan_json)
        if not plan:
            self.get_logger().warn("ℹ️ Plan is empty.")
        else:
            self.get_logger().info("🧾 DISASSEMBLY SEQUENCE (snapshot):")
            for i, item in enumerate(plan, start=1):
                key = item.get("key", "?")
                label = item.get("label", "unknown")
                arm = item.get("arm", "?")
                prio = item.get("priority", 0)
                approx = item.get("approx", {}) or {}
                ax = approx.get("x", float("nan"))
                ay = approx.get("y", float("nan"))
                az = approx.get("z", float("nan"))
                self.get_logger().info(
                    f"  {i:02d}. key={key:<16} label={label:<12} arm={arm:<5} prio={prio:<3} approx(m)=({ax:.4f},{ay:.4f},{az:.4f})"
                )
        return plan

    def _transform_and_print(self, plan_list: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        if not plan_list:
            return []
        fixed: List[Dict[str, Any]] = []
        for item in plan_list:
            label = item.get("label", "unknown")
            which = item.get("arm", "manip")
            approx = item.get("approx", None) or {}
            key = item.get("key", "?")
            if not all(k in approx for k in ("x", "y", "z")):
                self.get_logger().warn(f"⛔ Missing approx for {key} ({label}); skipping")
                continue

            ps = PointStamped()
            ps.header.frame_id = self.vision_frame
            ps.point.x = float(approx["x"])
            ps.point.y = float(approx["y"])
            ps.point.z = float(approx["z"])

            tp_req = TransformPoint.Request()
            tp_req.point_in = ps
            cli = self.tf_cli_tool if which == "tool" else self.tf_cli_man
            fut_tf = cli.call_async(tp_req)
            rclpy.spin_until_future_complete(self, fut_tf)
            tf_res = fut_tf.result()
            if not tf_res:
                self.get_logger().warn(f"⛔ transform_point failed for {key} ({label}); skipping")
                continue

            out = tf_res.point_out
            fixed.append(
                {
                    "key": key,
                    "label": label,
                    "arm": which,
                    "approx_frame": self.vision_frame,
                    "approx": {"x": ps.point.x, "y": ps.point.y, "z": ps.point.z},
                    "tf_frame": out.header.frame_id or "(unknown_frame)",
                    "target": {"x": out.point.x, "y": out.point.y, "z": out.point.z},
                    "priority": item.get("priority", 0),
                }
            )

        if not fixed:
            self.get_logger().warn("⛔ No valid transformed targets.")
            return []

        self.get_logger().info("🧊 FROZEN TARGETS (transformed):")
        for i, it in enumerate(fixed, start=1):
            ax, ay, az = it["approx"]["x"], it["approx"]["y"], it["approx"]["z"]
            tx, ty, tz = it["target"]["x"], it["target"]["y"], it["target"]["z"]
            self.get_logger().info(
                f"  {i:02d}. {it['label']:<12} arm={it['arm']:<5} | approx({it['approx_frame']}): "
                f"{ax:.4f},{ay:.4f},{az:.4f}  →  {it['tf_frame']}: {tx:.4f},{ty:.4f},{tz:.4f}"
            )
        return fixed

    # ---------------- sequence runner ----------------
    def _run_sequence(self, fixed: List[Dict[str, Any]]):
        if not fixed:
            self.get_logger().warn("ℹ️ Nothing to do.")
            return

        for i, it in enumerate(fixed, start=1):
            label = it["label"]
            which = it["arm"]
            basep = it["target"]
            basef = it["tf_frame"]
            self.get_logger().info(
                f"🔀 [{i}/{len(fixed)}] {it['key']} → {label}, arm={which} | "
                f"target {basef}: {basep['x']:.4f},{basep['y']:.4f},{basep['z']:.4f}"
            )

            if which == "tool" and label in ("screw_head", "screw"):
                # Park MANIP; keep TOOL at START until hover
                self._park_other("tool")

                # Hover above target for TOOL (fixed Z)
                z_m = self.hover_z_tool_mm / 1000.0
                pos_m = (float(basep["x"]), float(basep["y"]), z_m)
                self._go_pose(self.arm_tool, pos_m, self.tool_rpy, "TOOL")

                # PROBE (stepped) to τ3 or XY non-zero, then retract configurable height
                reason = self._probe_step_until_contact_or_xy(self.arm_tool, "TOOL")
                self.get_logger().info(f"🧪 Probe stop reason: {reason}")
                if self.retract_after_mm > 0:
                    self._retract_mm(self.arm_tool, "TOOL", self.retract_after_mm, speed=self.probe_speed_mm)

                # Align at this height
                self._align_tool_from_topic(wait_initial=True)

                # Peck with alignment until stop; returns (contact: bool, reason: "tau3"|"tau1"|"max_depth")
                contact, reason = self._peck_down_with_alignment_until_contact()
                if not contact:
                    self.get_logger().warn("⚠️ Peck ended without torque trigger; skipping unscrew for this target.")
                else:
                    # Pre-move based on reason
                    if reason == "tau3":
                        self.arm_tool.set_tool_position(
                            x=0, y=0, z=self._up_step("TOOL", self.unscrew_pre_retract_z_mm),
                            speed=self.unscrew_pre_move_speed, wait=True
                        )
                        self.get_logger().info(f"↕️ Pre-unscrew Z retract {self.unscrew_pre_retract_z_mm:.3f} mm (τ3 trigger)")
                    elif reason == "tau1":
                        self.arm_tool.set_position(
                            x=0.0, y=+abs(self.unscrew_pre_shift_y_mm), z=0.0,
                            roll=0.0, pitch=0.0, yaw=0.0,
                            speed=self.unscrew_pre_move_speed, is_radian=True,
                            wait=True, relative=True
                        )
                        self.get_logger().info(f"↔️ Pre-unscrew +Y shift {self.unscrew_pre_shift_y_mm:.3f} mm (τ1 trigger)")

                    # Unscrew routine (publish /tool_cmd=-1 and retract 1 mm on high torque)
                    self._unscrew_routine(self.arm_tool, tag="TOOL")

                # Big retract + return to START
                self._retract_mm(self.arm_tool, "TOOL", 100.0, speed=self.probe_speed_mm)
                self._go_pose(self.arm_tool, self.tool_start_m, self.tool_start_rpy, "TOOL")

            else:
                # MANIP item
                self._park_other("manip")

                # Move MANIP to START first
                self._go_pose(self.arm_manip, self.manip_start_m, self.manip_rpy, "MANIP")

                # Hover above target (use stop_height_mm offset)
                r, p, yaw = self.manip_rpy
                x_mm = float(basep["x"]) * 1000.0
                y_mm = float(basep["y"]) * 1000.0
                z_mm = float(basep["z"]) * 1000.0 + self.stop_height_mm
                self.arm_manip.set_position(x_mm, y_mm, z_mm, r, p, yaw,
                                            speed=self.move_speed, is_radian=True, wait=True)
                self.get_logger().info(f"🛬 MANIP hover @ ({x_mm:.1f},{y_mm:.1f},{z_mm:.1f}) mm")

                if not self.manip_hover_only:
                    # MANIP probe (OLD VC: descend until τ3 ≥ probe_thresh_nm, no max-depth)
                    self._probe_until_contact(self.arm_manip, "MANIP", timeout_s=10.0, use_tool_coord=True)

                    self.arm_manip.set_tool_position(
                        x=0, y=0, z=self._up_step("MANIP", 2.0),
                        speed=self.probe_speed_mm, wait=True
                    )
                    self.get_logger().info("↕️ MANIP retract 2.0 mm")

                    self.get_logger().info("🧲 MANIP vacuum ON")
                    try:
                        self.arm_manip.set_suction_cup(True, wait=True)
                    except Exception as e:
                        self.get_logger().warn(f"Vacuum ON error: {e}")

                    self.get_logger().info(f"⬆️ MANIP lift {self.lift_after_grip:.1f} mm")
                    self.arm_manip.set_tool_position(
                        x=0, y=0, z=self._up_step("MANIP", self.lift_after_grip),
                        speed=self.probe_speed_mm, wait=True
                    )

                    self._retract_mm(self.arm_manip, "MANIP", 100.0, speed=self.probe_speed_mm)

                # Park MANIP to IDEAL at the end of its action
                self._go_pose(self.arm_manip, self.manip_ideal_m, self.manip_rpy, "MANIP")

        # Final park: TOOL to START, MANIP to IDEAL
        self.get_logger().info("🎉 Sequence finished → TOOL to START, MANIP to IDEAL")
        self._go_pose(self.arm_tool, self.tool_start_m, self.tool_start_rpy, "TOOL")
        self._go_pose(self.arm_manip, self.manip_ideal_m, self.manip_rpy, "MANIP")

    # ---------------- parking helpers ----------------
    def _park_other(self, which_active: str):
        if which_active == "tool":
            if self.park_other_home:
                self._go_home(self.arm_manip, "MANIP")
            else:
                self._go_pose(self.arm_manip, self.manip_ideal_m, self.manip_rpy, "MANIP")
        else:
            if self.park_other_home:
                self._go_home(self.arm_tool, "TOOL")
            else:
                self._go_pose(self.arm_tool, self.tool_ideal_m, self.tool_rpy, "TOOL")

    # ---------------- TOOL alignment using xy_dir ----------------
    def _wait_for_xy_initial(self, timeout_s: float) -> bool:
        t_end = time.time() + float(max(0.0, timeout_s))
        start = self._xy_count
        while time.time() < t_end and self._xy_count == start:
            rclpy.spin_once(self, timeout_sec=0.05)
        return self._xy_count > start

    def _wait_for_xy_update(self, prev_count: int, timeout_s: float) -> bool:
        t_end = time.time() + float(max(0.0, timeout_s))
        while time.time() < t_end and self._xy_count == prev_count:
            rclpy.spin_once(self, timeout_sec=0.05)
        return self._xy_count > prev_count

    def _align_tool_from_topic(self, *, wait_initial: bool):
        if wait_initial and not self._wait_for_xy_initial(timeout_s=5.0):
            self.get_logger().warn(f"Alignment skipped: no messages on '{self.xy_dir_topic}' within 5s.")
            return

        self.get_logger().info(
            f"🎥 XY align: v={self.align_speed:.1f} mm/s | zero_hold={self.zero_hold_updates} | "
            f"timeout={self.dir_timeout_s:.1f}s | stale_limit={self.stale_limit} | max_XY={self.max_total_mm:.1f} mm"
        )

        zero_streak = 0
        total_travel = 0.0
        stale_left = int(max(0, self.stale_limit))
        prev_count = self._xy_count

        try:
            self.arm_tool.motion_enable(True); time.sleep(0.02)
            self.arm_tool.set_mode(0); time.sleep(0.02)
            self.arm_tool.set_state(0); time.sleep(0.02)
        except Exception as e:
            self.get_logger().warn(f"[align] prep raised: {e}")

        while True:
            if not self._wait_for_xy_update(prev_count, timeout_s=self.dir_timeout_s):
                stale_left -= 1
                if stale_left <= 0:
                    self.get_logger().warn("No fresh xy_dir updates; stopping alignment.")
                    break
                self.get_logger().warn(f"No fresh xy_dir; continuing (stale_left={stale_left}).")
                continue
            prev_count = self._xy_count

            dx_mm = float(self._dx_mm)
            dy_mm = float(self._dy_mm)

            if dx_mm == 0.0 and dy_mm == 0.0:
                zero_streak += 1
                self.get_logger().info(f"🎯 zeros ({zero_streak}/{self.zero_hold_updates})")
                if zero_streak >= self.zero_hold_updates:
                    self.get_logger().info("✅ XY alignment completed (received zeros).")
                    break
                continue
            zero_streak = 0

            step_sum = abs(dx_mm) + abs(dy_mm)
            if (total_travel + step_sum) > self.max_total_mm:
                self.get_logger().warn("XY travel cap reached; stopping alignment.")
                break

            try:
                code = self.arm_tool.set_position(
                    x=dx_mm, y=dy_mm, z=0.0,
                    roll=0.0, pitch=0.0, yaw=0.0,
                    speed=self.align_speed, is_radian=True,
                    wait=True, relative=True
                )
                if code != 0:
                    self.get_logger().warn(
                        f"[align] set_position code={code} (err={self.arm_tool.error_code}, warn={self.arm_tool.warn_code})"
                    )
                else:
                    self.get_logger().info(f"[align] moved Δ(mm)=({dx_mm:.3f},{dy_mm:.3f}) @ {self.align_speed:.1f} mm/s")
            except Exception as e:
                self.get_logger().warn(f"[align] set_position raised: {e}")
                break

            total_travel += step_sum

    # ---------------- nudge-on-zero helper ----------------
    def _nudge_until_nonzero(self) -> bool:
        if not (self._dx_mm == 0.0 and self._dy_mm == 0.0):
            return True

        total = 0.0
        prev_count = self._xy_count
        self.get_logger().info(
            f"🟡 Nudge-on-zero: step={self.nudge_step_mm:.3f} mm in Y @ {self.nudge_speed_mm_s:.1f} mm/s "
            f"(max {self.nudge_max_total_mm:.1f} mm) until detector non-zero."
        )
        try:
            self.arm_tool.motion_enable(True); time.sleep(0.01)
            self.arm_tool.set_mode(0); time.sleep(0.01)
            self.arm_tool.set_state(0); time.sleep(0.01)
        except Exception as e:
            self.get_logger().warn(f"[nudge] prep raised: {e}")

        while total + abs(self.nudge_step_mm) <= self.nudge_max_total_mm:
            try:
                code = self.arm_tool.set_position(
                    x=0.0, y=self.nudge_step_mm, z=0.0,
                    roll=0.0, pitch=0.0, yaw=0.0,
                    speed=self.nudge_speed_mm_s, is_radian=True,
                    wait=True, relative=True
                )
                if code != 0:
                    self.get_logger().warn(
                        f"[nudge] set_position code={code} (err={self.arm_tool.error_code}, warn={self.arm_tool.warn_code})"
                    )
            except Exception as e:
                self.get_logger().warn(f"[nudge] step raised: {e}")
                break

            total += abs(self.nudge_step_mm)

            if self._wait_for_xy_update(prev_count, timeout_s=self.dir_timeout_s):
                prev_count = self._xy_count
                if not (self._dx_mm == 0.0 and self._dy_mm == 0.0):
                    self.get_logger().info("🟢 Detector non-zero after nudge; resuming alignment.")
                    return True
            else:
                self.get_logger().warn("[nudge] No detector update; continuing.")

        self.get_logger().warn("⚠️ Nudge max distance reached with detector zero.")
        return False

    # ---------------- peck-down loop (stops on τ3 or τ1) ----------------
    def _peck_down_with_alignment_until_contact(self) -> Tuple[bool, str]:
        self.get_logger().info(
            f"🔁 Peck-down: step={self.peck_step_mm:.2f} mm @ {self.peck_speed_mm_s:.1f} mm/s, "
            f"max_total_z={self.peck_max_total_mm:.1f} mm; aligning each level (nudge on zero)."
        )

        total_z = 0.0
        s_down = self._arm_z_sign("TOOL") * abs(self.peck_step_mm)

        try:
            self.arm_tool.motion_enable(True); time.sleep(0.02)
            self.arm_tool.set_mode(0); time.sleep(0.02)
            self.arm_tool.set_state(0); time.sleep(0.02)
        except Exception as e:
            self.get_logger().warn(f"[peck] prep raised: {e}")

        while total_z + abs(self.peck_step_mm) <= self.peck_max_total_mm:
            # Z peck
            try:
                code = self.arm_tool.set_tool_position(
                    x=0, y=0, z=s_down, speed=self.peck_speed_mm_s, wait=True
                )
                if code != 0:
                    self.get_logger().warn(
                        f"[peck] set_tool_position code={code} (err={self.arm_tool.error_code}, warn={self.arm_tool.warn_code})"
                    )
            except Exception as e:
                self.get_logger().warn(f"[peck] Z step raised: {e}")
                break

            total_z += abs(self.peck_step_mm)
            self.get_logger().info(f"[peck] Z total descent = {total_z:.2f} mm")
            time.sleep(self.peck_pause_s)

            # Torque checks
            tau1, tau3 = self._read_taus(self.arm_tool)
            self.get_logger().info(f"[peck] τ₁={tau1 if tau1 is not None else 'NaN'} Nm, τ₃={tau3 if tau3 is not None else 'NaN'} Nm")

            if tau3 is not None and tau3 >= self.probe_thresh_nm:
                self.get_logger().info("✅ Peck stop: τ3 ≥ probe threshold.")
                return True, "tau3"
            if tau1 is not None and tau1 <= self.j1_stop_nm:
                self.get_logger().info("✅ Peck stop: τ1 ≤ stop threshold.")
                return True, "tau1"

            # Align at this depth
            self._align_tool_from_topic(wait_initial=False)

            # Nudge if still (0,0), then quick align again
            if self._dx_mm == 0.0 and self._dy_mm == 0.0:
                if self._nudge_until_nonzero():
                    self._align_tool_from_topic(wait_initial=False)

        self.get_logger().warn("Peck-down reached max Z descent without torque condition.")
        return False, "max_depth"

    # ---------------- PROBING helpers ----------------
    def _arm_z_sign(self, tag: str) -> int:
        return self.tool_z_sign if tag.upper() == "TOOL" else self.manip_z_sign

    def _down_vec_tool(self, tag: str, v_mm_s: float):
        s = self._arm_z_sign(tag)
        return [0, 0, s * abs(v_mm_s), 0, 0, 0]

    def _down_vec_base(self, v_mm_s: float):
        s = self.base_z_sign
        return [0, 0, s * abs(v_mm_s), 0, 0, 0]

    def _up_step(self, tag: str, step_mm: float) -> float:
        return -self._arm_z_sign(tag) * abs(step_mm)

    def _retract_mm(self, arm, tag: str, mm: float, speed: float = None):
        dz = self._up_step(tag, mm)
        try:
            arm.set_tool_position(x=0, y=0, z=dz, speed=(speed or self.probe_speed_mm), wait=True)
            self.get_logger().info(f"⬆️ {tag} retract {mm:.1f} mm (tool-Z)")
        except Exception as e:
            self.get_logger().warn(f"Retract failed for {tag}: {e}")

    # ---- TOOL: stepped set-position probing (unchanged) ----
    def _probe_step_until_contact_or_xy(self, arm: XArmAPI, tag: str) -> str:
        step_mm = abs(self.probe_step_mm)
        s_down = self._arm_z_sign(tag) * step_mm
        total = 0.0
        self.get_logger().info(
            f"🔬 {tag} stepped probe (TOOL): step={step_mm:.1f} mm, wait={self.probe_wait_after_step_s:.1f}s, max_total={self.probe_max_total_mm:.1f} mm"
        )

        while total + step_mm <= self.probe_max_total_mm:
            try:
                code = arm.set_tool_position(x=0, y=0, z=s_down, speed=self.probe_speed_mm, wait=True)
                if code != 0:
                    self.get_logger().warn(f"[probe] set_tool_position code={code} (err={arm.error_code}, warn={arm.warn_code})")
            except Exception as e:
                self.get_logger().warn(f"[probe] step move raised: {e}")
                break

            total += step_mm
            t0 = time.time()
            while (time.time() - t0) < self.probe_wait_after_step_s:
                rclpy.spin_once(self, timeout_sec=0.01)

                tau1, tau3 = self._read_taus(arm)
                if tau3 is not None and tau3 >= self.probe_thresh_nm:
                    self.get_logger().info(f"📏 {tag} probe stop: τ₃={tau3:.2f} Nm")
                    return "tau3"

                if (self._dx_mm != 0.0) or (self._dy_mm != 0.0):
                    self.get_logger().info("🎥 TOOL probe stop: xy_dir non-zero observed.")
                    return "xy"

                time.sleep(0.02)

        self.get_logger().warn("⛔ TOOL probe: max depth reached without trigger.")
        return "max_depth"

    # ---- MANIP: OLD VC probing until τ3 (no max-depth) ----
    def _fallback_probe(self, arm, tag, timeout_s=10.0, step_mm=0.5):
        t0 = time.time()
        self.get_logger().warn(f"🪤 {tag} VC probe unavailable → step-probe (Δz={step_mm} mm)")
        try:
            arm.set_mode(0); time.sleep(0.05)
            arm.set_state(0); time.sleep(0.05)
        except Exception:
            pass
        last_tau = None
        dz = self._arm_z_sign(tag) * abs(step_mm)
        while time.time() - t0 < timeout_s:
            code = arm.set_tool_position(
                x=0, y=0, z=dz, speed=max(5.0, min(self.probe_speed_mm, 50.0)), wait=True
            )
            if code != 0:
                self.get_logger().warn(f"⚠️ {tag} step move code={code}")
            code, torq = arm.get_joints_torque()
            if code == 0 and isinstance(torq, (list, tuple)) and len(torq) >= 3:
                tau = torq[2]; last_tau = tau
                if tau >= self.probe_thresh_nm:
                    self.get_logger().info(f"📏 {tag} contact τ₃={tau:.2f} Nm (step-probe)")
                    break
            time.sleep(0.02)
        return last_tau

    def _probe_until_contact(self, arm, tag, timeout_s=10.0, use_tool_coord=True):
        """
        Velocity-control probing: descend until τ3 ≥ probe threshold (no max-depth cap).
        Used for MANIP per your request.
        """
        vdown = float(max(1.0, min(self.probe_speed_mm, 60.0)))
        self.get_logger().info(
            f"🔬 {tag} VC probing until τ₃ ≥ {self.probe_thresh_nm:.2f} Nm (v={vdown:.1f} mm/s, tool_coord={use_tool_coord})"
        )
        # Enter VC
        try:
            try:
                if getattr(arm, "has_error", False): arm.clean_error()
                if getattr(arm, "has_warn",  False): arm.clean_warn()
            except Exception:
                pass
            arm.motion_enable(True); time.sleep(0.05)
            arm.set_mode(5); time.sleep(0.05)
            arm.set_state(0); time.sleep(0.05)
        except Exception as e:
            self.get_logger().warn(f"⚠️ {tag} entering VC mode raised: {e}")

        attempts = [
            ("tool", self._down_vec_tool(tag, vdown), True if use_tool_coord else False),
            ("tool(slow)", self._down_vec_tool(tag, max(2.0, min(vdown, 20.0))), True if use_tool_coord else False),
            ("base(slow)", self._down_vec_base(max(2.0, min(vdown, 20.0))), False),
        ]
        started = False
        for where, vec, toolflag in attempts:
            code = arm.vc_set_cartesian_velocity(vec, is_tool_coord=toolflag)
            if code == 0:
                started = True; self.get_logger().info(f"▶️ {tag} VC started in {where} frame"); break
            else:
                self.get_logger().warn(f"⛔ {tag} vc_set_cartesian_velocity code={code} in {where}")

        if not started:
            self._fallback_probe(arm, tag, timeout_s)
            try: arm.vc_set_cartesian_velocity([0, 0, 0, 0, 0, 0])
            except Exception: pass
            try: arm.set_mode(0); time.sleep(0.05); arm.set_state(0); time.sleep(0.05)
            except Exception: pass
            return

        # VC running → monitor torque until hit or timeout
        t0 = time.time()
        try:
            while time.time() - t0 < timeout_s:
                code, torq = arm.get_joints_torque()
                if code == 0 and isinstance(torq, (list, tuple)) and len(torq) >= 3:
                    tau = torq[2]
                    if tau >= self.probe_thresh_nm:
                        self.get_logger().info(f"📏 {tag} contact τ₃={tau:.2f} Nm (VC)")
                        break
                time.sleep(0.02)
        finally:
            try: arm.vc_set_cartesian_velocity([0, 0, 0, 0, 0, 0])
            except Exception: pass
            try: arm.set_mode(0); time.sleep(0.05); arm.set_state(0); time.sleep(0.05)
            except Exception: pass

    # ---------------- connection & motion helpers ----------------
    def _connect_arm(self, ip: str, tag: str) -> XArmAPI:
        self.get_logger().info(f"🔌 Connecting {tag} @ {ip} …")
        arm = XArmAPI(port=ip)
        arm.connect()
        try:
            arm.motion_enable(True)
        except Exception:
            pass
        time.sleep(0.05)
        try:
            arm.set_mode(0); time.sleep(0.05)
            arm.set_state(0); time.sleep(0.05)
        except Exception:
            pass
        try:
            arm.set_report_tau_or_i(1)
        except Exception:
            pass
        self.get_logger().info(f"✅ {tag} connected")
        return arm

    def _go_pose(self, arm: XArmAPI, pos_xyz_m, rpy_rad, tag: str, speed: float = None):
        x_m, y_m, z_m = pos_xyz_m
        roll, pitch, yaw = rpy_rad
        x_mm, y_mm, z_mm = x_m * 1000.0, y_m * 1000.0, z_m * 1000.0
        arm.motion_enable(True)
        arm.set_mode(0); time.sleep(0.02)
        arm.set_state(0); time.sleep(0.02)
        code = arm.set_position(
            x_mm, y_mm, z_mm, roll, pitch, yaw,
            speed=(speed or self.move_speed),
            is_radian=True, wait=True
        )
        if code != 0:
            self.get_logger().warn(
                f"⚠️ {tag} set_position code={code} (err={arm.error_code}, warn={arm.warn_code})"
            )
        else:
            self.get_logger().info(
                f"🛬 {tag} at pose mm=({x_mm:.1f},{y_mm:.1f},{z_mm:.1f}) rpy=({roll:.3f},{pitch:.3f},{yaw:.3f})"
            )

    def _go_home(self, arm: XArmAPI, tag: str):
        code = arm.move_gohome(wait=True)
        if code != 0:
            self.get_logger().warn(f"🏠 [{tag}] HOME code={code} (err={arm.error_code}, warn={arm.warn_code})")
        else:
            self.get_logger().info(f"🏠 [{tag}] at HOME")

    # ---------------- unscrewing ----------------
    def _unscrew_routine(self, arm, tag="TOOL"):
        """Drive screwdriver CCW (cmd=-1). When τ3 ≥ unscrew_thresh → retract 1 mm. Stop when torque stabilizes."""
        noise_thresh = 0.5
        stable_time = 1.5
        code, torq0 = arm.get_joints_torque()
        last_effort = torq0[2] if (code == 0 and isinstance(torq0, (list, tuple)) and len(torq0) >= 3) else 0.0
        last_change = time.time()

        self.get_logger().info("🔩 TOOL ON (unscrew)…")
        cmd = Int8(data=-1)
        self.tool_pub.publish(cmd)

        while True:
            code, torq = arm.get_joints_torque()
            effort = torq[2] if (code == 0 and isinstance(torq, (list, tuple)) and len(torq) >= 3) else last_effort

            if effort >= self.unscrew_thresh:
                self.get_logger().info(f"⬆️ τ₃={effort:.2f} ≥ {self.unscrew_thresh:.2f} → retract 1 mm")
                arm.set_tool_position(x=0, y=0, z=self._up_step(tag, 1.0),
                                      speed=self.probe_speed_mm, wait=True)
                last_change = time.time(); last_effort = effort
            elif abs(effort - last_effort) > noise_thresh:
                last_change = time.time(); last_effort = effort

            if (time.time() - last_change) >= stable_time:
                self.get_logger().info(f"🆙 Stable {stable_time:.1f}s → TOOL OFF")
                cmd.data = 0
                self.tool_pub.publish(cmd)
                break
            time.sleep(0.05)


def main(args=None):
    rclpy.init(args=args)
    node = Step8Control()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
