#!/usr/bin/env python3
"""
robot_control_node.py — dual-arm, fixed TF at start, probing for both arms
(Updated: retract 100 mm along tool-Z before final return for both arms)
"""

import os
import math
import time
import json
import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
from ament_index_python.packages import get_package_share_directory
from xarm.wrapper import XArmAPI

from hd_disassembly.srv import GetPlan, TransformPoint


# ---------------- helpers ----------------
def quaternion_to_euler(x, y, z, w):
    sinr = 2.0 * (w*x + y*z);  cosr = 1.0 - 2.0 * (x*x + y*y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (w*y - z*x)
    pitch = math.copysign(math.pi/2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    siny = 2.0 * (w*z + x*y);  cosy = 1.0 - 2.0 * (y*y + z*z)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


def _pose_from_cfg(block, key):
    p = block[key]['position']; q = block[key]['orientation']
    return ((float(p['x']), float(p['y']), float(p['z'])),
            (float(q['x']), float(q['y']), float(q['z']), float(q['w'])))


class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')
        self.get_logger().info("🤖 RobotControlNode (fixed TF, probing both arms) …")

        # -------- load YAML --------
        pkg = get_package_share_directory('hd_disassembly')
        cfg_path = os.path.join(pkg, 'config', 'disassembler_params.yaml')
        with open(cfg_path, 'r') as f:
            cfg = yaml.safe_load(f) or {}
        rcfg = cfg.get('robot', {})
        tcfg = rcfg.get('tool',  {})
        mcfg = rcfg.get('manip', {})
        vcfg = cfg.get('vision', {})
        cam  = vcfg.get('camera', {})

        # -------- IPs --------
        self.declare_parameter('robot.tool.ip',  tcfg.get('ip',  '192.168.1.239'))
        self.declare_parameter('robot.manip.ip', mcfg.get('ip', '192.168.1.195'))
        tool_ip  = self.get_parameter('robot.tool.ip').value
        manip_ip = self.get_parameter('robot.manip.ip').value

        # -------- poses --------
        tool_start_pos,  tool_start_q  = _pose_from_cfg(tcfg, 'start_pose')
        tool_ideal_pos,  tool_ideal_q  = _pose_from_cfg(tcfg, 'ideal_pose')
        manip_start_pos, manip_start_q = _pose_from_cfg(mcfg, 'start_pose')
        manip_ideal_pos, manip_ideal_q = _pose_from_cfg(mcfg, 'ideal_pose')

        self.tool_start_m   = tool_start_pos
        self.tool_ideal_m   = tool_ideal_pos
        self.manip_start_m  = manip_start_pos
        self.manip_ideal_m  = manip_ideal_pos
        self.tool_rpy       = quaternion_to_euler(*tool_ideal_q)
        self.manip_rpy      = quaternion_to_euler(*manip_ideal_q)

        # -------- motion / thresholds --------
        stop_h_m      = float(rcfg.get('stop_height',   0.10))   # m
        probe_mps     = float(rcfg.get('probing_speed', 0.0255)) # m/s
        thr           = rcfg.get('effort_threshold', {'probing': -6.0, 'unscrew': -6.0})
        probe_thresh  = float(thr.get('probing', -6.0))
        unscrew_thr   = float(thr.get('unscrew', -6.0))

        self.declare_parameter('robot.move_speed_mm_s',     150.0)
        self.declare_parameter('robot.stop_height_mm',      stop_h_m * 1000.0)
        self.declare_parameter('robot.probe_speed_mm_s',    probe_mps * 1000.0)
        self.declare_parameter('robot.probe_thresh_nm',     probe_thresh)
        self.declare_parameter('robot.unscrew_thresh_nm',   unscrew_thr)
        self.declare_parameter('robot.lift_after_grip_mm',  20.0)

        # Behavior toggles
        self.declare_parameter('park_other_arm_home', True)   # True: HOME; False: IDEAL
        self.declare_parameter('manip.hover_only',   False)   # default False → probe MANIP too

        # Per-arm probing signs & behavior
        self.declare_parameter('probe.prefer_tool_frame', True)
        self.declare_parameter('probe.tool_z_sign',   1)     # +1 if “down” is +Z in tool frame
        self.declare_parameter('probe.manip_z_sign',  1)     # +1 if “down” is +Z in tool frame
        self.declare_parameter('probe.base_z_sign',   1)     # +1 if “down” is +Z in base frame (fallback)

        # Vision frame for approx inputs
        self.declare_parameter('vision.frame_id', cam.get('frame_id', 'R_camera_color_optical_frame'))
        self.vision_frame = str(self.get_parameter('vision.frame_id').value)

        self.move_speed       = float(self.get_parameter('robot.move_speed_mm_s').value)
        self.stop_height_mm   = float(self.get_parameter('robot.stop_height_mm').value)
        self.probe_speed_mm   = float(self.get_parameter('robot.probe_speed_mm_s').value)
        self.probe_thresh_nm  = float(self.get_parameter('robot.probe_thresh_nm').value)
        self.unscrew_thresh   = float(self.get_parameter('robot.unscrew_thresh_nm').value)
        self.lift_after_grip  = float(self.get_parameter('robot.lift_after_grip_mm').value)

        self.park_other_home  = bool(self.get_parameter('park_other_arm_home').value)
        self.manip_hover_only = bool(self.get_parameter('manip.hover_only').value)

        self.prefer_tool_frame = bool(self.get_parameter('probe.prefer_tool_frame').value)
        self.tool_z_sign       = int(self.get_parameter('probe.tool_z_sign').value)
        self.manip_z_sign      = int(self.get_parameter('probe.manip_z_sign').value)
        self.base_z_sign       = int(self.get_parameter('probe.base_z_sign').value)

        self.get_logger().info(f"🧭 TOOL IP: {tool_ip} | MANIP IP: {manip_ip}")
        self.get_logger().info(f"⚙️ move={self.move_speed:.1f} mm/s, probe={self.probe_speed_mm:.1f} mm/s, stop_h={self.stop_height_mm:.1f} mm")
        self.get_logger().info(f"🧱 thresholds: probe={self.probe_thresh_nm:.2f} Nm, unscrew={self.unscrew_thresh:.2f} Nm")
        self.get_logger().info(f"🎯 vision frame: {self.vision_frame}")
        self.get_logger().info(f"🅿️ park_other_arm_home={self.park_other_home}, manip.hover_only={self.manip_hover_only}")

        # -------- connect arms --------
        self.arm_tool  = self._connect_arm(tool_ip,  'TOOL')
        self.arm_manip = self._connect_arm(manip_ip, 'MANIP')

        # screwdriver command
        self.tool_pub = self.create_publisher(Int8, '/tool_cmd', 10)

        # -------- services --------
        self.plan_cli    = self.create_client(GetPlan,        'get_plan')
        self.tf_cli_tool = self.create_client(TransformPoint, 'transform_point_tool')
        self.tf_cli_man  = self.create_client(TransformPoint, 'transform_point_manip')
        for name, cli in [('get_plan', self.plan_cli),
                          ('transform_point_tool', self.tf_cli_tool),
                          ('transform_point_manip', self.tf_cli_man)]:
            while not cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn(f"⏳ Waiting for {name}…")
            self.get_logger().info(f"✅ {name} ready")

        # -------- go! --------
        self.execute_plan()

    # -------------- arm I/O --------------
    def _connect_arm(self, ip, tag):
        self.get_logger().info(f"🔌 Connecting {tag} @ {ip} …")
        arm = XArmAPI(port=ip)
        arm.connect()
        arm.motion_enable(True); time.sleep(0.1)
        arm.set_mode(0); time.sleep(0.1)
        arm.set_state(0); time.sleep(0.1)
        try:
            arm.set_report_tau_or_i(1)
        except Exception:
            pass
        self.get_logger().info(f"✅ {tag} connected")
        return arm

    def _go_home(self, arm, tag):
        code = arm.move_gohome(wait=True)
        if code != 0:
            self.get_logger().warn(f"🏠 [{tag}] HOME code={code} (err={arm.error_code}, warn={arm.warn_code})")
        else:
            self.get_logger().info(f"🏠 [{tag}] at HOME")

    def _go_pose(self, arm, pos_xyz_m, rpy, tag, speed=None):
        x, y, z = pos_xyz_m
        r, p, yaw = rpy
        arm.motion_enable(True)
        arm.set_mode(0);  time.sleep(0.05)
        arm.set_state(0); time.sleep(0.05)
        arm.set_position(x*1000.0, y*1000.0, z*1000.0,
                         r, p, yaw,
                         speed=(speed or self.move_speed),
                         is_radian=True, wait=True)
        self.get_logger().info(f"✅ {tag} reached pose {pos_xyz_m}")

    def _park_other(self, which_doing: str):
        if which_doing == 'tool':
            if self.park_other_home: self._go_home(self.arm_manip, 'MANIP')
            else:                    self._go_pose(self.arm_manip, self.manip_ideal_m, self.manip_rpy, 'MANIP')
        else:
            if self.park_other_home: self._go_home(self.arm_tool, 'TOOL')
            else:                    self._go_pose(self.arm_tool, self.tool_ideal_m, self.tool_rpy, 'TOOL')

    def _hover_above(self, arm, base_point, rpy, tag, speed=None):
        r, p, yaw = rpy
        x_mm = base_point['x'] * 1000.0
        y_mm = base_point['y'] * 1000.0
        z_mm = base_point['z'] * 1000.0 + self.stop_height_mm
        self.get_logger().info(
            f"🧮 HOVER input {tag}: base(m)=({base_point['x']:.4f},{base_point['y']:.4f},{base_point['z']:.4f}), "
            f"stop_h={self.stop_height_mm:.1f} → cmd(mm)=({x_mm:.1f},{y_mm:.1f},{z_mm:.1f})"
        )
        arm.set_position(x_mm, y_mm, z_mm, r, p, yaw,
                         speed=(speed or self.move_speed),
                         is_radian=True, wait=True)
        self.get_logger().info(f"🛬 {tag} hover @ ({x_mm:.1f},{y_mm:.1f},{z_mm:.1f}) mm")

    # ----- probing helpers (per-arm Z sign) -----
    def _arm_z_sign(self, tag: str) -> int:
        return self.tool_z_sign if tag.upper() == 'TOOL' else self.manip_z_sign

    def _down_vec_tool(self, tag: str, v_mm_s: float):
        s = self._arm_z_sign(tag)
        return [0, 0, s * abs(v_mm_s), 0, 0, 0]

    def _down_vec_base(self, v_mm_s: float):
        s = self.base_z_sign
        return [0, 0, s * abs(v_mm_s), 0, 0, 0]

    def _up_step(self, tag: str, step_mm: float) -> float:
        return -self._arm_z_sign(tag) * abs(step_mm)

    def _retract_mm(self, arm, tag: str, mm: float, speed: float = None):
        """Retract along the tool Z (away from contact) by +mm (always 'up')."""
        dz = self._up_step(tag, mm)
        arm.set_tool_position(x=0, y=0, z=dz,
                              speed=(speed or self.probe_speed_mm),
                              wait=True)
        self.get_logger().info(f"⬆️ {tag} retract {mm:.1f} mm (tool-Z)")

    # ----- fallback step-probing (when VC cannot start) -----
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
            code = arm.set_tool_position(x=0, y=0, z=dz,
                                         speed=max(5.0, min(self.probe_speed_mm, 50.0)),
                                         wait=True)
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
        vdown = float(max(1.0, min(self.probe_speed_mm, 60.0)))
        self.get_logger().info(
            f"🔬 {tag} probing until τ₃ ≥ {self.probe_thresh_nm:.2f} Nm "
            f"(v={vdown:.1f} mm/s, tool_coord={use_tool_coord})"
        )
        # enter VC
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

        # try tool frame, then slow tool, then slow base frame
        attempts = [
            ("tool", self._down_vec_tool(tag, vdown), True if use_tool_coord else False),
            ("tool(slow)", self._down_vec_tool(tag, max(2.0, min(vdown, 20.0))), True if use_tool_coord else False),
            ("base(slow)", self._down_vec_base(max(2.0, min(vdown, 20.0))), False),
        ]

        started = False
        for where, vec, toolflag in attempts:
            code = arm.vc_set_cartesian_velocity(vec, is_tool_coord=toolflag)
            if code == 0:
                started = True
                self.get_logger().info(f"▶️ {tag} VC started in {where} frame")
                break
            else:
                self.get_logger().warn(f"⛔ {tag} vc_set_cartesian_velocity code={code} in {where}")

        if not started:
            self._fallback_probe(arm, tag, timeout_s)
            try: arm.vc_set_cartesian_velocity([0,0,0,0,0,0])
            except Exception: pass
            try: arm.set_mode(0); time.sleep(0.05); arm.set_state(0); time.sleep(0.05)
            except Exception: pass
            return

        # VC running → monitor torque
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
            try: arm.vc_set_cartesian_velocity([0,0,0,0,0,0])
            except Exception: pass
            try: arm.set_mode(0); time.sleep(0.05); arm.set_state(0); time.sleep(0.05)
            except Exception: pass

    def _unscrew_routine(self, arm, tag='TOOL'):
        noise_thresh = 0.5
        stable_time  = 1.5
        code, torq0 = arm.get_joints_torque()
        last_effort = torq0[2] if (code == 0 and len(torq0) >= 3) else 0.0
        last_change = time.time()

        self.get_logger().info("🔩 TOOL ON (screwdriver)…")
        cmd = Int8(data=-1); self.tool_pub.publish(cmd)

        while True:
            code, torq = arm.get_joints_torque()
            effort = torq[2] if (code == 0 and len(torq) >= 3) else last_effort

            if effort >= self.unscrew_thresh:
                self.get_logger().info(f"⬆️ τ₃={effort:.2f} ≥ {self.unscrew_thresh:.2f} → retract 1 mm")
                arm.set_tool_position(x=0, y=0, z=self._up_step(tag, 1.0),
                                      speed=self.probe_speed_mm, wait=True)
                last_change = time.time(); last_effort = effort
            elif abs(effort - last_effort) > noise_thresh:
                last_change = time.time(); last_effort = effort

            if (time.time() - last_change) >= stable_time:
                self.get_logger().info(f"🆙 Stable {stable_time:.1f}s → TOOL OFF")
                cmd.data = 0; self.tool_pub.publish(cmd)
                break
            time.sleep(0.05)

    # ---------- main ----------
    def execute_plan(self):
        # Put arms in known state while snapshot & transform
        self._go_pose(self.arm_tool,  self.tool_start_m,  self.tool_rpy,  'TOOL')
        self._go_pose(self.arm_manip, self.manip_ideal_m, self.manip_rpy, 'MANIP')

        # Snapshot plan
        preq = GetPlan.Request(); preq.mode = 'snapshot'; preq.dedup_tol_m = 0.015
        fut = self.plan_cli.call_async(preq)
        rclpy.spin_until_future_complete(self, fut)
        pres = fut.result()
        if not pres or not pres.ok:
            msg = pres.message if pres else "(no response)"
            self.get_logger().error(f"❌ get_plan failed: {msg}")
            return

        try:
            plan = json.loads(pres.plan_json)  # list of {key,label,arm,priority,approx:{x,y,z}}
        except Exception as e:
            self.get_logger().error(f"❌ plan_json parse error: {e}")
            return

        if not plan:
            self.get_logger().warn("ℹ️ Plan is empty; nothing to do."); return

        # Freeze transforms NOW (camera/TOOL is in known pose)
        fixed = []
        from geometry_msgs.msg import PointStamped
        for item in plan:
            label = item.get('label', 'unknown')
            which = item.get('arm', 'manip')
            approx = item.get('approx', None)
            key    = item.get('key', '?')
            if not approx or any(k not in approx for k in ('x','y','z')):
                self.get_logger().warn(f"⛔ Missing approx for {key} ({label}); skipping"); continue

            ps = PointStamped()
            ps.header.frame_id = self.vision_frame
            ps.point.x = float(approx['x']); ps.point.y = float(approx['y']); ps.point.z = float(approx['z'])

            tp_req = TransformPoint.Request(); tp_req.point_in = ps
            fut_tf = (self.tf_cli_tool if which == 'tool' else self.tf_cli_man).call_async(tp_req)
            rclpy.spin_until_future_complete(self, fut_tf)
            tf_res = fut_tf.result()
            if not tf_res:
                self.get_logger().warn(f"⛔ transform_point failed for {key} ({label}); skipping"); continue

            out = tf_res.point_out
            fixed.append({
                'key': key, 'label': label, 'arm': which,
                'approx_frame': self.vision_frame,
                'approx': {'x': ps.point.x, 'y': ps.point.y, 'z': ps.point.z},
                'tf_frame': out.header.frame_id or '(unknown_frame)',
                'target': {'x': out.point.x, 'y': out.point.y, 'z': out.point.z},
            })

        if not fixed:
            self.get_logger().warn("⛔ No valid transformed targets. Aborting."); return

        # Print final frozen sequence
        self.get_logger().info("🧾 FINAL SEQUENCE (frozen transforms):")
        for i, it in enumerate(fixed, start=1):
            ax, ay, az = it['approx']['x'], it['approx']['y'], it['approx']['z']
            tx, ty, tz = it['target']['x'], it['target']['y'], it['target']['z']
            self.get_logger().info(
                f"  {i:02d}. {it['label']:>6} → arm={it['arm']:<5} | "
                f"approx({it['approx_frame']}): {ax:.4f},{ay:.4f},{az:.4f}  →  "
                f"{it['tf_frame']}: {tx:.4f},{ty:.4f},{tz:.4f}"
            )

        # Execute (no re-acquire)
        for i, it in enumerate(fixed, start=1):
            label = it['label']; which = it['arm']; basep = it['target']; basef = it['tf_frame']
            self.get_logger().info(
                f"🔀 [{i}/{len(fixed)}] {it['key']} → {label}, arm={which} | "
                f"target {basef}: {basep['x']:.4f},{basep['y']:.4f},{basep['z']:.4f}"
            )

            # Park other arm
            self._park_other(which)

            if which == 'tool' and label == 'screw_head':
                # TOOL: hover → probe → retract → unscrew → BIG retract → back to START
                self._hover_above(self.arm_tool, basep, self.tool_rpy, 'TOOL', speed=self.move_speed)
                self._probe_until_contact(self.arm_tool, 'TOOL', timeout_s=10.0, use_tool_coord=True)

                # small relief before unscrew (as before)
                self.get_logger().info("↕️ TOOL retract 5 mm…")
                self.arm_tool.set_tool_position(x=0, y=0, z=self._up_step('TOOL', 5.0),
                                                speed=self.probe_speed_mm, wait=True)

                self._unscrew_routine(self.arm_tool, tag='TOOL')

                # NEW: big retract 100 mm before returning
                self._retract_mm(self.arm_tool, 'TOOL', 100.0, speed=self.probe_speed_mm)

                self._go_pose(self.arm_tool, self.tool_start_m, self.tool_rpy, 'TOOL')

            else:
                # MANIP: hover → (optionally) probe → vacuum ON → lift → BIG retract → park
                self._go_pose(self.arm_manip, self.manip_start_m, self.manip_rpy, 'MANIP')
                self._hover_above(self.arm_manip, basep, self.manip_rpy, 'MANIP', speed=self.move_speed)

                if self.manip_hover_only:
                    self.get_logger().info("⏸ MANIP hover-only mode → skipping probe/lift for validation.")
                else:
                    self._probe_until_contact(self.arm_manip, 'MANIP', timeout_s=10.0, use_tool_coord=True)

                    # small relief
                    self.get_logger().info("↕️ MANIP retract 2 mm…")
                    self.arm_manip.set_tool_position(x=0, y=0, z=self._up_step('MANIP', 2.0),
                                                     speed=self.probe_speed_mm, wait=True)

                    self.get_logger().info("🧲 MANIP vacuum ON")
                    try:
                        self.arm_manip.set_suction_cup(True, wait=True)
                    except Exception as e:
                        self.get_logger().warn(f"Vacuum ON error: {e}")

                    self.get_logger().info(f"⬆️ MANIP lift {self.lift_after_grip:.1f} mm…")
                    self.arm_manip.set_tool_position(x=0, y=0, z=self._up_step('MANIP', self.lift_after_grip),
                                                     speed=self.probe_speed_mm, wait=True)

                    # NEW: big retract 100 mm before parking
                    self._retract_mm(self.arm_manip, 'MANIP', 100.0, speed=self.probe_speed_mm)

                    # Park MANIP at IDEAL (unchanged behavior)
                    self._go_pose(self.arm_manip, self.manip_ideal_m, self.manip_rpy, 'MANIP')

        # Park both at the end
        self.get_logger().info("🎉 Plan complete → park both")
        self._go_pose(self.arm_tool,  self.tool_ideal_m,  self.tool_rpy,  'TOOL')
        self._go_pose(self.arm_manip, self.manip_ideal_m, self.manip_rpy, 'MANIP')


def main(args=None):
    rclpy.init(args=args)
    node = RobotControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
