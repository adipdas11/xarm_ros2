#!/usr/bin/env python3
import os, yaml, json, time, heapq, math
from collections import defaultdict

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
from hd_disassembly.srv import GetSequence, GetPlan   # <- new

class PlannerNode(Node):
    def __init__(self):
        super().__init__('planner_node')
        self.get_logger().info("🚀 PlannerNode starting up…")

        # Load YAML
        pkg_share = get_package_share_directory('hd_disassembly')
        with open(os.path.join(pkg_share, 'config', 'disassembler_params.yaml'), 'r') as f:
            cfg = yaml.safe_load(f)

        # Config
        self.PRI = cfg['disassembly']['part_priority']
        self.ARM = cfg['disassembly'].get('arm_assign', {'default':'manip'})
        if 'default' not in self.ARM:
            self.ARM['default'] = 'manip'

        # Tunables
        self.declare_parameter('build_delay_sec', 0.5)     # wait to gather tracks
        self.declare_parameter('dedup_tol_m',     0.01)    # 1 cm spatial bin for keys

        self.build_delay_sec = float(self.get_parameter('build_delay_sec').value)
        self.dedup_tol_m     = float(self.get_parameter('dedup_tol_m').value)

        self.get_logger().info(f"🔧 Part priorities: {self.PRI}")
        self.get_logger().info(f"🔧 Arm assignment:  {self.ARM}")
        self.get_logger().info(f"🔧 dedup_tol_m:     {self.dedup_tol_m:.3f}")

        # State (from vision)
        self.latest_parts = []   # list of {"id", "part", "position": {"x","y","z"}, ...}

        # IO
        self.create_subscription(String, 'vision_tracks', self._tracks_cb, 10)
        self.create_service(GetSequence, 'get_sequence', self._seq_cb)  # legacy
        self.create_service(GetPlan,     'get_plan',     self._plan_cb) # NEW
        self.get_logger().info("✅ Services: /get_sequence (legacy), /get_plan (one-shot plan)")

    # ---------- utils ----------
    def _priority(self, label):  return self.PRI.get(label, self.PRI['default'])
    def _arm_for(self, label):   return self.ARM.get(label, self.ARM['default'])

    def _spatial_key(self, label, xyz, tol):
        """Stable key: label:q_x:q_y:q_z with q_* = round(coord/tol)."""
        if xyz is None: return f"{label}:nan:nan:nan"
        x,y,z = xyz
        if tol <= 0: tol = self.dedup_tol_m
        qx, qy, qz = int(round(x/tol)), int(round(y/tol)), int(round(z/tol))
        return f"{label}:{qx}:{qy}:{qz}"

    def _labels_summary(self, parts):
        c = defaultdict(int)
        for p in parts: c[p.get('part','?')] += 1
        return ", ".join(f"{k}:{v}" for k,v in c.items()) or "none"

    # ---------- subscribers ----------
    def _tracks_cb(self, msg: String):
        try:
            self.latest_parts = json.loads(msg.data)  # list of dicts
        except Exception as e:
            self.get_logger().error(f"❌ vision_tracks JSON parse error: {e}")

    # ---------- legacy: step-by-step ----------
    def _seq_cb(self, req, res):
        # Keep old behavior for anything using it
        rt = req.request_type.lower()
        if rt == 'start':
            # small wait to gather tracks
            end = time.time() + self.build_delay_sec
            while rclpy.ok() and time.time() < end:
                rclpy.spin_once(self, timeout_sec=0.05)

        # Simple priority order on current snapshot; return one item
        parts = list(self.latest_parts)
        parts.sort(key=lambda p: (self._priority(p.get('part','unknown')), p.get('id', 1<<30)))
        if not parts:
            res.part_id = -1; res.part_label = ''; res.assigned_arm = ''; res.finished = True
            return res

        if rt == 'start':
            chosen = parts[0]
        else:
            chosen = parts[1] if len(parts) > 1 else parts[0]

        res.part_id     = int(chosen.get('id', -1))
        res.part_label  = chosen.get('part','unknown')
        res.assigned_arm= self._arm_for(res.part_label)
        res.finished    = False
        return res

    # ---------- NEW: one-shot plan ----------
    def _plan_cb(self, req, res):
        # Mode "snapshot": take one stabilized snapshot, build plan
        tol = float(req.dedup_tol_m) if req.dedup_tol_m > 0.0 else self.dedup_tol_m

        # wait briefly to fill latest_parts
        end = time.time() + self.build_delay_sec
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

        parts = list(self.latest_parts)
        if not parts:
            res.ok = False
            res.plan_json = "[]"
            res.message = "No parts visible"
            return res

        # Dedup by spatial key; average positions inside a bin
        bins = {}   # key -> {'label','arm','priority','sum':[x,y,z],'n':n}
        for p in parts:
            lbl = p.get('part','unknown')
            pos = p.get('position') or {}
            xyz = (float(pos.get('x', math.nan)),
                   float(pos.get('y', math.nan)),
                   float(pos.get('z', math.nan)))
            if any(map(lambda v: math.isnan(v) or abs(v) > 10, xyz)):
                continue
            key = self._spatial_key(lbl, xyz, tol)
            if key not in bins:
                bins[key] = {
                    'label': lbl,
                    'arm': self._arm_for(lbl),
                    'priority': self._priority(lbl),
                    'sum': [0.0,0.0,0.0],
                    'n': 0,
                }
            b = bins[key]
            b['sum'][0] += xyz[0]; b['sum'][1] += xyz[1]; b['sum'][2] += xyz[2]
            b['n'] += 1

        # Build plan list
        plan = []
        for key, b in bins.items():
            n = max(1, b['n'])
            avg = {'x': b['sum'][0]/n, 'y': b['sum'][1]/n, 'z': b['sum'][2]/n}
            plan.append({
                'key': key,
                'label': b['label'],
                'arm': b['arm'],
                'priority': int(b['priority']),
                'approx': avg
            })

        # Sort by priority, then key (deterministic)
        plan.sort(key=lambda e: (e['priority'], e['label'], e['key']))

        res.ok = True
        res.plan_json = json.dumps(plan)
        res.message = f"Built plan with {len(plan)} items (snapshot: {self._labels_summary(parts)})"
        return res

def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()