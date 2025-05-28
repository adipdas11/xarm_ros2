#!/usr/bin/env python3

import json
import sys
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time as RclpyTime
from std_msgs.msg import String, Int8
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Pose
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Time as TimeMsg

import tf2_ros
import tf2_geometry_msgs  
from xarm_msgs.srv import PlanPose, PlanExec
from disassembly_xarm5.srv import GetPartGraph

# Disassembly priority: lower values removed first
PART_PRIORITY = {
    'screw':   0,
    'lid':     1,
    'pcb':     2,
    'default': 3,
}

# When joint5 effort < this, we've contacted the screw
EFFORT_THRESHOLD = -0.0013

class DisassemblyNode(Node):
    MAX_RETRIES = 3

    def __init__(self):
        super().__init__('disassembly_node')

        # TF2
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # motion service clients
        self.pose_cli = self.create_client(PlanPose, '/xarm_pose_plan')
        self.exec_cli = self.create_client(PlanExec, '/xarm_exec_plan')
        if not self.pose_cli.wait_for_service(5.0) or not self.exec_cli.wait_for_service(5.0):
            self.get_logger().error("❌ Motion services unavailable")
            sys.exit(1)

        # part-graph service client
        self.graph_cli = self.create_client(GetPartGraph, 'get_part_graph')
        if not self.graph_cli.wait_for_service(5.0):
            self.get_logger().error("❌ get_part_graph service unavailable")
            sys.exit(1)

        # Publishers & Subscribers
        self.seq_pub     = self.create_publisher(String, '/disassembly_sequence', 10)
        self.marker_pub  = self.create_publisher(Marker, '/disassembly/target_marker', 10)
        self.tool_pub    = self.create_publisher(Int8, '/tool_cmd', 10)
        self.create_subscription(JointState, '/xarm5_joint_states',
                                 self._joint_states_cb, 10)

        # internal state
        self.positions         = {}
        self.labels            = {}
        self.removed_ids       = set()
        self.sequence          = []
        self.current_pose      = None
        self.retry_counts      = {}
        self.current_nid       = None
        self.monitoring_effort = False
        self.last_effort       = None

        # home pose
        self.home = Pose()
        self.home.position.x = 0.4
        self.home.position.y = 0.0
        self.home.position.z = 0.15
        self.home.orientation.x = 1.0
        self.home.orientation.y = 0.0
        self.home.orientation.z = 0.0
        self.home.orientation.w = 0.0

        self.get_logger().info('✅ DisassemblyNode initialized.')
        self.fetch_graph()

    def fetch_graph(self):
        req = GetPartGraph.Request()
        self.graph_cli.call_async(req).add_done_callback(self._on_graph_response)

    def _on_graph_response(self, fut):
        try:
            resp = fut.result()
        except Exception as e:
            self.get_logger().error(f"❌ get_part_graph call failed: {e}")
            return

        self.get_logger().info("🔄 Received graph via service")
        data = json.loads(resp.json_graph)

        self.positions.clear()
        self.labels.clear()
        for node in data.get('nodes', []):
            self.positions[node['id']] = node['position']
            self.labels[node['id']]    = node['label']

        parts = []
        for nid, label in self.labels.items():
            if not label.startswith('screw_'): continue
            if nid in self.removed_ids:      continue
            prio = PART_PRIORITY.get(label.split('_')[0], PART_PRIORITY['default'])
            parts.append((prio, label, nid))
        parts.sort(key=lambda x: (x[0], x[1]))
        self.sequence = [nid for _,_,nid in parts]

        labels = [self.labels[nid] for nid in self.sequence]
        self.get_logger().info(f"2️⃣ Screw sequence: {labels}")
        self.seq_pub.publish(String(data=json.dumps(labels)))

        time.sleep(2.0)
        self.process_next()

    def _joint_states_cb(self, msg: JointState):
        # always track last joint5 effort
        try:
            idx = msg.name.index('joint5')
            self.last_effort = msg.effort[idx]
        except (ValueError, IndexError):
            return

        if self.monitoring_effort and self.last_effort is not None \
           and self.last_effort < EFFORT_THRESHOLD:
            self.monitoring_effort = False
            label = self.labels[self.current_nid]
            self.get_logger().info(
                f"🛑 Effort {self.last_effort:.4f} < threshold: reached {label}"
            )
            self._unscrew(label)

    def _unscrew(self, label: str):
        # 1) start unscrew motor
        self.get_logger().info(f"🔧 Unscrewing {label} for 10 s…")
        self.tool_pub.publish(Int8(data=-1))

        # 2) monitor and print effort for 10s
        end_time = time.time() + 10.0
        while time.time() < end_time:
            # if self.last_effort is not None:
            #     self.get_logger().info(f"    joint5 effort: {self.last_effort:.4f}")
            time.sleep(0.5)

        # 3) stop motor
        self.tool_pub.publish(Int8(data=0))
        self.get_logger().info("🔧 Unscrewing complete, stopping tool_cmd=0")

        # 4) return home
        self.get_logger().info("7️⃣ Planning pose to home")
        self._plan_home()

    def process_next(self):
        if not self.sequence:
            self.get_logger().info("🎉 All screws removed.")
            return

        self.current_nid = self.sequence[0]
        label = self.labels[self.current_nid]
        x,y,z = self.positions[self.current_nid]
        self.get_logger().info(
            f"3️⃣ Next screw {label}, camera coords [{x:.3f},{y:.3f},{z:.3f}]"
        )

        ps_cam = PoseStamped()
        ps_cam.header.frame_id = 'camera_color_optical_frame'
        ps_cam.header.stamp = TimeMsg()
        ps_cam.pose.position.x = x
        ps_cam.pose.position.y = y
        ps_cam.pose.position.z = z
        ps_cam.pose.orientation = self.home.orientation

        while not self.tf_buffer.can_transform(
                'link_base', ps_cam.header.frame_id, RclpyTime()):
            time.sleep(0.05)

        try:
            psb = self.tf_buffer.transform(
                ps_cam, 'link_base', timeout=Duration(seconds=1.0))
        except Exception as e:
            self.get_logger().error(f"❌ TF transform failed: {e}")
            return

        psb.pose.orientation.x = 1.0
        psb.pose.orientation.y = 0.0
        psb.pose.orientation.z = 0.0
        psb.pose.orientation.w = 0.0
        self.current_pose = psb.pose

        p,o = psb.pose.position, psb.pose.orientation
        self.get_logger().info(
            f"4️⃣ Transformed → pos=({p.x:.3f},{p.y:.3f},{p.z:.3f}),"
            f" ori=(x={o.x:.1f},y={o.y:.1f},z={o.z:.1f},w={o.w:.1f})"
        )

        m = Marker()
        m.header = psb.header; m.ns='disassembly_target'; m.id=0
        m.type=Marker.SPHERE; m.action=Marker.ADD; m.pose=psb.pose
        m.scale.x=m.scale.y=m.scale.z=0.05; m.color.r=1.0; m.color.a=0.8
        self.marker_pub.publish(m)

        self.get_logger().info("5️⃣ Planning pose to target")
        self._plan_screw()

    def _plan_screw(self):
        tag = "screw_plan"; self.retry_counts[tag]=0
        req = PlanPose.Request(target=self.current_pose)
        self.pose_cli.call_async(req).add_done_callback(
            lambda f: self._on_plan(f, tag))

    def _on_plan(self, fut, tag):
        try:
            resp = fut.result()
        except Exception as e:
            self.get_logger().error(f"❌ [{tag}] {e}")
            return

        if resp.success:
            self.get_logger().info("6️⃣ Executing planned pose")
            self.monitoring_effort = True
            self.exec_cli.call_async(PlanExec.Request(wait=True))
        else:
            cnt = self.retry_counts[tag] + 1
            if cnt < self.MAX_RETRIES:
                self.retry_counts[tag] = cnt
                self.get_logger().warn(f"⚠️ [{tag}] retry {cnt}")
                time.sleep(1.0); self._plan_screw()
            else:
                self.get_logger().error(f"❌ [{tag}] aborting")
                raise RuntimeError

    def _plan_home(self):
        tag = "home_plan"; self.retry_counts[tag]=0
        req = PlanPose.Request(target=self.home)
        self.pose_cli.call_async(req).add_done_callback(
            lambda f: self._on_home_plan(f, tag))

    def _on_home_plan(self, fut, tag):
        try:
            resp = fut.result()
        except Exception as e:
            self.get_logger().error(f"❌ [home] {e}")
            return

        if resp.success:
            self.get_logger().info("8️⃣ Executing home pose")
            self.exec_cli.call_async(PlanExec.Request(wait=True)).add_done_callback(
                lambda f: self._on_home_exec(f, tag))
        else:
            cnt = self.retry_counts[tag] + 1
            if cnt < self.MAX_RETRIES:
                self.retry_counts[tag] = cnt
                self.get_logger().warn(f"⚠️ [home] retry {cnt}")
                time.sleep(1.0); self._plan_home()
            else:
                self.get_logger().error(f"❌ [home] aborting")
                raise RuntimeError

    def _on_home_exec(self, fut, tag):
        try:
            resp = fut.result()
        except Exception as e:
            self.get_logger().error(f"❌ [{tag}] {e}")
            return

        if resp.success:
            self.removed_ids.add(self.current_nid)
            self.get_logger().info("9️⃣ Home complete — will refresh graph in 5 s…\n")
            time.sleep(5.0)
            self.fetch_graph()
        else:
            cnt = self.retry_counts[tag] + 1
            if cnt < self.MAX_RETRIES:
                self.retry_counts[tag] = cnt
                self.get_logger().warn(f"⚠️ [{tag}] retry {cnt}")
                time.sleep(1.0); self._on_home_exec(fut, tag)
            else:
                self.get_logger().error(f"❌ [{tag}] aborting")
                raise RuntimeError


def main(args=None):
    rclpy.init(args=args)
    node = DisassemblyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()