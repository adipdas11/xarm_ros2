#!/usr/bin/env python3
import json

import rclpy
from rclpy.node import Node
from disassembly_xarm5.srv import GetPartGraph
from std_msgs.msg import String
from sensor_msgs.msg import Image as RosImage
from cv_bridge import CvBridge
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
import numpy as np
import cv2

ACTION_COLORS = {
    'unscrew': "#F5D882",
    'remove':  "#C883D4",
    'drop':    "#F59D82",
}
ARM_COLORS = {
    'tooling':      '#1565C0',
    'manipulation': '#2E7D32',
}


class GraphUI(Node):
    def __init__(self):
        super().__init__('graph_ui_node')

        # ROS2 setup
        self.cli = self.create_client(GetPartGraph, 'get_part_graph')
        if not self.cli.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("Service get_part_graph not available")

        self.bridge = CvBridge()
        self.latest_img = None

        # subscribe to segmented_image
        self.create_subscription(
            RosImage,
            '/yolov11/segmented_image',
            self._on_image, 10
        )
        # subscribe to sequence & plan
        self.sequence = []
        self.create_subscription(
            String,
            '/disassembly_sequence',
            self._on_sequence, 10
        )
        self.dist_plan = []
        self.create_subscription(
            String,
            '/distributed_plan',
            self._on_plan, 10
        )

        # ---- Tkinter UI ----
        self.root = tk.Tk()
        self.root.title("Disassembly Visualizer")
        self.root.geometry("1000x800")

        # grid weights
        self.root.columnconfigure(0, weight=0)
        self.root.columnconfigure(1, weight=1)
        for r in range(8):
            self.root.rowconfigure(r, weight=0)
        self.root.rowconfigure(1, weight=1)

        # headers
        ttk.Label(self.root, text="Segmentation View",
                  font=('TkDefaultFont', 14, 'bold')).grid(row=0, column=0, pady=(8,0))
        ttk.Label(self.root, text="Part Graph",
                  font=('TkDefaultFont', 14, 'bold')).grid(row=0, column=1, pady=(8,0))

        # row 1: segmentation & graph
        self.img_label = tk.Label(self.root, bg='black')
        self.img_label.config(width=640, height=480)
        self.img_label.grid(row=1, column=0, padx=5, pady=5)
        self._make_scrollable_canvas('graph', height=480,
                                     both=True, row=1, column=1)

        # Disassembly Sequence
        ttk.Label(self.root, text="Disassembly Sequence",
                  font=('TkDefaultFont', 14, 'bold')).grid(row=2, column=0, columnspan=2, pady=(8,0))
        self._make_scrollable_canvas('seq', height=80,
                                     horiz_only=True, row=3,
                                     column=0, colspan=2)

        # Dual-Arm Plan (now linear)
        ttk.Label(self.root, text="Dual-Arm Plan",
                  font=('TkDefaultFont', 14, 'bold')).grid(row=4, column=0, columnspan=2, pady=(8,0))
        self._make_scrollable_canvas('plan', height=80,
                                     horiz_only=True, row=5,
                                     column=0, colspan=2)

        # Refresh + legend
        ttk.Button(self.root, text="Refresh Graph",
                   command=self.update_graph).grid(
            row=6, column=0, columnspan=2, sticky='ew', padx=50, pady=(4,10)
        )
        self._draw_legend().grid(
            row=7, column=0, columnspan=2, sticky='ew', padx=10, pady=(4,12)
        )

        # internal storage
        self.node_pos   = {}  # id -> (x,y)
        self.node_pos3  = {}  # id -> (x,y,z)
        self.node_label = {}  # id -> label
        self.edges      = []

        # initial graph draw
        self.update_graph()

    def _make_scrollable_canvas(self, name, height=200,
                                horiz_only=False, both=False,
                                row=0, column=0, colspan=1):
        frame = ttk.Frame(self.root)
        frame.grid(row=row, column=column,
                   columnspan=colspan, sticky='nsew',
                   padx=10, pady=4)
        frame.columnconfigure(0, weight=1)
        c = tk.Canvas(frame, bg='white', height=height)
        c.grid(row=0, column=0, sticky='nsew')
        if horiz_only or both:
            xsb = ttk.Scrollbar(frame, orient='horizontal', command=c.xview)
            xsb.grid(row=1, column=0, sticky='ew')
            c.configure(xscrollcommand=xsb.set)
        if both:
            ysb = ttk.Scrollbar(frame, orient='vertical', command=c.yview)
            ysb.grid(row=0, column=1, sticky='ns')
            c.configure(yscrollcommand=ysb.set)
        setattr(self, f"{name}_canvas", c)

    def _draw_legend(self):
        frame = ttk.Frame(self.root)
        # Arm outlines
        ttk.Label(frame, text="Arm outlines:",
                  font=('TkDefaultFont', 10, 'bold')).grid(row=0, column=0, sticky='w')
        col = 1
        for arm, colr in ARM_COLORS.items():
            cc = tk.Canvas(frame, width=20, height=20)
            cc.grid(row=0, column=col, padx=4)
            cc.create_rectangle(2,2,18,18, outline=colr, width=4, fill='white')
            ttk.Label(frame, text=arm).grid(row=0, column=col+1, padx=(0,12))
            col += 2
        # Action fills
        ttk.Label(frame, text="Action fills:",
                  font=('TkDefaultFont', 10, 'bold')).grid(row=1, column=0, sticky='w')
        col = 1
        for act, fill in ACTION_COLORS.items():
            cc = tk.Canvas(frame, width=20, height=20)
            cc.grid(row=1, column=col, padx=4)
            cc.create_rectangle(2,2,18,18, fill=fill, outline='black')
            ttk.Label(frame, text=act).grid(row=1, column=col+1, padx=(0,12))
            col += 2
        return frame

    def _on_image(self, msg: RosImage):
        cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        pil = Image.fromarray(cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB))
        imgtk = ImageTk.PhotoImage(pil.resize((640, 480)))
        self.latest_img = imgtk
        self.img_label.configure(image=imgtk)

    def _on_sequence(self, msg: String):
        try:
            self.sequence = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn("Bad JSON on /disassembly_sequence")
            return
        self._draw_sequence()

    def _on_plan(self, msg: String):
        try:
            self.dist_plan = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn("Bad JSON on /distributed_plan")
            return
        self._draw_plan()

    def update_graph(self):
        req = GetPartGraph.Request()
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        resp = fut.result()
        if resp is None:
            self.get_logger().error("Failed to call get_part_graph")
            return

        g = json.loads(resp.json_graph)
        # cache nodes & 3D positions
        self.node_pos.clear()
        self.node_pos3.clear()
        self.node_label.clear()
        for n in g['nodes']:
            x,y,z = n['position']
            self.node_pos[n['id']]  = (x, y)
            self.node_pos3[n['id']] = (x, y, z)
            self.node_label[n['id']] = n['label']
        self.edges = g['edges']
        self._draw_graph()

    def _draw_graph(self):
        c = self.graph_canvas
        c.delete("all")
        if not self.node_pos:
            return

        # flip vertically by using + instead of - in y mapping
        xs = [x for x,y in self.node_pos.values()]
        ys = [y for x,y in self.node_pos.values()]
        minx, maxx = min(xs), max(xs)
        miny, maxy = min(ys), max(ys)
        midx = (minx + maxx)/2
        midy = (miny + maxy)/2
        spanx = max(maxx-minx,1e-3)
        spany = max(maxy-miny,1e-3)
        W, H = 640, 480
        scale = 0.8 * min(W/spanx, H/spany)

        def tc(x,y):
            # flip Y: positive y goes down
            return (W/2 + (x-midx)*scale,
                    H/2 + (y-midy)*scale)

        # draw edges
        for e in self.edges:
            s,t = e['source'], e['target']
            if s in self.node_pos and t in self.node_pos:
                x1,y1 = tc(*self.node_pos[s])
                x2,y2 = tc(*self.node_pos[t])
                c.create_line(x1,y1,x2,y2, fill='gray', width=2)

        # draw nodes + labels + coords
        for nid,(x,y) in self.node_pos.items():
            cx, cy = tc(x,y)
            c.create_oval(cx-6, cy-6, cx+6, cy+6, fill='red', outline='')
            label = self.node_label[nid]
            cx_off = cx
            # label above
            c.create_text(cx_off, cy-12, text=label, font=('TkDefaultFont',8))
            # coords below
            x3,y3,z3 = self.node_pos3[nid]
            txt = f"({x3:.2f},{y3:.2f},{z3:.2f})"
            c.create_text(cx_off, cy+12, text=txt, font=('TkDefaultFont',7))

    def _draw_sequence(self):
        c = self.seq_canvas
        c.delete("all")
        n = len(self.sequence)
        if not n:
            return
        pad = 10
        total = pad*(n+1)
        box_w = max(60, (640-total)/n)
        box_h = 30
        c.config(scrollregion=(0,0,(box_w+pad)*n+pad,80))
        x = pad; y = (80-box_h)/2
        for lbl in self.sequence:
            c.create_rectangle(x,y,x+box_w,y+box_h,
                               fill='#cce5ff', outline='#3399ff', width=2)
            c.create_text(x+box_w/2,y+box_h/2,
                          text=lbl, font=('TkDefaultFont',10),
                          width=box_w-4)
            x += box_w + pad

    def _draw_plan(self):
        c = self.plan_canvas
        c.delete("all")
        if not self.dist_plan:
            return
        pad = 5
        box_w = 120
        box_h = 40
        count = len(self.dist_plan)
        c.config(scrollregion=(0,0,(box_w+pad)*count+pad, box_h+2*pad))
        x = pad; y = pad
        for item in self.dist_plan:
            arm, part = item['arm'], item['part']
            action     = item.get('action','')
            fill    = ACTION_COLORS.get(action, 'lightgray')
            outline = ARM_COLORS.get(arm, 'black')
            c.create_rectangle(x,y,x+box_w,y+box_h,
                               fill=fill, outline=outline, width=4)
            txt = f"{part}\n{action}"
            c.create_text(x+box_w/2,y+box_h/2,
                          text=txt, font=('TkDefaultFont',8),
                          width=box_w-4)
            x += box_w + pad

    def run(self):
        self.root.mainloop()


def main():
    rclpy.init()
    ui = GraphUI()
    ui.run()
    ui.destroy_node()
    rclpy.shutdown()


if __name__=='__main__':
    main()
