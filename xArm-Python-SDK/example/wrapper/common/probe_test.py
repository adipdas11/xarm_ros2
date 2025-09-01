#!/usr/bin/env python3
"""
xArm probe-down test (MANIP) — position to start, then VC probe along base Z.

- Robot: MANIP at 192.168.1.195
- Start pose (m):  x=0.40, y=0.30, z=0.40
- Start orientation (quat): (x=1, y=0, z=0, w=0)  -> roll=180°, pitch=0°, yaw=0°
- Probe in velocity mode along BASE Z until joint3 torque >= -6.0 Nm
- Retract 5 mm upward and stop

Change BASE_Z_SIGN to +1 if your base +Z is DOWN (or if motion goes the wrong way).
"""

import math
import time
from xarm.wrapper import XArmAPI

# ---------------- User-tunable constants ----------------
IP                = "192.168.1.195"
PROBE_SPEED_MM_S  = 25.5            # linear speed during probing
PROBE_TIMEOUT_S   = 10.0            # safety timeout
TORQUE_THRESH_NM  = -6.0            # joint3 torque threshold (trigger when >= this)
RETRACT_MM        = 5.0             # retract distance after contact
BASE_Z_SIGN       = -1              # "down" in base frame: -1 if +Z is up (most setups), +1 if +Z is down
SLEEP_DT          = 0.02            # polling period (50 Hz)
# --------------------------------------------------------


def quaternion_to_euler_deg(x, y, z, w):
    """Return roll, pitch, yaw in DEGREES from quaternion (x, y, z, w)."""
    # from tf_transformations (inlined)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def main():
    arm = XArmAPI(IP)
    try:
        print(f"[INFO] Connecting to xArm @ {IP} …")
        arm.connect()
        # enable motion + position mode
        arm.motion_enable(True); time.sleep(0.1)
        arm.set_mode(0);         time.sleep(0.1)
        arm.set_state(0);        time.sleep(0.1)

        # optional: enable torque reporting (model dependent, ignored if unsupported)
        try:
            arm.set_report_tau_or_i(1)
        except Exception:
            pass

        # ---- Move to START POSE (position control) ----
        start_xyz_m = (0.40, 0.30, 0.40)
        qx, qy, qz, qw = (1.0, 0.0, 0.0, 0.0)  # 180° about X
        roll_deg, pitch_deg, yaw_deg = quaternion_to_euler_deg(qx, qy, qz, qw)

        x_mm = start_xyz_m[0] * 1000.0
        y_mm = start_xyz_m[1] * 1000.0
        z_mm = start_xyz_m[2] * 1000.0

        print(f"[INFO] Going to START pose: XYZ(mm)=({x_mm:.1f}, {y_mm:.1f}, {z_mm:.1f}), "
              f"RPY(deg)=({roll_deg:.1f}, {pitch_deg:.1f}, {yaw_deg:.1f})")

        code = arm.set_position(
            x=x_mm, y=y_mm, z=z_mm,
            roll=roll_deg, pitch=pitch_deg, yaw=yaw_deg,
            speed=100, mvacc=2000, is_radian=False, wait=True
        )
        if code != 0:
            print(f"[ERROR] set_position failed, code={code}, err={arm.error_code}, warn={arm.warn_code}")
            return
        print("[OK] Reached start pose")

        # ---- Switch to velocity mode (VC) and probe along BASE Z ----
        # Enter VC
        # clear any stop/error first
        try:
            if getattr(arm, "has_error", False):
                print("[WARN] Arm has_error → clean_error()")
                arm.clean_error()
            if getattr(arm, "has_warn", False):
                print("[WARN] Arm has_warn  → clean_warn()")
                arm.clean_warn()
        except Exception:
            pass

        arm.set_mode(5); time.sleep(0.1)
        arm.set_state(0); time.sleep(0.1)

        v = abs(PROBE_SPEED_MM_S)
        vec = [0.0, 0.0, BASE_Z_SIGN * v, 0.0, 0.0, 0.0]  # base/world frame “down”
        print(f"[INFO] Starting VC probe: vec={vec} (base frame)")

        code = arm.vc_set_cartesian_velocity(vec, is_tool_coord=False)
        if code != 0:
            print(f"[ERROR] vc_set_cartesian_velocity failed (code={code}), state={arm.state} "
                  f"(err={arm.error_code}, warn={arm.warn_code})")
            print("[HINT] If state=4 (stopped), clear error/warn and try again.")
            return

        # Monitor torque until threshold
        t0 = time.time()
        contact = False
        last_tau = None
        while time.time() - t0 < PROBE_TIMEOUT_S:
            code, torq = arm.get_joints_torque()
            if code == 0 and isinstance(torq, (list, tuple)) and len(torq) >= 3:
                last_tau = torq[2]
                # trigger when τ3 >= threshold (threshold is negative)
                if last_tau >= TORQUE_THRESH_NM:
                    print(f"[INFO] Contact detected: τ3={last_tau:.2f} Nm (>= {TORQUE_THRESH_NM:.2f})")
                    contact = True
                    break
            time.sleep(SLEEP_DT)

        # Stop VC
        arm.vc_set_cartesian_velocity([0, 0, 0, 0, 0, 0])
        arm.set_mode(0); time.sleep(0.05)
        arm.set_state(0); time.sleep(0.05)

        if not contact:
            print(f"[WARN] Probe timeout ({PROBE_TIMEOUT_S}s). Last τ3={last_tau}")
        else:
            # Retract upward in base Z by RETRACT_MM
            code, pose = arm.get_position(is_radian=True)  # [x,y,z,rx,ry,rz]
            if code == 0 and pose and len(pose) >= 6:
                x, y, z, rx, ry, rz = pose[:6]
                z_up = z + (-BASE_Z_SIGN) * RETRACT_MM  # up = opposite of down
                print(f"[INFO] Retracting {RETRACT_MM} mm up to z={z_up:.1f} mm …")
                arm.set_position(x, y, z_up, rx, ry, rz, is_radian=True, speed=50, wait=True)
                print("[OK] Retracted")
            else:
                print("[WARN] Could not read current pose to retract; skipping.")

        print("[DONE] Test complete.")

    finally:
        try:
            arm.vc_set_cartesian_velocity([0, 0, 0, 0, 0, 0])
        except Exception:
            pass
        try:
            arm.disconnect()
        except Exception:
            pass


if __name__ == "__main__":
    main()
