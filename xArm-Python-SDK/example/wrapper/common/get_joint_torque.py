# from xarm.wrapper import XArmAPI
# import time
# arm = XArmAPI('192.168.1.239', debug=False)

# arm.motion_enable(True)
# arm.set_mode(0)
# arm.set_state(0)
# arm.set_report_tau_or_i(1)    # 1 == current, or 0 for torque

# while True:
#     _, torques = arm.get_joints_torque()
#     effort = torques[2] 
#     print(f"Joint 3 torque: {effort:.2f} Nm")
#     time.sleep(0.5)
    
from xarm.wrapper import XArmAPI
import time

# --- CONFIGURATION ---
IP       = '192.168.1.239'
THRESH   = 0.1       # Nm — consider any change > THRESH as “movement”
STABLE_T = 5.0       # seconds before declaring “unscrewed”
INTERVAL = 0.1       # sampling interval (seconds)

# --- SETUP ---
arm = XArmAPI(IP, debug=False)
arm.motion_enable(True)
arm.set_mode(0)
arm.set_state(0)
arm.set_report_tau_or_i(1)    # 1 == current, or 0 for torque

# prime first reading
_, torques     = arm.get_joints_torque()
last_effort    = torques[2]
last_change_t  = time.time()

print(f"Starting monitoring joint 3 torque (stable threshold: ±{THRESH} Nm)")

# --- MONITOR LOOP ---
while True:
    _, torques = arm.get_joints_torque()
    effort     = torques[2]

    # has it “moved” more than our noise threshold?
    if abs(effort - last_effort) > THRESH:
        # reset the timer
        last_change_t = time.time()
        last_effort   = effort

    # how long since last significant change?
    elapsed = time.time() - last_change_t

    print(f"[{elapsed:5.2f}s] Joint 3 torque: {effort:.2f} Nm")

    # if torque has been effectively constant for STABLE_T, we’re done
    if elapsed >= STABLE_T:
        print("\n>>> Unscrewed!  No torque change detected for "
              f"{STABLE_T:.1f} seconds.\n")
        break

    time.sleep(INTERVAL)

