from xarm.wrapper import XArmAPI
import time
arm = XArmAPI('192.168.1.239', debug=False)

arm.motion_enable(True)
arm.set_mode(0)
arm.set_state(0)
arm.set_report_tau_or_i(1)    # 1 == current, or 0 for torque

while True:
    torques = arm.get_joints_torque()
    print(torques[1][0:5])
    time.sleep(0.5)
    
