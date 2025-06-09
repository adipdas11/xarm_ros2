import matplotlib.pyplot as plt
import pandas as pd

# 1) Load the CSV produced by the node
df = pd.read_csv('/home/adip/workspaces/dev_ws/src/xarm_ros2/velocity_control/EffortCSV/efforts2.csv')

# 2) Plot each joint’s effort vs. time
plt.figure(figsize=(10, 6))
plt.plot(df['time'], df['joint1'], label='joint1')
plt.plot(df['time'], df['joint2'], label='joint2')
plt.plot(df['time'], df['joint3'], label='joint3')
plt.plot(df['time'], df['joint4'], label='joint4')
plt.plot(df['time'], df['joint5'], label='joint5')

plt.xlabel('Time (s)')
plt.ylabel('Effort')
plt.title('Joint Efforts Over Time')
plt.legend()
plt.grid(True)
plt.show()
