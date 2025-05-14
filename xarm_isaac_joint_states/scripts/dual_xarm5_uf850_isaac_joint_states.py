#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointProcessor(Node):
    def __init__(self):
        super().__init__('joint_processor')
        # Subscriber to the raw joint_states
        self.sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
        # Publishers for the two output command topics
        self.pub_xarm5 = self.create_publisher(JointState, '/xarm5_joint_command', 10)
        self.pub_uf850 = self.create_publisher(JointState, '/uf850_joint_command', 10)

    def joint_state_callback(self, msg: JointState):
        # Build a name->position map for quick lookup
        pos_map = {name: pos for name, pos in zip(msg.name, msg.position)}

        # --- Right arm group (/xarm5_joint_command) ---
        # Extract & rename R_* joints
        r_order = ['R_joint1', 'R_joint2', 'R_joint3', 'R_joint4', 'R_joint5']
        r_positions = [pos_map.get(f'R_joint{i}', 0.0) for i in range(1,6)]
        drive_pos = pos_map.get('R_drive_joint', 0.0)

        # Synthesize knuckle/finger joints by flipping sign on drive_pos
        left_inner_knuckle__joint  = -drive_pos
        right_outer_knuckle_joint  =  drive_pos
        right_inner_knuckle__joint =  drive_pos
        left_finger_joint          =  drive_pos
        right_finger_joint         = -drive_pos

        # Fixed slider_joint = 0
        slider_joint = 0.0

        # Assemble and publish
        js_x = JointState()
        js_x.header.stamp = self.get_clock().now().to_msg()
        js_x.name = [
            'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'drive_joint',
            'left_inner_knuckle__joint',
            'right_outer_knuckle_joint',
            'right_inner_knuckle__joint',
            'left_finger_joint',
            'right_finger_joint',
            'slider_joint'
        ]
        js_x.position = (
            r_positions +
            [-drive_pos] +
            [
                left_inner_knuckle__joint,
                right_outer_knuckle_joint,
                right_inner_knuckle__joint,
                left_finger_joint,
                right_finger_joint,
                slider_joint
            ]
        )
        self.pub_xarm5.publish(js_x)

        # --- Left arm group (/uf850_joint_command) ---
        # Extract & rename L_* joints
        l_positions = [pos_map.get(f'L_joint{i}', 0.0) for i in [1,2,3,4,5,6]]
        # Fixed drill_bit_drive = 0
        drill_bit_drive = 0.0

        js_l = JointState()
        js_l.header.stamp = self.get_clock().now().to_msg()
        js_l.name = [
            'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6',
            'screw_bit_drive'
        ]
        js_l.position = l_positions + [drill_bit_drive]
        self.pub_uf850.publish(js_l)


def main(args=None):
    rclpy.init(args=args)
    node = JointProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
