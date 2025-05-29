#!/usr/bin/env python3
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SequenceDistributor(Node):
    def __init__(self):
        super().__init__('sequence_distributor')

        # Subscribers
        self.create_subscription(
            String,
            '/disassembly_sequence',
            self.on_sequence,
            10
        )

        # Publishers
        self.pub_tooling = self.create_publisher(String, '/tooling_arm_tasks', 10)
        self.pub_manip   = self.create_publisher(String, '/manipulation_arm_tasks', 10)
        self.pub_plan    = self.create_publisher(String, '/distributed_plan', 10)

        self.get_logger().info("✅ Sequence Distributor ready.")

    def on_sequence(self, msg: String):
        try:
            seq = json.loads(msg.data)
            if not isinstance(seq, list):
                raise ValueError
        except Exception:
            self.get_logger().warn("Received invalid sequence JSON")
            return

        tooling = []
        manip   = []
        plan    = []

        for part in seq:
            if part.startswith('screw_'):
                arm = 'tooling'
                # tooling arm: unscrew then drop
                tooling.extend([f"unscrew_{part}", f"drop_{part}"])
                plan.append({'arm': arm, 'part': part, 'action': 'unscrew'})
                plan.append({'arm': arm, 'part': part, 'action': 'drop'})

            elif part.startswith(('lid_','pcb_')):
                arm = 'manipulation'
                # manipulation arm: remove then drop
                manip.extend([f"remove_{part}", f"drop_{part}"])
                plan.append({'arm': arm, 'part': part, 'action': 'remove'})
                plan.append({'arm': arm, 'part': part, 'action': 'drop'})

            else:
                # unknown default to manipulation
                arm = 'manipulation'
                manip.extend([f"handle_{part}", f"drop_{part}"])
                plan.append({'arm': arm, 'part': part, 'action': 'handle'})
                plan.append({'arm': arm, 'part': part, 'action': 'drop'})

        # publish tooling arm task list
        msg_tooling = String(data=json.dumps(tooling))
        self.pub_tooling.publish(msg_tooling)

        # publish manipulation arm task list
        msg_manip = String(data=json.dumps(manip))
        self.pub_manip.publish(msg_manip)

        # publish full distributed plan
        msg_plan = String(data=json.dumps(plan))
        self.pub_plan.publish(msg_plan)

        self.get_logger().info(f"🔄 Distributed plan: {plan}")

def main(args=None):
    rclpy.init(args=args)
    node = SequenceDistributor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
