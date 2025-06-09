from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_disassembly_pkg', executable='vision_graph_node',
            name='vision_graph', output='screen',
            parameters=[{'mode':'sim'}]
        ),
        Node(
            package='my_disassembly_pkg', executable='sequence_planner_node',
            name='sequence_planner', output='screen'
        ),
        Node(
            package='my_disassembly_pkg', executable='pose_transform_service',
            name='pose_transform', output='screen'
        ),
        Node(
            package='my_disassembly_pkg', executable='velocity_servo_action_server',
            name='velocity_servo', output='screen'
        ),
        Node(
            package='my_disassembly_pkg', executable='contact_approach_action_server',
            name='contact_approach', output='screen'
        ),
        Node(
            package='my_disassembly_pkg', executable='unscrew_action_server',
            name='unscrew', output='screen'
        ),
        Node(
            package='my_disassembly_pkg', executable='disassembly_orchestrator_node',
            name='orchestrator', output='screen'
        ),
    ])
