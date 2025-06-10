from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    # Declare a 'mode' argument to switch between sim and real
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='sim',
        description='Run mode: "sim" or "real"'
    )
    mode = LaunchConfiguration('mode')

    # RealSense camera node (only when mode == 'real')
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense_camera',
        output='screen',
        parameters=[{
            'enable_color': False,
            'spatial_filter.enable': True,
            'temporal_filter.enable': True
        }],
        condition=IfCondition(PythonExpression([mode, " == 'real'"]))
    )

    # Vision + graph builder node
    vision_graph_node = Node(
        package='disassembly_v3',
        executable='vision_graph_node.py',
        name='vision_graph',
        output='screen',
        parameters=[{'mode': mode}]
    )

    # Sequence planner node
    sequence_planner_node = Node(
        package='disassembly_v3',
        executable='sequence_planner_node.py',
        name='sequence_planner',
        output='screen'
    )

    # Pose transform service node
    pose_transform_node = Node(
        package='disassembly_v3',
        executable='pose_transform_service.py',
        name='pose_transform',
        output='screen'
    )

    # Velocity-servo action server
    velocity_servo_node = Node(
        package='disassembly_v3',
        executable='velocity_servo_action_server.py',
        name='velocity_servo',
        output='screen'
    )

    # Contact-approach action server
    contact_approach_node = Node(
        package='disassembly_v3',
        executable='contact_approach_action_server.py',
        name='contact_approach',
        output='screen'
    )

    # Unscrew action server
    unscrew_node = Node(
        package='disassembly_v3',
        executable='unscrew_action_server.py',
        name='unscrew',
        output='screen'
    )

    # High-level orchestrator node
    orchestrator_node = Node(
        package='disassembly_v3',
        executable='disassembly_orchestrator_node.py',
        name='orchestrator',
        output='screen'
    )

    return LaunchDescription([
        mode_arg,
        realsense_node,
        vision_graph_node,
        sequence_planner_node,
        pose_transform_node,
        velocity_servo_node,
        contact_approach_node,
        unscrew_node,
        orchestrator_node,
    ])
