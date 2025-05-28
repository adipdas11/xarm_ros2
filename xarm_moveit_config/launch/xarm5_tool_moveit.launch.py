#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='sim',
        description='Run mode: "sim" for simulation, "real" for real hardware'
    )
    
    enable_tool_arg = DeclareLaunchArgument(
        'enable_tool',
        default_value='true',
        description='Whether to launch the tool controller'
    )
    
    mode = LaunchConfiguration('mode')
    enable_tool = LaunchConfiguration('enable_tool')
    
    isaac_sim_joint_states = Node(
        package='xarm_isaac_joint_states',
        executable='xarm5_slider_tool_isaac_joint_states.py',
        output='screen'
    )

    slider_tf_node = Node(
        package='ufactory_linear_motor_description',
        executable='linear_motor_tf.py',
        output='screen'
    )
    
    slider_control_node = Node(
        package='ufactory_linear_motor_description',
        executable='linear_service_control.py',
        output='screen',
        parameters=[{'ip': '192.168.1.239'}],
    )
    
    xarm5_camera_calibration = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('xarm_isaac_joint_states'),
                'launch',
                'xarm5_camera_calibration_link5.launch.py'
            ])
        ),
    )
    
    tool_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('tool_controller'),
                'launch',
                'tool_control.launch.py'
            ])
        ),
        launch_arguments={
            'port': '/dev/ttyACM0',
            'baud': '115200',
        }.items(),
        condition=IfCondition(enable_tool)
    )
    
    fake_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('xarm_moveit_config'),
                'launch',
                'xarm5_tool_moveit_fake.launch.py'
            ])
        ),
        condition=IfCondition(PythonExpression(["'", mode, "' == 'sim'"]))
    )
    real_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('xarm_moveit_config'),
                'launch',
                'xarm5_tool_moveit_realmove.launch.py'
            ])
        ),
        condition=IfCondition(PythonExpression(["'", mode, "' == 'real'"]))
    )
    
    delayed_ready_pose = TimerAction(
        period=3.0,
        actions=[
                Node(
                    package='disassembly_xarm5',
                    executable='ready_pose_node.py',
                    output='screen'
            )
        ]
    )

    delayed_publisher = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'topic', 'pub',
                    '/linear_motor_joint_commands',
                    'sensor_msgs/msg/JointState',
                    "{name: ['slider_joint'], position: [0.0]}",
                    '--once'
                ],
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        slider_tf_node,
        slider_control_node,
        xarm5_camera_calibration,
        mode_arg,
        enable_tool_arg,
        isaac_sim_joint_states,
        fake_moveit,
        real_moveit,
        tool_controller,
        delayed_ready_pose,  
        delayed_publisher
    ])
