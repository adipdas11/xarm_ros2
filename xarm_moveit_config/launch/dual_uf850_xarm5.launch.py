#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    # Declare arguments
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='sim',
        description='Run mode: "sim" for simulation, "real" for real hardware'
    )
    enable_tool_arg = DeclareLaunchArgument(
        'enable_tool',
        default_value='false',
        description='Whether to launch the tool controller'
    )

    # Get configuration
    mode = LaunchConfiguration('mode')
    enable_tool = LaunchConfiguration('enable_tool')

    # Node for ISAAC sim joint states
    isaac_sim_joint_states = Node(
        package='xarm_isaac_joint_states',
        executable='dual_xarm5_uf850_isaac_joint_states.py',
        output='screen'
    )

    # Choose MoveIt launch based on mode
    fake_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('xarm_moveit_config'),
                'launch',
                'dual_uf850_xarm5_moveit_fake.launch.py'
            ])
        ),
        condition=IfCondition(PythonExpression(["'", mode, "' == 'sim'"]))
    )
    real_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('xarm_moveit_config'),
                'launch',
                'dual_uf850_xarm5_moveit_realmove.launch.py'
            ])
        ),
        condition=IfCondition(PythonExpression(["'", mode, "' == 'real'"]))
    )

    # Tool controller inclusion, conditional on enable_tool == true
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
    
    ready_pose = Node(
        package='disassembly_v2',
        executable='ready_pose_node.py',
        output='screen'
    )

    return LaunchDescription([
        mode_arg,
        enable_tool_arg,
        isaac_sim_joint_states,
        fake_moveit,
        real_moveit,
        tool_controller,
        ready_pose
    ])
