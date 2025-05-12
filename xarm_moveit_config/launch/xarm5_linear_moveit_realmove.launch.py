#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    robot_ip = LaunchConfiguration('robot_ip')
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    
    slider_tf_node = Node(
        package='ufactory_linear_motor_description',
        executable='linear_motor_tf.py',
        output='screen'
    )
    
    slider_control_node = Node(
        package='ufactory_linear_motor_description',
        executable='linear_service_control.py',
        output='screen',
        parameters=[{'ip': robot_ip}],
    )
    
    # robot moveit realmove launch
    # xarm_moveit_config/launch/_robot_moveit_realmove.launch.py
    robot_moveit_realmove_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_moveit_config'), 'launch', '_robot_moveit_realmove.launch.py'])),
        launch_arguments={
            'robot_ip': robot_ip,
            'dof': '5',
            'robot_type': 'xarm',
            'hw_ns': hw_ns,
            'no_gui_ctrl': 'true',
            'attach_to': 'robot_base',  
            'attach_xyz': '0 0 0',
            'attach_rpy': '0 0 0',
            'add_gripper': 'true',
            'add_realsense_d435i': 'true',
        }.items(),
    )
    
    xarm5_camera_calibration = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([
                    FindPackageShare('xarm_isaac_joint_states'),
                    'launch',
                    'xarm5_camera_calibration_link5.launch.py'
                ])
            ),
    )
    
    isaac_joint_states = Node(
        package='xarm_isaac_joint_states',
        executable='xarm5_slider_isaac_joint_states.py',
        output='screen'
    )
    
    # 1) After 5 seconds → launch the ready-pose file
    delayed_ready_pose = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([
                    FindPackageShare('xarm_planner'),
                    'launch',
                    'xarm5_ready_pose.launch.py'
                ])),
            )
        ]
    )

    # 2) After 7 seconds (i.e. 5 + 2) → publish once
    delayed_publisher = TimerAction(
        period=10.0,
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
        robot_moveit_realmove_launch,
        xarm5_camera_calibration,
        slider_control_node,
        isaac_joint_states,
        delayed_ready_pose,   
        delayed_publisher
    ])
