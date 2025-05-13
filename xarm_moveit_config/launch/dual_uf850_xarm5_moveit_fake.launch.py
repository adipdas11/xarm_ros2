#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    hw_ns = LaunchConfiguration('hw_ns', default='ufactory')
    
    
    isaac_sim_joint_states = Node(
        package='xarm_isaac_joint_states',
        executable='dual_xarm5_uf850_isaac_joint_states.py',
        output='screen'
    )
    
    # robot moveit fake launch
    # xarm_moveit_config/launch/_dual_robot_moveit_fake.launch.py
    robot_moveit_fake_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_moveit_config'), 'launch', '_dual_robot_moveit_fake.launch.py'])),
        launch_arguments={
            'dof_1': '6',
            'robot_type_1': 'uf850',
            'attach_to_1': 'world',
            'attach_xyz_1': '0 0 0',
            'attach_rpy_1': '0 0 0',
            
            'dof_2': '5',
            'robot_type_2': 'xarm',
            'attach_to_2': 'world',
            'attach_xyz_2': '0 1 0',
            'attach_rpy_2': '0 0 0',
            'add_gripper_2': 'true',
            'add_realsense_d435i_2': 'true',
            
            'hw_ns': hw_ns,
            'no_gui_ctrl': 'true',
        }.items(),
    )
    
    return LaunchDescription([
        robot_moveit_fake_launch,
        isaac_sim_joint_states
    ])
