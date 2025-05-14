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
import os

def generate_launch_description():
    hw_ns = LaunchConfiguration('hw_ns', default='ufactory')

    # robot moveit real launch
    # xarm_moveit_config/launch/_dual_robot_moveit_realmove.launch.py
    robot_moveit_fake_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_moveit_config'), 'launch', '_dual_robot_moveit_realmove.launch.py'])),
        launch_arguments={
            'robot_ip_1': '192.168.1.123',
            'dof_1': '6',
            'robot_type_1': 'uf850',
            'attach_to_1': 'world',
            'attach_xyz_1': '0 0 0',
            'attach_rpy_1': '0 0 0',
            'add_other_geometry_1':'true',   
            'geometry_type_1':'mesh',   
            'geometry_mesh_filename_1':'ElectricScrew_EndEffector.stl',   
            'geometry_mesh_origin_xyz_1':"0 0 0",   
            'geometry_mesh_origin_rpy_1':"0 3.14 0",   
            'geometry_mesh_tcp_xyz_1':"0 0 0.176",   
            'geometry_mesh_tcp_rpy_1':"0 0 0",

            'robot_ip_1': '192.168.1.239',
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
    ])
