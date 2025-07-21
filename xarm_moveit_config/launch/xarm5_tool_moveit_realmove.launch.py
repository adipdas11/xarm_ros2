#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')

    robot_moveit_fake_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('xarm_moveit_config'),
            'launch',
            '_robot_moveit_realmove.launch.py'
        ])),
        launch_arguments={
            'robot_ip': '192.168.1.239',
            'dof': '5',
            'robot_type': 'xarm',
            'hw_ns': hw_ns,
            'no_gui_ctrl': 'true',
            'attach_to': 'robot_base',  
            'attach_xyz': '0 0 0',
            'attach_rpy': '0 0 0',
            'add_realsense_d435i': 'true',
            'linear_motor': 'true',
            'add_other_geometry':'true',   
            'geometry_type':'mesh',   
            'geometry_mesh_filename':'ElectricScrew_EndEffector.stl',   
            'geometry_mesh_origin_xyz':"0 0 0",   
            'geometry_mesh_origin_rpy':"0 3.14 3.14",   
            'geometry_mesh_tcp_xyz':"0 0 0.176",   
            'geometry_mesh_tcp_rpy':"0 0 0",
            'effort_control': 'true',
        }.items(),
    )
    
    
    return LaunchDescription([
        robot_moveit_fake_launch
    ])
