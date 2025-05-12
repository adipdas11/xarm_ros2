#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    # Declare 'mode' launch argument
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='sim',
        description="Choose 'sim' to launch the fake MoveIt config, or 'real' to launch the real MoveIt + Realsense"
    )
    mode = LaunchConfiguration('mode')

    # Locate our package share
    pkg_share = get_package_share_directory('xarm_moveit_config')

    # Include the fake MoveIt launch (sim)
    fake_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'xarm5_linear_moveit_fake.launch.py')
        ),
        condition=IfCondition(
            PythonExpression(["'", mode, "' == 'sim'"])
        )
    )

    # Include the real MoveIt launch (real)
    real_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'xarm5_linear_moveit_realmove.launch.py')
        ),
        condition=IfCondition(
            PythonExpression(["'", mode, "' == 'real'"])
        )
    )

    # Include the Realsense driver only in real mode
    realsense_pkg = get_package_share_directory('realsense2_camera')
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_pkg, 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'depth_module.depth_profile': '1280x720x30',
            'pointcloud.enable': 'true'
        }.items(),
        condition=IfCondition(
            PythonExpression(["'", mode, "' == 'real'"])
        )
    )
    
     # ArUco cube detection node, takes 'mode' as parameter
    cube_detect_node = Node(
        package='xarm5_vision_pick_place',
        executable='aruco_cube_detection.py',
        name='aruco_cube_detection',
        output='screen',
        parameters=[ { 'mode': mode } ],
    )

    # Delay launching the ArUco pick & place state machine by 10 seconds
    pick_place_timer = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='xarm5_vision_pick_place',
                executable='aruco_pick_place.py',
                name='aruco_pick_place',
                output='screen'
            ),
        ],
    )

    return LaunchDescription([
        mode_arg,
        fake_moveit,
        real_moveit,
        realsense_launch,
        cube_detect_node,
        pick_place_timer,
    ])
