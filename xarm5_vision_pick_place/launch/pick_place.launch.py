#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare 'mode' launch argument (sim or real)
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='sim',
        description="Camera mode: 'sim' uses /xarm5/D435_1/... topics, 'real' uses /camera/... topics"
    )
    mode = LaunchConfiguration('mode')

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

    # Assemble and return the LaunchDescription
    ld = LaunchDescription([
        mode_arg,
        cube_detect_node,
        pick_place_timer,
    ])
    return ld
