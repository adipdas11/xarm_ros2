""" Static transform publisher acquired via MoveIt 2 hand-eye calibration """
""" EYE-IN-HAND: link5 -> camera_color_optical_frame """
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    nodes = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="log",
            arguments=[
                "--frame-id",
                "link5",
                "--child-frame-id",
                "camera_color_optical_frame",
                "--x",
                "0.349715",
                "--y",
                "-0.0200713",
                "--z",
                "-1.0339",
                "--qx",
                "-0.00204973",
                "--qy",
                "0.00416799",
                "--qz",
                "0.717642",
                "--qw",
                "0.696397",
                # "--roll",
                # "3.13276",
                # "--pitch",
                # "3.13873",
                # "--yaw",
                # "-1.54074",
            ],
        ),
    ]
    return LaunchDescription(nodes)
