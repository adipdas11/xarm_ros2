""" Static transform publisher acquired via MoveIt 2 hand-eye calibration """
""" EYE-IN-HAND: link_tcp -> camera_color_optical_frame """
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
                "link_tcp",
                "--child-frame-id",
                "camera_color_optical_frame",
                "--x",
                "0.0679",
                "--y",
                "-0.015",
                "--z",
                "-0.175",
                "--qx",
                "0.00131488",
                "--qy",
                "0.00087582",
                "--qz",
                "0.645235",
                "--qw",
                "0.763982",
                # "--roll",
                # "0.000878871",
                # "--pitch",
                # "0.00303504",
                # "--yaw",
                # "1.40266",
            ],
        ),
    ]
    return LaunchDescription(nodes)
