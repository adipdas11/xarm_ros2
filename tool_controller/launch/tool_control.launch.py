from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # ---- existing args ----
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyACM0',
        description='Serial port device for the Pico (e.g. /dev/ttyACM0)'
    )
    baud_arg = DeclareLaunchArgument(
        'baud',
        default_value='115200',
        description='Baud rate for the serial connection'
    )

    # ---- new: camera arg ----
    camera_index_arg = DeclareLaunchArgument(
        'camera_index',
        default_value='10',
        description='OpenCV camera index (integer, e.g. 6, 10)'
    )

    return LaunchDescription([
        port_arg,
        baud_arg,
        camera_index_arg,

        # Tool control node (kept as you wrote it)
        Node(
            package='tool_controller',
            executable='tool_control.py',   # if you have a console_script, use 'tool_control' instead
            name='tool_control',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baud': LaunchConfiguration('baud'),
            }]
        ),

        # Camera publisher node (publishes to /tool_camera, RELIABLE QoS)
        Node(
            package='tool_controller',
            executable='tool_camera.py',       # console_scripts entry point name
            name='tool_camera',
            output='screen',
            parameters=[{
                'camera_index': LaunchConfiguration('camera_index'),
            }]
        ),
    ])
