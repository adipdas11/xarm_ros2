from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments for serial port and baud rate
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

    return LaunchDescription([
        port_arg,
        baud_arg,
        Node(
            package='tool_controller', 
            executable='tool_control.py',
            name='tool_control',
            output='screen',
            parameters=[
                {'port': LaunchConfiguration('port'),
                 'baud': LaunchConfiguration('baud')}
            ]
        )
    ])
