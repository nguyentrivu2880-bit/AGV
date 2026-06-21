from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='agv_bridge',
            executable='esp32_bridge_odom',
            name='esp32_bridge_odom',
            output='screen',
            parameters=[{
                'port': '/dev/ttyUSB0',
                'baudrate': 115200,

                'cmd_vel_topic': '/cmd_vel',
                'odom_topic': '/odom',

                'odom_frame': 'odom',
                'base_frame': 'base_footprint',

                'wheel_radius': 0.065,
                'wheel_base': 0.170,
                'ticks_per_rev': 1975.0,

                'left_tick_sign': 1.0,
                'right_tick_sign': 1.0,

                'left_cmd_sign': 1.0,
                'right_cmd_sign': 1.0,

                'cmd_send_rate_hz': 20.0,
                'cmd_timeout_sec': 0.5,
            }]
        )
    ])
