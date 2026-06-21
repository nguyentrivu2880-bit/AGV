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
                'shaped_cmd_vel_topic': '/cmd_vel_shaped',
                'odom_topic': '/odom',

                'odom_frame': 'odom',
                'base_frame': 'base_footprint',

                'wheel_radius': 0.030,
                'wheel_base': 0.170,
                'cmd_wheel_base': 0.170,
                'odom_wheel_base': 0.187,
                'ticks_per_rev': 1975.0,

                'left_tick_sign': 1.0,
                'right_tick_sign': 1.0,

                'left_cmd_sign': 1.0,
                'right_cmd_sign': 1.0,

                'cmd_send_rate_hz': 20.0,
                'cmd_timeout_sec': 0.5,
                'force_zero_linear_on_spin': True,
                'spin_linear_deadband': 0.04,
                'spin_angular_deadband': 0.02,
                'linear_cmd_deadband': 0.01,
                'min_cmd_linear_x': 0.10,
                'min_cmd_angular_z': 0.27,
                'angular_floor_delay_sec': 0.35,
                'angular_sign_change_delay_sec': 0.40,
                'angular_reverse_release_z': 0.08,
                'cmd_linear_accel_limit': 0.20,
                'cmd_angular_accel_limit': 1.50,
            }]
        )
    ])
