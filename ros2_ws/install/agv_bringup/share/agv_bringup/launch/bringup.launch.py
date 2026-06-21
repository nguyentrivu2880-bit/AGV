from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def include_launch(package_name, launch_file, condition):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare(package_name),
                'launch',
                launch_file,
            ])
        ),
        condition=IfCondition(condition),
    )


def generate_launch_description():
    use_description = LaunchConfiguration('use_description')
    use_bridge = LaunchConfiguration('use_bridge')
    use_lidar = LaunchConfiguration('use_lidar')
    use_slam = LaunchConfiguration('use_slam')
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')

    description_launch = include_launch(
        'agv_description',
        'display.launch.py',
        use_description,
    )

    bridge_launch = include_launch(
        'agv_bridge',
        'esp32_bridge.launch.py',
        use_bridge,
    )

    lidar_launch = include_launch(
        'agv_lidar',
        'ld14.launch.py',
        use_lidar,
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('slam_toolbox'),
                'launch',
                'online_async_launch.py',
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file,
        }.items(),
        condition=IfCondition(use_slam),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_description',
            default_value='true',
            description='Start joint_state_publisher and robot_state_publisher.',
        ),
        DeclareLaunchArgument(
            'use_bridge',
            default_value='true',
            description='Start ESP32 wheel bridge and odometry publisher.',
        ),
        DeclareLaunchArgument(
            'use_lidar',
            default_value='true',
            description='Start LD14 lidar through agv_lidar.',
        ),
        DeclareLaunchArgument(
            'use_slam',
            default_value='true',
            description='Start slam_toolbox for mapping. Disable this before Nav2/AMCL.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock. Keep false on the real robot.',
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('slam_toolbox'),
                'config',
                'mapper_params_online_async.yaml',
            ]),
            description='slam_toolbox parameter file.',
        ),
        description_launch,
        bridge_launch,
        lidar_launch,
        slam_launch,
    ])
