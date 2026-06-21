from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
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
    use_robot = LaunchConfiguration('use_robot')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    common_nav2_arguments = {
        'params_file': params_file,
        'use_sim_time': use_sim_time,
        'autostart': autostart,
        'use_composition': 'False',
        'use_respawn': use_respawn,
        'log_level': log_level,
    }

    nav2_lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'collision_monitor',
        'bt_navigator',
        'waypoint_follower',
    ]

    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
    ]

    nav2_node_params = [
        params_file,
        {'use_sim_time': use_sim_time},
    ]

    description_launch = include_launch(
        'agv_description',
        'display.launch.py',
        use_robot,
    )

    bridge_launch = include_launch(
        'agv_bridge',
        'esp32_bridge.launch.py',
        use_robot,
    )

    lidar_launch = include_launch(
        'agv_lidar',
        'ld14.launch.py',
        use_robot,
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'localization_launch.py',
            ])
        ),
        launch_arguments={
            **common_nav2_arguments,
            'map': map_yaml,
        }.items(),
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=nav2_node_params,
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
    )

    collision_monitor = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=nav2_node_params,
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings,
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=nav2_node_params,
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings,
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=nav2_node_params,
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings,
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=nav2_node_params,
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings,
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=nav2_node_params,
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings,
    )

    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': nav2_lifecycle_nodes},
        ],
        arguments=['--ros-args', '--log-level', log_level],
    )

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        DeclareLaunchArgument(
            'map',
            default_value='/home/pi/agv_project/maps/agv_map.yaml',
            description='Full path to the saved occupancy grid map YAML.',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('agv_bringup'),
                'config',
                'nav2_params.yaml',
            ]),
            description='Full path to the Nav2 parameters file.',
        ),
        DeclareLaunchArgument(
            'use_robot',
            default_value='true',
            description='Start AGV description, ESP32 bridge, and lidar.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock. Keep false on the real robot.',
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically configure and activate Nav2 lifecycle nodes.',
        ),
        DeclareLaunchArgument(
            'use_respawn',
            default_value='false',
            description='Whether to respawn Nav2 nodes if they crash.',
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Nav2 log level.',
        ),
        description_launch,
        bridge_launch,
        lidar_launch,
        localization_launch,
        controller_server,
        planner_server,
        behavior_server,
        collision_monitor,
        bt_navigator,
        waypoint_follower,
        lifecycle_manager_navigation,
    ])
