from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  #Include LDLidar launch file
  ldlidar_launch = IncludeLaunchDescription(
      launch_description_source=PythonLaunchDescriptionSource([
          get_package_share_directory('ldlidar_sl_ros2'),
          '/launch/ld14.launch.py'
      ])
  )

  # Define LaunchDescription variable
  ld = LaunchDescription()

  ld.add_action(ldlidar_launch)
  
  return ld
