import os

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

#Unified launcher to Navigation with ROSA.
#Launch ROSA model and Navigation with amcl
  
def generate_launch_description():
  
  launchers_dir = os.path.join(
        get_package_share_directory('rosa_launchers_pkg'))
  
  rosa_description_dir = os.path.join(
        get_package_share_directory('rosa_description'))
  
  nav2_launcher_dir = os.path.join(
        get_package_share_directory('nav2_bringup'))
  
  #Lab map as default for ROSA navigation
  map_file = os.path.join(rosa_description_dir, '../../../../maps', 'rst.yaml')

  nav2_params_file = os.path.join(rosa_description_dir, 'config', 'nav2_params.yaml')

  nav2_params_file = LaunchConfiguration('params_file', default =nav2_params_file)
  map_dir = LaunchConfiguration('map', default = map_file)

  ld = LaunchDescription()

  #Launch arguments  
  ld.add_action(
    DeclareLaunchArgument(
        'params_file',
        default_value=nav2_params_file,
        description='Path to params file'
    )
  )

  ld.add_action(
    DeclareLaunchArgument(
        'map',
        default_value=map_dir,
        description='Path to map file'
    )
  )

  #rosa_gazebo launcher from rosa_launchers_pkg/launc
  ld.add_action(
    IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [launchers_dir, '/launch/rosa.launch.py']
        ),
    )
  )

  # Nav2 launcher for SLAM
  ld.add_action(
    IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [nav2_launcher_dir, '/launch/bringup_launch.py']
        ),
        launch_arguments={
            'use_sim_time': 'False',
            'params_file': nav2_params_file,
            'slam': 'False',
            'map': map_dir
        }.items(),
    ),
  )
  
  # Rviz2 config
  ld.add_action(
    Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(rosa_description_dir, 'rviz', 'nav2_rviz_config.rviz')]
    ),
  )
  
  return ld
