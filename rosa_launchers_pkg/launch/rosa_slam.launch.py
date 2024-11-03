import os

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

#Unified launcher to SLAM with ROSA.
#Launch ROSA model and SLAM
  
def generate_launch_description():
  
  gazebo_launcher_dir = os.path.join(
        get_package_share_directory('rosa_launchers_pkg'))
  
  rosa_description_dir = os.path.join(
        get_package_share_directory('rosa_description'))
  
  nav2_launcher_dir = os.path.join(
        get_package_share_directory('nav2_bringup'))
  
  #necessary to launch bringup but not used for slam
  map_file = os.path.join(rosa_description_dir, 'gz_slam_map', 'gaz_world.yaml')

  slam_params_file = os.path.join(rosa_description_dir, 'config', 'slam_params.yaml')
  nav2_params_file = os.path.join(rosa_description_dir, 'config', 'nav2_params.yaml')

  slam_params_file = LaunchConfiguration('slam_params', default = slam_params_file)
  nav2_params_file = LaunchConfiguration('params_file', default =nav2_params_file)

  ld = LaunchDescription()

  #Launch arguments
  ld.add_action(
    DeclareLaunchArgument(
        'slam_params',
        default_value=slam_params_file,
        description='Path to SLAM parms file'
    )
  )
  
  ld.add_action(
    DeclareLaunchArgument(
        'params_file',
        default_value=nav2_params_file,
        description='Path to params file'
    )
  )

  #rosa launcher from rosa_launchers_pkg/launc
  ld.add_action(
    IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [gazebo_launcher_dir, '/launch/rosa.launch.py']
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
            'slam': 'True',
            'slam_params_file': slam_params_file,
            'map': map_file #necessary to launch bringup but not used for slam
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
        arguments=['-d', os.path.join(rosa_description_dir, 'rviz', 'slam_rviz_config.rviz')]
    ),
  )
  
  return ld
