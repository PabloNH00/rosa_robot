import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    
    #Get dynamic path of /launch
    current_launch_dir = os.path.dirname(os.path.realpath(__file__))

    map_dir = os.path.join(current_launch_dir, '../gz_slam_map', 'gaz_world.yaml') 
    
    slam_params_file = os.path.join(current_launch_dir, '../config', 'mapper_params_online_async.yaml')
    nav2_params_file = os.path.join(current_launch_dir, '../config', 'nav2_params.yaml')
    nav2_launcher_dir = os.path.join(
        get_package_share_directory('nav2_bringup'))

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    slam = LaunchConfiguration('slam', default='False')
    slam_params_file = LaunchConfiguration('slam_params', default = slam_params_file)
    nav2_params_file = LaunchConfiguration('params_file', default =nav2_params_file)
    map_dir = LaunchConfiguration('map', default = map_dir)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'slam',
            default_value='False',
            description='Execute SLAM for mapping if "True", else execute navigation'
        ),
        DeclareLaunchArgument(
            'slam_params',
            default_value=slam_params_file,
            description='Path to SLAM parms file'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=nav2_params_file,
            description='Path to params file'
        ),
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Path to map file'
        ),   

        # Nav2 launcher for navigation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_launcher_dir, '/launch/bringup_launch.py']
            ),
            launch_arguments={
                'map':map_dir,
                'use_sim_time': use_sim_time,
                'params_file': nav2_params_file,
                'slam': slam,
                'slam_params_file': 'slam_params'
            }.items(),
        ),

        # Rviz2 config
        Node(
            condition=LaunchConfigurationNotEquals('slam', 'False'),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(
                get_package_share_directory('rosa_description'), 'rviz', 'slam_rviz_config.rviz')]
        ),
        Node(
            condition=LaunchConfigurationEquals('slam', 'False'),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(
                get_package_share_directory('rosa_description'), 'rviz', 'nav2_rviz_config.rviz')]
        )
    ])


