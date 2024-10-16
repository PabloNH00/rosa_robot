import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import LaunchConfigurationNotEquals

def generate_launch_description():

  use_sim_time = LaunchConfiguration('use_sim_time', default='false')  

  share_dir =  get_package_share_directory('rosa_description')
  model_file = os.path.join(share_dir, 'description', 'rosa','rosa.xacro')
  urdf_content = xacro.process_file(model_file).toprettyxml()
  urg_node2_dir = os.path.join(
        get_package_share_directory('urg_node2'))
  camera_dir = os.path.join(
        get_package_share_directory('realsense2_camera'))
  
  lidar_arg = DeclareLaunchArgument(
    'lidar',
    default_value='true',
    description='Execute urg_node2 for activate the lidar node'
  )

  camera_arg = DeclareLaunchArgument(
    'camera',
    default_value='true',
    description='Execute realsense node for activate the RGBD'
  )

  ld = LaunchDescription()
  ld.add_action(DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'))
  
  ld.add_action(lidar_arg)
  ld.add_action(camera_arg)

  ld.add_action(IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
                [urg_node2_dir, '/launch/urg_node2.launch.py']
    ),
    condition=LaunchConfigurationNotEquals('lidar', 'false'),
    )   
  )  


  ld.add_action(IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
                [camera_dir, '/launch/rs_launch.py']
    ),
    launch_arguments={
              'pointcloud.enable': "true",
            }.items(),
    condition=LaunchConfigurationNotEquals('camera', 'false'),
    )   
  )  

  ld.add_action(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description':urdf_content, 'use_sim_time': use_sim_time }])) 
                         
  return ld
