import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


  
def generate_launch_description():

  use_sim_time = LaunchConfiguration('use_sim_time', default='true')
 

  share_dir =  get_package_share_directory('rosa_description')
  world_file = os.path.join(share_dir,'worlds','pal_office.world')
  model_file = os.path.join(share_dir, 'description', 'rosa','rosa.xacro')
  urdf_content = xacro.process_file(model_file).toprettyxml()
  


  ld = LaunchDescription()
  ld.add_action(DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'))
            
  ld.add_action(ExecuteProcess(
            cmd=['gazebo', world_file , '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
            output='screen'))
  #'--verbose' add this as parameter of gazebo command to see more info          
  ld.add_action(Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='urdf_spawner',
            output='screen',
            arguments=["-topic", "/robot_description", "-entity", "rosa"]) )         
                   
  ld.add_action(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description':urdf_content, 'use_sim_time': use_sim_time }])) 
                         
  return ld
