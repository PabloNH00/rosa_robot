# rosa_launchers_pkg

This package unifies launchers of the other ones to make it east to launch all components and functionalities at once.
It is divided in ROSA launchers (rosa.launch.py, rosa_slam.launch.py and rosa_navigation.launch.py) to work with the real robot, and simulation launchers (rosa_gazebo.launch.py, rosa_gazebo_slam.launch.py and rosa_gazebo_navigation.launch.py) to work with the Gazebo simulation.

This package is intended to be a shortcut to work with ROSA if no specific components and functionalities need to be launched. All the processes that these launchers can run can also be runned with the individual launchers from the rest of the packages if the correct combination is done.

**NONE OF THESE LAUNCHERS ARE USED IN THE ROSA MONITOR**

## ROSA Launchers
All ROSA launchers launch the URDF model from [rosa_description package](../rosa_description/description/rosa/), the transform node for odometry from [rosa_firmware_package](../rosa_firmware/rosa_driver/rosa_driver.cpp) and by default the [LiDAR node](../urg_node2/launch/urg_node2.launch.py) and the [RGBD Camera node](../rosa_camera/realsense-ros/realsense2_camera/launch/rs_launch.py) with the pointcloud.enable argument set to true. 

rosa_slam and rosa_navigation launchers launch the default [rosa.launch.py](launch/rosa.launch.py) beside the **bringup_launch.py** from nav2 packages and the preconfigurated RVIZ2.

### Launch Arguments

- **rosa.launch.py**
  - "lidar": true to launch urg_node2. Default to true
  - "camera": true to launch realsense node. Default to true
  - All the urg_node2 and realsense launcher's nested arguments
- **rosa_slam.launch.py**
  - "slam_params": path to slam config. Default [mapper_aparms_online_async.yaml](../rosa_description/config/mapper_params_online_async.yaml)
  - "params_file": path to navigation config. Default [nav2_params.yaml](../rosa_description/config/nav2_params.yaml)
  - All the rosa.launch.py and bringup launcher's nested arguments
- **rosa_navigation.launch.py**
  - "map": path to map to navigate. Default laboratory map (Only in ROSA NUC)
  - "params_file": path to navigation config. Default [nav2_params.yaml](../rosa_description/config/nav2_params.yaml)
  - All the rosa.launch.py and bringup launcher's nested arguments

## Simulation Launchers
All simulation launchers launch the URDF model and gazebo from [rosa_description package](../rosa_description/description/rosa/). 

rosa_gazebo_slam and rosa_gazebo_navigation launchers launch the default [rosa_gazebo.launch.py](launch/rosa_gazebo.launch.py) beside the **bringup_launch.py** from nav2 packages and the preconfigurated RVIZ2.

### Launch Arguments

- **rosa_gazebo_slam.launch.py**
  - "slam_params": path to slam config. Default [mapper_aparms_online_async.yaml](../rosa_description/config/mapper_params_online_async.yaml)
  - "params_file": path to navigation config. Default [nav2_params.yaml](../rosa_description/config/nav2_params.yaml)
  - All the rosa_gazebo.launch.py and bringup launcher's nested arguments
- **rosa_gazebo_navigation.launch.py**
  - "map": path to map to navigate. Default [Gazebo map](../rosa_description/gz_slam_map/gaz_world.yaml)
  - "params_file": path to navigation config. Default [nav2_params.yaml](../rosa_description/config/nav2_params.yaml)
  - All the rosa_gazebo.launch.py and bringup launcher's nested arguments
