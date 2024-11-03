# LAUNCH FOLDER

This folder contains ROS2 launchers created for map and navigate with ROSA robot. Most of them can be executed with differents arguments depending on the objective and they will launch processes on different ways to simulate on Gazebo or run the real robot. 

**navigation_launch.py and slam_launch.py are not used and may be outdated, but they are in the folder for possible future uses**

## rosa_gazebo_launch.py

Launches Gazebo program with the ROS2 parameter "use_sim_time" set to true. This launcher execute "spawn_entity.py", run the "robot_state_publisher" and load the [URDF model](../description/rosa/).

Gazebo opens with preloaded URDF a world, which correspond to [pal_office.world](../worlds/pal_office.world)

    ros2 launch rosa_description rosa_gazebo_launch.py

![rosa_gazebo_launch.py](../images/rosa_gazebo_launch.png)

# rosa_urdf_launch.py

Load URDF model to work with ROS2 using real ROSA. Set "use_sim_time" param to false and run "robot_state_publisher" with the [xacro model](../description/rosa/).

    ros2 launch rosa_description rosa_urdf_launch.py

# rosa_nav_slam_launch.py

It will run "bringup_launch.py", it will also execute a pre-configured rviz2 with all necessary components. This launcher is supposed to be used both for SLAM and for navigation. Localization with AMCL wil always be active to help the robot to map and to navigate.

It is necessary an active joint between the "odom" and "base_footprint" frames, so **one of the previous launcher should be executed first with the rosa_driver node if using the real robot**

    ros2 launch rosa_description rosa_nav_slam_launch.py

## Launch Arguments

These are the specific launcher's arguments and their default value

* use_sim_time: false
* slam: False
* slam_params: [slam_params.yaml](../config/slam_params.yaml)
* params_file: [nav2_params.yaml](../config/nav2_params.yaml)
* map: gaz_world.yaml

**Lidar topic /scan should always be active to properly navigate or doing SLAM**

## SLAM
This way of launching rosa_nav_slam_launch.py execute **bringup_launch.py,** from the nav2_bringup package, which launch **slam_launch.py** using **online_sync_launch.py** from the slam_toolbox package.

    ros2 launch rosa_description rosa_nav_slam_launch.py slam:=True

If you are using Gazebo and **not in the NUC of the robot**, it is recomended to edit slam_launch.py to launch online_async_launch.py. It is due to the async launcher is faster at the cost of a loss of quality on the map that does not really affect in the simulation.

  sudo gedit /opt/ros/humble/share/nav2_bringup/launch/slam_launch.py

Start moving the robot publishing in /cmd_vel or using goal pose and save the map using the SLAM plug-in openned in rviz2 
* "save map" for .pgm and .yaml (necessary for navigation)
* "serialize map" for serialized version (.data and .posegraph)

Once you have your map files you can set it for navigation using launch arguments:

    ros2 launch rosa_description rosa_nav_slam_launch.py map:=(map.yaml path)
    
## Navigation
This way launches **bringup_launch.py,** from the nav2_bringup package.

    ros2 launch rosa_description rosa_nav_slam_launch.py map:=(map.yaml path)

The default map in this launcher is [gaz_world.yaml](../gz_slam_map/gaz_world.yaml) and you have to use parameters to change it

The initial pose of the robot is set as a parameter to (0, 0, 0, 0) in [nav2_params.yaml](../config/nav2_params.yaml). If it is not accurate set another initial pose using RVIZ.
Start navigation by setting a goal pose. If navigation is working properly, "navigation" and "localization" should be shown as "active" down left the screen.

While the robot is moving the path planned should appear in the RVIZ visualizer. You can stop the navigation using the RVIZ nav2 plugin. You can also interrupt the movement for a moment if working with the real robot by using the remote controller but, either stopping or moving it to another place, the navigation planner will resume the movement to the goal pose once the controller is no loger intervening.

