# LAUNCH FOLDER

This folder contains ROS2 launchers created for ROSA robot, SLAM and navigation. Most of them can be executed with differents arguments depending on the objective and they will launch processes on different ways to simulate on Gazebo or run the real robot. 

## rosa_gazebo_launch.py

Launches Gazebo program with the ROS2 parameter "use_sim_time" set to true. This launcher execute "spawn_entity.py", run the "robot_state_publisher" and load the [URDF model](../description/rosa/).

Gazebo opens with preloaded URDF a world, which correspond to [pal_office.world](../worlds/pal_office.world)

    ros2 launch rosa_description rosa_gazebo_launch.py

![rosa_gazebo_launch.py](../images/rosa_gazebo_launch.png)

## rosa_urdf_launch.py

Load URDF model to work with ROS2 using real ROSA. Set "use_sim_time" param to false and run "robot_state_publisher" with the [xacro model](../description/rosa/).

    ros2 launch rosa_description rosa_urdf_launch.py

## rosa_nav_slam_launch.py

It will run "bringup_launch.py", it will also execute a pre-configured rviz2 with all necessary components. This launcher is supposed to be used both for SLAM and for navigation. Localization with AMCL wil always be active to help the robot to map and to navigate.

It is necessary an active joint between the "odom" and "base_footprint" frames, so **one of the previous launcher should be executed first**

    ros2 launch rosa_description rosa_nav_slam_launch.py

### Launch Arguments

These are the specific launcher's arguments and their default value

* use_sim_time: false
* slam: False
* slam_params: [mapper_params_online_async.yaml](../config/mapper_params_online_async.yaml)
* params_file: [nav2_params.yaml](../config/nav2_params.yaml)
* map: gaz_world.yaml

**Lidar topic /scan should always be active to properly navigate or doing SLAM**

### SLAM

    ros2 launch rosa_description rosa_nav_slam_launch.py slam:=True

Start moving the robot publishing in /cmd_vel or using goal pose and save the map using the SLAM plug-in openned in rviz2 
* "save map" for .pgm and .yaml (necessary for navigation)
* "serialize map" for serialized version (.data and .posegraph)

Once you have your map files you can set it for navigation using launch arguments:

    ros2 launch rosa_description rosa_nav_slam_launch.py map:=(map.yaml path)

Or by modifying "nav2_params.yaml" and changing "yaml_filename" param with your path to the map file:

    map_server:
        ros__parameters:
            use_sim_time: True
            # Overridden in launch by the "map" launch configuration or provided default value.
            # To use in yaml, remove the default "map" value in the tb3_simulation_launch.py file & provide full path to map below.
            yaml_filename: "${map without .yaml}"
    
### Navigation

    ros2 launch rosa_description rosa_nav_slam_launch.py

Set the initial pose of the robot to initialize the map<->odom transform and start navigation by setting a goal pose. If navigation is working properly, "navigation" and "localization" should be shown as "active" down left the screen.

While the robot is moving the path planned should appear in the RVIZ visualizer. You can stop the navigation using the RVIZ nav2 plugin. You can also interrupt the movement if working with the real robot by using the remote controller but, either stopping or moving it to another place, the navigation planner will resume the movement to the goal pose once the controller is no loger intervening.
