# rosa_description

This include the xacro model for URDF besides other configuration and launcher files. This is also the main package for navigation and slam although it could be launched in different ways.

## Instal dependencies

    cd rosa_ws
    sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
    sudo apt install ros-humble-slam-toolbox
    rosdep install -i --from-path src

## config/
<details>
<summary>Expand to see folder's info</summary>
TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO


TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO
</details>

## description/rosa/
<details>
<summary>Expand to see folder's info</summary>

In this folder is located the xacro elements to represent ROSA in URDF format, with their Gazebo plugin if necessary.
Includes a "meshes/" folder with the mesh data for the LiDAR, camera, wheels and robot body.

The URDF model stablishes a transform tree headed by "base_footprint" frame, which correspond to "base_footprint" link.

### Links

- **base_footprint:** Auxiliar link to be the header frame of the URDF tf tree.
- **base_link:** Used to support the fixed transforms of components.
- **chasis:** Represents the body of ROSA.
- **lidar_sensor_link:** Represents the Hokuyo LiDAR.
- **camera_link:** Represents the Realsense RGBD camera. This is the top part of the transforms structure derivated from the specifications of RGBD's cameras.
- **wheel_front_left_link** & **wheel_front_right_link** & **wheel_back_left_link** & **wheel_fback_right_link:** Represent the four mecanum wheels of ROSA.

### Frames and joints

![Transform Tree](images/tf_tree.png)

Frames from top to bottom:
- **base_footprint:** 
  - Parent with fixed joint with **base_link**
  - Parent with continuous joint with the four **omni wheels**
- **base_link:**
  - Child with fixed joint with **base_footprint**
  - Parent with fixed joint with **chasis**
  - Parent with fixed joint with **lidar_sensor_link**
  - Parent with fixed joint with **camera_link**
 
- **wheel_front_left_link** & **wheel_front_right_link** & **wheel_back_left_link** & **wheel_fback_right_link:** 
  - Child with continuous joint with **base_link**
- **chasis:**
  - Child with fixed joint with **base_link**
- **lidar_sensor_link:**
  - Child with fixed joint with **base_link**
- **camera_link:**
  - Child with fixed joint with **base_link**
  - Parent with fixed joint with **camera_depth_frame**
  - Parent with fixed joint with **camera_color_frame**
- **camera_depth_frame:** 
  - Child with fixed joint with **camera_link**
  - Parent with fixed joint with **camera_depth_optical_frame:** Rotated on axis {-pi/2} 0 {-pi/2} due to the RGBD's specifications
- **camera_color_frame:** 
  - Child with fixed joint with **camera_link**
  - Parent with fixed joint with **camera_color_optical_frame:** Rotated on axis {-pi/2} 0 {-pi/2} due to the RGBD's specifications
- **camera_depth_optical_frame:** 
  - Child with fixed joint with **camera_depth_frame**
- **camera_color_frame:** 
  - Child with fixed joint with **camera_color_frame**

### Plugins and Gazebo

For the simulation in Gazebo there are plugins included in the corresponding xacros. 

lidar_sensor.xacro uses a type "ray" plugin configured to emulate the real hokuyo LiDAR.

realsense_d435.xacro uses a type "depth" plugin configured to emulate the real camera.

For the omni_wheel.xacro the omnidirectional movement plugin is located in the [gz_rosa_control](../gz_rosa_control/) package, also included in the project.

</details>

## gz_slam_map/
<details>
<summary>Expand to see folder's info</summary>

This folder contains all the files generated after map the gazebo world. It is set as default map for navigation in the launchers (gaz_wolrd.yaml)

![gaz_world.pgm](images/gazebo_map.png)

</details>

## launch/
<details>
<summary>Expand to see folder's info</summary>

This folder contains ROS2 launchers created for ROSA robot, SLAM and navigation. Most of them can be executed with differents arguments depending on the objective and they will launch processes on different ways to simulate on Gazebo or run the real robot. 

### rosa_gazebo_launch.py

Launches Gazebo program with the ROS2 parameter "use_sim_time" set to true. This launcher execute "spawn_entity.py", run the "robot_state_publisher" and load the [URDF model](description/rosa/).

Gazebo opens with preloaded URDF a world, which correspond to [pal_office.world](worlds/pal_office.world)

    ros2 launch rosa_description rosa_gazebo_launch.py

![rosa_gazebo_launch.py](images/rosa_gazebo_launch.png)

## rosa_urdf_launch.py

Load URDF model to work with ROS2 using real ROSA. Set "use_sim_time" param to false and run "robot_state_publisher" with the [xacro model](description/rosa/).

    ros2 launch rosa_description rosa_urdf_launch.py

## rosa_nav_slam_launch.py

It will run "bringup_launch.py", it will also execute a pre-configured rviz2 with all necessary components. This launcher is supposed to be used both for SLAM and for navigation. Localization with AMCL wil always be active to help the robot to map and to navigate.

It is necessary an active joint between the "odom" and "base_footprint" frames, so **one of the previous launcher should be executed first**

    ros2 launch rosa_description rosa_nav_slam_launch.py

### Launch Arguments

These are the specific launcher's arguments and their default value

* use_sim_time: false
* slam: False
* slam_params: [mapper_params_online_async.yaml](config/mapper_params_online_async.yaml)
* params_file: [nav2_params.yaml](config/nav2_params.yaml)
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

</details>

## rviz/
<details>
<summary>Expand to see folder's info</summary>

Preconfigured rviz configuration for navigation and SLAM with ROSA. 
They are included in the launchers to open rviz with the necessary displays and components to visualize.

### nav2_rviz_config.rviz

![rviz slam config.](images/rviz_navigation_config.png)

Displays included:
 - Grid
 - TF
 - RobotModel
 - LaserScan
 - PointCloud2
 - Camera
 - Map: unactive, for visualize preloaded map
 - Path: for navigation calculated paths
 - Map: for global costmap
 - Map: for local costmap
 - Nav2_rviz_plugin: to visualize navigation status
 
### slam_rviz_config.rviz

![rviz slam config.](images/rviz_slam_config.png)

Displays included:
 - Grid
 - TF
 - RobotModel
 - LaserScan
 - PointCloud2
 - Camera
 - Map: to visualize map while generating
 - SlamToolboxPlugin: to save generated map

</details>

