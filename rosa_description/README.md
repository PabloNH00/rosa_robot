# README #

ROSA robot repository for the ROS2 Simulation package 

### How to run the simulation ###
    colcon build --symlink-install
    source install/local_setup.bash
    ros2 launch rosa_description rosa_gazebo_launch.py

### How to run ROSA ###
    colcon build --symlink-install
    source install/local_setup.bash
    ros2 launch rosa_description rosa_urdf_launch.py    

### How to run the navigation toolbox ###
To install slam-toolbox

    sudo apt install ros-humble-slam-toolbox

To install nav2

    sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
    sudo apt install ros-humble-turtlebot3*

After launch gazebo or URDF with ROSA launch the following launcher for the navigation functionalities:

    ros2 launch rosa_description rosa_nav_slam_launch.py




* Repo owner or admin
* Other community or team contact
