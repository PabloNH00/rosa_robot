# rosa_driver
ROS2 node for controlling the ROSA robot

Serial_driver package depends on "ros-humble-asio-cmake-module", if it is not installed you have to install it manually by doing:

    sudo apt install ros-humble-asio-cmake-module

Once installed, update the remaining dependencies and build the package to launch

    sudo rosdep update
    rosdep install --from-paths src -y --ignore-src
    colcon build

To connect ROSA's firmware to ROS2, necessary to make a joint between "odom" frame and "base_footprint", execute the following command

    ros2 run rosa_firmware rosa_driver --ros-args -p "device_name:=/dev/ttyUSB2"

If everything is running propperly, you should see the odom pose by running this command

    ros2 topic echo /odom --field pose.pose

The rosa_driver node receives odom messages through the serial port once firmware calculates the robot position with its kinematics model, and then publishes them in the /odom topic. This node also create the joint between odom and the transform tree of ROSA

