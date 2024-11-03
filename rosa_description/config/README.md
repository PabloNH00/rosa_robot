# CONFIG FOLDER

This folder contains the parameters file for navigation and SLAM. Here are the explanation for the most important and changed parameters for the development.

**load_pre_mapped_params_online_async.yaml is not used and may be outdated, but it is in the folder for possible future uses**

## slam_params.yaml

This params file is a copy paste from the "slam_toolbox/config/mapper_params_online_async.yaml" (same as slam_toolbox/config/mapper_params_online_sync.yaml), but is included in the package to be modified easily if necessary.

## nav2_params_yaml

The default params file from nav2_bringup is modified for this project rquirements. Mainly to change from differential to omnidirectional movement, so most of the changes are refered to velocities and accelerations, but also to adapt the navigation to the robot size, components and enviroment.

Here are some of the most important changes. If no coment is following the value it is full added, either the comented value is the default one:

```yaml
amcl:
  ros__parameters:
    laser_max_range: 30.0 #100
    robot_model_type: "nav2_amcl::OmniMotionModel" #"nav2_amcl::DifferentialMotionModel"
    set_initial_pose: true
    initial_pose: { x: 0.0, y: 0.0, z: 0.0, yaw: 0.0 }

controller_server:
  ros__parameters:
    min_y_velocity_threshold: 0.001
    FollowPath:
      min_vel_x: -0.05 #0.0
      min_vel_y: -0.15 #0.0
      max_vel_x: 0.15 #0.26
      max_vel_y: 0.15 #0.0
      max_vel_theta: 0.25 #1.0
      min_speed_xy: 0.001 #0.0
      max_speed_xy: 0.15 #0.26
      min_speed_theta: 0.01 #0.0
      acc_lim_y: 2.5 #0.0
      decel_lim_y: -2.5 #0.0
      vy_samples: 40 #5
      critics: ["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist", "Twirling"] #Same without "Twirling"

local_costmap:
  local_costmap:
    ros__parameters:
      footprint: "[[0.35, 0.295], [0.35, -0.295], [-0.35, -0.295], [-0.35, 0.295]]" #robot_radius: 0.46

map_server:
  ros__parameters:
    use_sim_time: True
    # Overridden in launch by the "map" launch configuration or provided default value.
    # To use in yaml, remove the default "map" value in the tb3_simulation_launch.py file & provide full path to map below.
    yaml_filename: "../gz_slam_map/gaz_world"


behavior_server:
  ros__parameters:
    max_rotational_vel: 0.25 #1.0
    min_rotational_vel: 0.001 #0.4

velocity_smoother:
  ros__parameters:
    max_velocity: [0.15, 0.15, 0.25] #[0.26, 0.0, 1.0]
    min_velocity: [-0.05, -0.15, -0.25] #[-0.26, 0.0, -1.0]
    max_accel: [1.0, 1.0, 1.5] #[2.5, 0.0, 3.2]
    max_decel: [-1.0, -1.0, -1.5] #[-2.5, 0.0, -3.2]
```
Other parameters such as inflation layers of the maps are also modified but must be adjusted depending on the purpose and enviroment.

As shown in the modifications, most of them are related to allow horizontal movement, decreasing minimum thresholds and increasing maximum velocities.

DWBLocalPlanner is selected due to its performance during tests.
