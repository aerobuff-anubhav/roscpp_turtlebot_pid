# roscpp_turtlebot_pid

## Overview
This package implements the pid controller (so far only 'p' of the pid) on the linear and angular velocity of any ground robot, such as kobuki. The linear velocity control is only in x-direction while the angular velocity control is clockwise and counter-clockwise. Waypoints are taken from the setpoint generator in "nwu" frame.

## Installation
Create a catkin workspace for the project.
```
$ cd catkin_ws/src
```
Assuming that kobuki packages for respective ros-distro are installed on the firmware, git clone the dependencies in the workspace source folder 'catkin_ws/src'.
```
$ cd catkin_ws
$ catkin build
$ source devel/setup.bash
```

### Dependencies
- [reef_msgs]
- [ros_vrpn_client]
- [setpoint_generator]
- [kobuki]
- [kobuki_core]
- [kobuki_msgs]
- [turtlebot]


## Configuration
### Waypoints in setpoint_generator
This step is common for hardware and simulation. These ways points are meant for turtlebot since the z-coordinate is zero.
Path: setpoint_generator/waypoint_files/basic_waypoints.yaml
```
waypoint_list:
  [
    {position: [0.0, 0.0, 0.0], yaw_degrees: 0},
    {position: [0.0, 1.5, 0.0], yaw_degrees: 0},
    {position: [1.0, 3.0, 0.0], yaw_degrees: 90},
    {position: [0.0, 0.0, 0.0], yaw_degrees: 0}
  ]
```

### Coordinate Frame in setpoint_generator
- Set the following parameter manually in *setpoint_generator/params/basic_param.yaml*.

For turtlebot
```
robot_command_orientation: NWU
```
For copter
```
robot_command_orientation: NED
```

### Hardware
- Set the following parameters manually in *roscpp_turtlebot_pid/config/params.yaml*.
```
sim: false
mocap: true
safety_distance: 0.6
```
- Set the following parameters manually in *setpoint_generator/params/basic_param.yaml*.
```
is_sim: false
use_mocap: true
```
- Turn on the mocap system and create a rigid body by the name "tb3_1".
- ssh into the computer on the turtlebot.
```
$ roslaunch roscpp_turtlebot_pid firmware.launch
```

### Simulation
- Set the following parameters manually in *roscpp_turtlebot_pid/config/params.yaml*.
```
sim: true
mocap: false
safety_distance: 0.2
```
Terminal window 1
```
$ roscore
```
Terminal window 2
```
$ roslaunch gazebo_ros empty_world.launch
```
Terminal window 3
```
$ roslaunch roscpp_turtlebot_pid sim.launch
```

## Troubleshooting
**1. Turtlebot doesn't move, but I'm giving the command velocity.**

Check the mapping in the firmware.launch. It should be to /mobile_base/commands/velocity (of rosnode mobile_base_nodelet_managers)

## Credits

## Changelog


[reef_msgs]: http://192.168.1.101/infrastructure/reef_msgs
[setpoint_generator]: http://192.168.1.101/AVL-Summer-18/setpoint_generator
[ros_vrpn_client]: http://192.168.1.101/infrastructure/ros_vrpn_client
[kobuki]: https://github.com/yujinrobot/kobuki
[kobuki_core]: https://github.com/yujinrobot/kobuki_core
[kobuki_msgs]: https://github.com/yujinrobot/kobuki_msgs
[turtlebot]: https://github.com/turtlebot/turtlebot
