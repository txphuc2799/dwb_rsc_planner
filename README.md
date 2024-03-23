# The integration of ROS2's Rotation Shim Controller with ROS's DWB Local Planner.

## Description
I noticed that ROS2 has a [Rotation Shim Controller](https://github.com/ros-planning/navigation2/tree/main/nav2_rotation_shim_controller) written by Steve Macenski, and its integration with the [ROS2 DWB Local Planner](https://github.com/ros-planning/navigation2/tree/main/nav2_dwb_controller) is highly effective. Therefore, I modified the [ROS DWB Local Planner](https://github.com/locusrobotics/robot_navigation/tree/noetic/dwb_local_planner) and added the Rotation Shim Controller. I successfully tested it in simulation.
## Install
```
$ cd your-catkin-workspace/src
$ git clone https://github.com/txphuc2799/dwb_rsc_planner.git
$ cd ..
$ rosdep install --from-paths src --ignore-src -r -y
$ catkin_make --pkg dwb_rsc_msgs dwb_rsc_local_planner dwb_rsc_critics dwb_rsc_plugins mir_dwb_rsc_critics
```
