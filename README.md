# TurtleBot navigation in Flatland

This ROS 2 example drives a TurtleBot-sized robot in Flatland with Nav2. Build [Flatland](https://github.com/avidbots/flatland) and this package in the same colcon workspace. Install `navigation2`, `nav2_bringup`, `twist_mux`, and `rviz2` for your ROS distribution first.

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch turtlebot_flatland turtlebot_in_flatland.launch.py
```

The standalone XML example is also available with `ros2 launch turtlebot_flatland turtlebot_in_flatland.launch`. Both start the hospital world with the robot at `(3, 7)` and run Flatland, Nav2 with AMCL, stamped `twist_mux`, and RViz. The XML example uses Nav2's stock parameters and remaps its command output to the mux; the Python example also adjusts collision monitor scan height and robot radius. Set `show_viz:=false` to run headless.

The robot's `InitialPose` model plugin sends its starting map-frame pose to AMCL after a subscriber joins. Use **Nav2 Goal** in RViz to set a destination; **2D Pose Estimate** remains available if localization needs correcting. AMCL supplies `map -> odom`, Flatland's drive supplies `odom -> base`, and its model TF publisher supplies `base -> base_footprint`, `base -> base_link`, and the laser frame. Flatland also publishes `/scan`, `/odom`, and `/clock`.

`twist_mux` accepts Nav2's collision-checked `TwistStamped` commands on `/cmd_vel_nav` and higher-priority stamped teleoperation commands on `/cmd_vel_teleop`. It outputs `/cmd_vel` to Flatland; both inputs time out after 0.5 seconds.
