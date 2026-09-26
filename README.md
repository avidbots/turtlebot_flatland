# TurtleBot navigation in Flatland

This ROS 2 example drives a TurtleBot-sized robot in Flatland with Nav2. The demo
is currently supported on ROS 2 Lyrical only. Kilted and Rolling are expected to
work as well, but have not been verified.

[<img width="320" height="240" alt="FlatlandTrailer" src="https://github.com/user-attachments/assets/418f50e4-aa7e-402e-aa83-260e86ab07e9" />](https://youtu.be/NnZE7pkUSM8)

## Dev container (Lyrical)

With Docker and the VS Code Dev Containers extension installed, open this
repository in VS Code and select **Dev Containers: Reopen in Container**. The
container checks out [Flatland](https://github.com/avidbots/flatland) into
`/opt/overlay_ws/src/flatland` before using `rosdep` to install dependencies for
both repositories. The workspace is stored in a persistent container volume;
the demo repository remains mounted from your local checkout. Build from the
workspace root:

```bash
cd /opt/overlay_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch turtlebot_flatland turtlebot_in_flatland.launch.py
```

RViz requires a working X11 display on the host. To run without a display, add
`show_viz:=false` to the launch command.

## Manual setup

Build Flatland and this package in the same colcon workspace. Install
`navigation2`, `nav2_bringup`, `twist_mux`, and `rviz2` for your ROS distribution
first. From the workspace root:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch turtlebot_flatland turtlebot_in_flatland.launch.py
```

The standalone XML example is also available with `ros2 launch turtlebot_flatland turtlebot_in_flatland.launch`. Both start the hospital world with the robot at `(3, 7)` and run Flatland, Nav2 with AMCL, stamped `twist_mux`, and RViz. The XML example uses Nav2's stock parameters and remaps its command output to the mux; the Python example also adjusts collision monitor scan height and robot radius. Set `show_viz:=false` to run headless.

The Python launch uses the TurtleBot without a trailer by default. To load the
trailer version of the robot and world, run:

```bash
ros2 launch turtlebot_flatland turtlebot_in_flatland.launch.py trailer:=true
```

The robot's `InitialPose` model plugin sends its starting map-frame pose to AMCL after a subscriber joins. Use **Nav2 Goal** in RViz to set a destination; **2D Pose Estimate** remains available if localization needs correcting. AMCL supplies `map -> odom`, Flatland's drive supplies `odom -> base`, and its model TF publisher supplies `base -> base_footprint`, `base -> base_link`, and the laser frame. Flatland also publishes `/scan`, `/odom`, and `/clock`.

`twist_mux` accepts Nav2's collision-checked `TwistStamped` commands on `/cmd_vel_nav` and higher-priority stamped teleoperation commands on `/cmd_vel_teleop`. It outputs `/cmd_vel` to Flatland; both inputs time out after 0.5 seconds.
