from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindExecutable
import launch.conditions as conditions
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    #pkg_share = get_package_share_directory("turtlebot_flatland")
    pkg_share = FindPackageShare("turtlebot_flatland")

    global_frame_id = LaunchConfiguration("global_frame_id")
    min_obstacle_height = LaunchConfiguration("min_obstacle_height")
    max_obstacle_height = LaunchConfiguration("max_obstacle_height")

    initial_pose_x = LaunchConfiguration("initial_pose_x")
    initial_pose_y = LaunchConfiguration("initial_pose_y")
    initial_pose_a = LaunchConfiguration("initial_pose_a")

    world_path = LaunchConfiguration("world_path")
    update_rate = LaunchConfiguration("update_rate")
    step_size = LaunchConfiguration("step_size")
    show_viz = LaunchConfiguration("show_viz")
    viz_pub_rate = LaunchConfiguration("viz_pub_rate")
    use_sim_time = LaunchConfiguration("use_sim_time")

    ld = LaunchDescription(
        [
            DeclareLaunchArgument(name="laser_topic", default_value="scan"), # default laser topic in flatland
            DeclareLaunchArgument(name="odom_topic", default_value="odom"),
            DeclareLaunchArgument(name="odom_frame_id", default_value="odom"),
            DeclareLaunchArgument(name="base_frame_id", default_value="base"),
            DeclareLaunchArgument(name="global_frame_id", default_value="map"),
            # Name of the map to use (without path nor extension) and initial position
            DeclareLaunchArgument(name="initial_pose_x", default_value="3.0"),
            DeclareLaunchArgument(name="initial_pose_y", default_value="7.0"),
            DeclareLaunchArgument(name="initial_pose_a", default_value="0.0"),
            DeclareLaunchArgument(name="min_obstacle_height", default_value="0.0"),
            DeclareLaunchArgument(name="max_obstacle_height", default_value="5.0"),

            # ******************** flatland********************
            # You can override these default values:
            #   roslaunch flatland_Server server.launch world_path:="/some/world.yaml" initial_rate:="30.0"
            DeclareLaunchArgument(
                name="world_path",
                default_value=PathJoinSubstitution([pkg_share, "maps/hospital_section.world.yaml"]),
            ),
            DeclareLaunchArgument(name="update_rate", default_value="100.0"),
            DeclareLaunchArgument(name="step_size", default_value="0.01"),
            DeclareLaunchArgument(name="show_viz", default_value="true"),
            DeclareLaunchArgument(name="viz_pub_rate", default_value="30.0"),
            DeclareLaunchArgument(name="use_sim_time", default_value="true"),

            SetEnvironmentVariable(name="ROSCONSOLE_FORMAT", value="[${severity} ${time} ${logger}]: ${message}"),

            # launch flatland server
            Node(
                name="flatland_server",
                package="flatland_server",
                executable="flatland_server",
                output="screen",
                parameters=[
                    # Use the arguments passed into the launchfile for this node
                    {"world_path": world_path},
                    {"update_rate": update_rate},
                    {"step_size": step_size},
                    {"show_viz": show_viz},
                    {"viz_pub_rate": viz_pub_rate},
                    {"use_sim_time": use_sim_time},
                ],
            ),

            # ***************** Robot Model *****************
            ExecuteProcess(
                cmd=[[
                    FindExecutable(name='ros2'),
                    " service ",
                    "call ",
                    "/spawn_model ",
                    "flatland_msgs/srv/SpawnModel ",
                    "\"{yaml_path: '", PathJoinSubstitution([pkg_share, 'robot/turtlebot.model.yaml']),
                    "', name: 'turtlebot0', ns: '', pose: ",
                    "{x: ", initial_pose_x, ", y: ", initial_pose_y, ", theta: ", initial_pose_a, "}}\"",
                ]],
                shell=True,
            ),

            # ****** Maps *****
            Node(
                name="map_server",
                package="nav2_map_server",
                executable="map_server",
                output='screen',
                parameters=[
                    {"yaml_filename": PathJoinSubstitution([pkg_share, "maps/hospital_section.yaml"])},
                    {"frame_id": global_frame_id},
                ],
            ),
            Node(
                name='lifecycle_manager_navigation',
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                output='screen',
                parameters=[
                    {'autostart': False},
                    {'node_names': ["map_server"]},
                ],
            ),
            Node(
                name="tf",
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
            ),

            # ***************** Manually setting some parameters *************************
            DeclareLaunchArgument(name="move_base/local_costmap/obstacle_layer/scan/min_obstacle_height", default_value=min_obstacle_height),
            DeclareLaunchArgument(name="move_base/local_costmap/obstacle_layer/scan/max_obstacle_height", default_value=max_obstacle_height),
            DeclareLaunchArgument(name="move_base/global_costmap/obstacle_layer/scan/min_obstacle_height", default_value=min_obstacle_height),
            DeclareLaunchArgument(name="move_base/global_costmap/obstacle_layer/scan/max_obstacle_height", default_value=max_obstacle_height),

            # **************** Visualisation ****************
            Node(
                name="rviz",
                package="rviz2",
                executable="rviz2",
                arguments=["-d", PathJoinSubstitution([pkg_share, "rviz/robot_navigation.rviz"])],
                parameters=[{"use_sim_time": use_sim_time}],
                condition=conditions.IfCondition(show_viz),
            ),
        ]
    )
    return ld


if __name__ == "__main__":
    generate_launch_description()
