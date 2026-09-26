from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_share = FindPackageShare("turtlebot_flatland")
    nav2_share = FindPackageShare("nav2_bringup")
    nav2_params = RewrittenYaml(
        source_file=PathJoinSubstitution([nav2_share, "params", "nav2_params.yaml"]),
        root_key="",
        param_rewrites={
            "cmd_vel_out_topic": "cmd_vel_safe",
            "min_height": "0.0",
            "robot_radius": "0.27",
            "wait_for_service_timeout": "10000",
            "alpha1": "0.05",
            "alpha2": "0.05",
            "alpha3": "0.05",
            "alpha4": "0.05",
            "alpha5": "0.05",
            "z_hit": "0.8",
            "z_rand": "0.2",
            "laser_likelihood_max_dist": "10.0",
            "update_min_d": "0.1",
            "update_min_a": "0.1",
            "local_costmap.local_costmap.ros__parameters.voxel_layer.observation_sources": "scan scan_3d",
            "global_costmap.global_costmap.ros__parameters.obstacle_layer.observation_sources": "scan scan_3d",
            **{
                f"{costmap}.{costmap}.ros__parameters.{layer}.scan_3d.{parameter}": value
                for costmap, layer in (
                    ("local_costmap", "voxel_layer"),
                    ("global_costmap", "obstacle_layer"),
                )
                for parameter, value in (
                    ("topic", "scan_3d"),
                    ("max_obstacle_height", "2.0"),
                    ("clearing", "true"),
                    ("marking", "true"),
                    ("data_type", "LaserScan"),
                    ("raytrace_max_range", "3.0"),
                    ("raytrace_min_range", "0.0"),
                    ("obstacle_max_range", "2.5"),
                    ("obstacle_min_range", "0.0"),
                )
            },
            # "xy_goal_tolerance": "0.5",
            # "yaw_goal_tolerance": "0.5",
        },
        convert_types=True,
    )
    flatland_server = Node(
        name="flatland_server",
        package="flatland_server",
        executable="flatland_server",
        output="screen",
        parameters=[
            {"world_path": LaunchConfiguration("world_path")},
            {"update_rate": LaunchConfiguration("update_rate")},
            {"step_size": LaunchConfiguration("step_size")},
            {"show_viz": LaunchConfiguration("show_viz")},
            {"viz_pub_rate": LaunchConfiguration("viz_pub_rate")},
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable("FASTDDS_BUILTIN_TRANSPORTS", "UDPv4"),
            DeclareLaunchArgument(
                name="world_path",
                default_value=PathJoinSubstitution([pkg_share, "maps/hospital_section.world.yaml"]),
            ),
            DeclareLaunchArgument(name="update_rate", default_value="100.0"),
            DeclareLaunchArgument(name="step_size", default_value="0.01"),
            DeclareLaunchArgument(name="show_viz", default_value="true"),
            DeclareLaunchArgument(name="viz_pub_rate", default_value="30.0"),
            flatland_server,
            Node(
                package="twist_mux",
                executable="twist_mux",
                name="twist_mux",
                parameters=[
                    PathJoinSubstitution([pkg_share, "config", "cmd_vel_mux.yaml"]),
                    {"use_sim_time": True},
                ],
                remappings=[("cmd_vel_out", "cmd_vel")],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([nav2_share, "launch", "bringup_launch.py"])
                ),
                launch_arguments={
                    "map": PathJoinSubstitution([pkg_share, "maps", "hospital_section.yaml"]),
                    "use_sim_time": "true",
                    "autostart": "true",
                    "use_composition": "true",
                    "use_keepout_zones": "false",
                    "use_speed_zones": "false",
                    "params_file": nav2_params,
                }.items(),
            ),
            Node(
                name="rviz",
                package="rviz2",
                executable="rviz2",
                arguments=["-d", PathJoinSubstitution([pkg_share, "rviz", "robot_navigation.rviz"])],
                parameters=[{"use_sim_time": True}],
                condition=IfCondition(LaunchConfiguration("show_viz")),
            ),
        ]
    )


if __name__ == "__main__":
    generate_launch_description()
