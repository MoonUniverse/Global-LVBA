from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz = LaunchConfiguration("rviz")
    package_share = FindPackageShare("global_lvba")
    config_path = PathJoinSubstitution([package_share, "config", "config.yaml"])
    rviz_config = PathJoinSubstitution([package_share, "rviz_cfg", "lv_ba.rviz"])

    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="true"),
            Node(
                package="global_lvba",
                executable="lidar_visual_ba",
                name="lv_ba",
                output="screen",
                parameters=[config_path],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                condition=IfCondition(rviz),
            ),
        ]
    )