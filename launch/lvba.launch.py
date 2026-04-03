from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_workspace_file(subdir, filename):
    launch_file = Path(__file__).resolve()
    installed_file = launch_file.parent.parent / subdir / filename

    for parent in launch_file.parents:
        candidate = parent / subdir / filename
        if (parent / "CMakeLists.txt").exists() and candidate.exists():
            return str(candidate)

    return str(installed_file)


def generate_launch_description():
    rviz = LaunchConfiguration("rviz")
    config_file = LaunchConfiguration("config_file")
    rviz_config_file = LaunchConfiguration("rviz_config")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="Launch RViz2 alongside the LVBA node.",
            ),
            DeclareLaunchArgument(
                "config_file",
                default_value=_resolve_workspace_file("config", "config.yaml"),
                description="Path to the YAML parameter file.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=_resolve_workspace_file(
                    "rviz_cfg", "lv_ba.rviz"
                ),
                description="Path to the RViz2 config file.",
            ),
            Node(
                package="global_lvba",
                executable="lidar_visual_ba",
                name="lv_ba",
                output="screen",
                parameters=[config_file, {"runtime_config_path": config_file}],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config_file],
                condition=IfCondition(rviz),
            ),
        ]
    )
