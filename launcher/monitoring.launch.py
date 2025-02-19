from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    OpaqueFunction,
    DeclareLaunchArgument,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    monitor_target = DeclareLaunchArgument(
        "target",
        default_value="both",
        description=""" what should be monitored ?
            pipelines - monitor the pipelines
            resources - monitor the resources
            both - pipelines + resources
        """,
    )

    monitoring_pipelines_launch = os.path.join(
        get_package_share_directory("monitoring_pipelines"),
        "launch",
        "monitoring_pipelines.launch.py",
    )

    monitoring_resources_launch = os.path.join(
        get_package_share_directory("monitoring_resources"),
        "launch",
        "monitoring_resources.launch.py",
    )

    def assign_chosen_descriptors(context):
        monitor_target = context.launch_configurations["target"]
        to_launch = []

        if monitor_target == "pipelines":
            to_launch += [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(monitoring_pipelines_launch)
                )
            ]

        if monitor_target == "resources":
            to_launch += [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(monitoring_resources_launch)
                )
            ]

        if monitor_target == "both":
            to_launch += [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(monitoring_resources_launch)
                )
            ]
            to_launch += [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(monitoring_pipelines_launch)
                )
            ]

        return to_launch

    return LaunchDescription(
        [
            monitor_target,
            OpaqueFunction(function=assign_chosen_descriptors),
        ]
    )
