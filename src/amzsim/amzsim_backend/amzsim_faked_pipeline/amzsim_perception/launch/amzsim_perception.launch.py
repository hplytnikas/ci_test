from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition


def generate_launch_description():
    config_path = f'{get_package_share_directory("amzsim_perception")}/config/amzsim_perception_config.yaml'

    track_path = DeclareLaunchArgument(
        "track_path",
        default_value=f'{get_package_share_directory("amzsim_tracks")}/tracks_csv/standard/skidpad/Skidpad.csv',
    )
    perception_gt = DeclareLaunchArgument("perception_gt", default_value="false")
    estimation_gt = DeclareLaunchArgument("estimation_gt", default_value="false")
    pipeline_id = DeclareLaunchArgument("pipeline_id")

    amzsim_perception = Node(
        package="amzsim_perception",
        executable="fake_perception",
        namespace="perception_faked",
        name="faked_perception",
        # condition=UnlessCondition(LaunchConfiguration("estimation_gt")),
        parameters=[
            {"track_path": LaunchConfiguration("track_path")},
            {"perception_gt": LaunchConfiguration("perception_gt")},
            {"estimation_gt": LaunchConfiguration("estimation_gt")},
            {"pipeline_id": LaunchConfiguration("pipeline_id")},
            {config_path},
        ],
    )

    return LaunchDescription(
        [
            track_path,
            perception_gt,
            estimation_gt,
            pipeline_id,
            amzsim_perception,
        ]
    )
