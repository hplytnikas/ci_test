from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition


def generate_launch_description():
    ve_gt = DeclareLaunchArgument("ve_gt", default_value="false")
    launch_file_dir = DeclareLaunchArgument(
        "launch_file_dir", default_value=ThisLaunchFileDir()
    )
    config_path = (
        f'{get_package_share_directory("amzsim_ve")}/config/amzsim_ve_config.yaml'
    )

    launch_fake_can = GroupAction(
        actions=[
            Node(
                package="amzsim_ve",
                executable="steering",
                name="steering_faker",
                output="screen",
            ),
            Node(
                package="amzsim_ve",
                executable="velocity_est",
                name="ve_faker",
                output="screen",
                parameters=[
                    {"ve_gt": LaunchConfiguration("ve_gt")},
                    {"config": config_path},
                ],
            ),
        ]
    )
    # sensor tf faker has been launched in estimation based on estimation_gt setting

    return LaunchDescription(
        [
            ve_gt,
            launch_file_dir,
            launch_fake_can,
        ]
    )
