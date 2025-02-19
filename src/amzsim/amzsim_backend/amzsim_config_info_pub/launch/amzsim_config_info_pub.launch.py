from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    discipline_name = DeclareLaunchArgument("discipline_name", default_value="skidpad")
    track_path = DeclareLaunchArgument("track_path")
    control_node_name = DeclareLaunchArgument("control_node_name", default_value="mpc2")
    lapopt_launched = DeclareLaunchArgument("lapopt_launched", default_value="false")
    record_rosbag = DeclareLaunchArgument("record_rosbag", default_value="false")
    play_rosbag = DeclareLaunchArgument("play_rosbag", default_value="false")
    rosbag_path = DeclareLaunchArgument("rosbag_path", default_value="rosbag_path")
    ve_gt = DeclareLaunchArgument("ve_gt", default_value="false")
    perception_gt = DeclareLaunchArgument("perception_gt", default_value="false")
    estimation_gt = DeclareLaunchArgument("estimation_gt", default_value="false")
    grip_estimation = DeclareLaunchArgument("grip_estimation", default_value="false")

    config_publisher = Node(
        name="amzsim_config",
        package="amzsim_config_info_pub",
        executable="config",
        output="screen",
        parameters=[
            {"discipline_name": LaunchConfiguration("discipline_name")},
            {"track_path": LaunchConfiguration("track_path")},
            {"control_node_name": LaunchConfiguration("control_node_name")},
            {"lapopt_launched": LaunchConfiguration("lapopt_launched")},
            {"record_rosbag": LaunchConfiguration("record_rosbag")},
            {"play_rosbag": LaunchConfiguration("play_rosbag")},
            {"rosbag_path": LaunchConfiguration("rosbag_path")},
            {"ve_gt": LaunchConfiguration("ve_gt")},
            {"perception_gt": LaunchConfiguration("perception_gt")},
            {"estimation_gt": LaunchConfiguration("estimation_gt")},
            {"grip_estimation": LaunchConfiguration("grip_estimation")},
        ],
    )

    return LaunchDescription(
        [
            discipline_name,
            track_path,
            control_node_name,
            ve_gt,
            perception_gt,
            estimation_gt,
            grip_estimation,
            record_rosbag,
            play_rosbag,
            rosbag_path,
            lapopt_launched,
            config_publisher,
        ]
    )
