import os

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_path
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    # Define Launch Arguments
    gui_value_arg = DeclareLaunchArgument("gui_value", default_value="true")
    show_lap_info_arg = DeclareLaunchArgument("show_lap_info", default_value="false")
    show_velocity_arg = DeclareLaunchArgument("show_velocity", default_value="false")
    show_track_length_arg = DeclareLaunchArgument(
        "show_track_length", default_value="false"
    )
    show_collision_detect_arg = DeclareLaunchArgument(
        "show_collision_detect", default_value="false"
    )
    show_fov_cones_arg = DeclareLaunchArgument("show_fov_cones", default_value="false")
    show_ego_frame_arg = DeclareLaunchArgument("show_ego_frame", default_value="false")
    show_veh_frame_arg = DeclareLaunchArgument("show_veh_frame", default_value="false")
    show_online_map_arg = DeclareLaunchArgument(
        "show_online_map", default_value="false"
    )
    show_bounded_path_arg = DeclareLaunchArgument(
        "show_bounded_path", default_value="false"
    )
    show_delaunay_arg = DeclareLaunchArgument("show_delaunay", default_value="false")
    show_control_bounds_arg = DeclareLaunchArgument(
        "show_control_bounds", default_value="false"
    )

    # Define Rviz Configuration Argument
    rvizconfig_arg = DeclareLaunchArgument(
        "rvizconfig",
        default_value=os.path.join(
            get_package_share_path("amzsim_rviz_handler"), "rviz/config.rviz"
        ),
    )

    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration("gui_value")),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
        output="screen",
    )

    edit_rviz_node = Node(
        package="amzsim_rviz_handler",
        executable="edit_rviz.py",
        name="edit_rviz_node",
        output="screen",
        parameters=[
            {"show_lap_info": LaunchConfiguration("show_lap_info")},
            {"show_velocity": LaunchConfiguration("show_velocity")},
            {"show_track_length": LaunchConfiguration("show_track_length")},
            {"show_collision_detect": LaunchConfiguration("show_collision_detect")},
            {"show_fov_cones": LaunchConfiguration("show_fov_cones")},
            {"show_ego_frame": LaunchConfiguration("show_ego_frame")},
            {"show_veh_frame": LaunchConfiguration("show_veh_frame")},
            {"show_online_map": LaunchConfiguration("show_online_map")},
            {"show_bounded_path": LaunchConfiguration("show_bounded_path")},
            {"show_delaunay": LaunchConfiguration("show_delaunay")},
            {"show_control_bounds": LaunchConfiguration("show_control_bounds")},
            {"rvizconfig": LaunchConfiguration("rvizconfig")},
        ],
    )

    # Launch Description
    return LaunchDescription(
        [
            gui_value_arg,
            show_lap_info_arg,
            show_velocity_arg,
            show_track_length_arg,
            show_collision_detect_arg,
            show_fov_cones_arg,
            show_ego_frame_arg,
            show_veh_frame_arg,
            show_online_map_arg,
            show_bounded_path_arg,
            show_delaunay_arg,
            show_control_bounds_arg,
            rvizconfig_arg,
            rviz_node,
            edit_rviz_node,
            # rviz_overlay_amzsim_node
        ]
    )
