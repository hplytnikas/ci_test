from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import UnlessCondition, IfCondition
from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    DeclareLaunchArgument,
    SetLaunchConfiguration,
    TimerAction,
    ExecuteProcess,
)


def generate_launch_description():
    # Make sure simulation time is used
    sim_time = DeclareLaunchArgument("use_sim_time_arg", default_value="False")
    use_sim_time = SetLaunchConfiguration(
        "use_sim_time", LaunchConfiguration("use_sim_time_arg")
    )

    # launch arguments for pipeline modularity
    track_path = DeclareLaunchArgument(
        "track_path_arg", default_value="/tracks_csv/standard/skidpad/Skidpad.csv"
    )
    discipline_name = DeclareLaunchArgument("discipline_name_arg")
    control_node_name = DeclareLaunchArgument(
        "control_node_name_arg", default_value="mpc4"
    )
    ve_gt = DeclareLaunchArgument("ve_gt_arg", default_value="False")
    perception_gt = DeclareLaunchArgument("perception_gt_arg", default_value="False")
    estimation_gt = DeclareLaunchArgument("estimation_gt_arg", default_value="False")
    grip_estimation = DeclareLaunchArgument(
        "grip_estimation_arg", default_value="False"
    )
    pipeline_id = DeclareLaunchArgument("pipeline_id_arg")

    # TODO: add rosbag compatability with simultor
    record_rosbag = DeclareLaunchArgument("record_rosbag_arg", default_value="false")
    play_rosbag = DeclareLaunchArgument("play_rosbag_arg", default_value="false")
    rosbag_path = DeclareLaunchArgument("rosbag_path_arg", default_value="rosbag_path")

    # launch arguments for visualization customization
    rviz_on = DeclareLaunchArgument("rviz_on_arg", default_value="true")
    show_lap_info = DeclareLaunchArgument("show_lap_info_arg", default_value="false")
    show_velocity = DeclareLaunchArgument("show_velocity_arg", default_value="false")
    show_track_length = DeclareLaunchArgument(
        "show_track_length_arg", default_value="false"
    )
    show_collision_detection = DeclareLaunchArgument(
        "show_collision_detect_arg", default_value="false"
    )
    show_fov_cones = DeclareLaunchArgument("show_fov_cones_arg", default_value="false")
    show_ego_frame = DeclareLaunchArgument("show_ego_frame_arg", default_value="false")
    show_veh_frame = DeclareLaunchArgument("show_veh_frame_arg", default_value="false")
    show_online_map = DeclareLaunchArgument(
        "show_online_map_arg", default_value="false"
    )
    show_bounded_path = DeclareLaunchArgument(
        "show_bounded_path_arg", default_value="false"
    )
    show_delaunay = DeclareLaunchArgument("show_delaunay_arg", default_value="false")
    show_control_bounds = DeclareLaunchArgument(
        "show_control_bounds_arg", default_value="false"
    )
    lapopt_launched = DeclareLaunchArgument(
        "lapopt_launched_arg", default_value="false"
    )

    # launch arguments for model
    tv_ff = DeclareLaunchArgument("tv_ff_arg", default_value="0.6")
    tv_exp = DeclareLaunchArgument("tv_exp_arg", default_value="2.45")
    tv_p = DeclareLaunchArgument("tv_p_arg", default_value="325")
    tv_i = DeclareLaunchArgument("tv_i_arg", default_value="0")
    tv_d = DeclareLaunchArgument("tv_d_arg", default_value="0")
    ax_m = DeclareLaunchArgument("ax_m_arg", default_value="0.6")
    ax_q = DeclareLaunchArgument("ax_q_arg", default_value="2.45")
    ax_p = DeclareLaunchArgument("ax_p_arg", default_value="600")
    ax_i = DeclareLaunchArgument("ax_i_arg", default_value="100")
    ax_d = DeclareLaunchArgument("ax_d_arg", default_value="0")
    pge = DeclareLaunchArgument("pge_arg", default_value="0")
    roll = DeclareLaunchArgument("roll_arg", default_value="false")
    # tv_ff = LaunchConfiguration("tv_ff_arg", default_value="0.6")
    # tv_exp = LaunchConfiguration("tv_exp_arg", default_value="2.45")
    # tv_p = LaunchConfiguration("tv_p_arg", default_value="325")
    # tv_i = LaunchConfiguration("tv_i_arg", default_value="0")
    # tv_d = LaunchConfiguration("tv_d_arg", default_value="0")
    # ax_m = LaunchConfiguration("ax_m_arg", default_value="0.6")
    # ax_q = LaunchConfiguration("ax_q_arg", default_value="2.45")
    # ax_p = LaunchConfiguration("ax_p_arg", default_value="600")
    # ax_i = LaunchConfiguration("ax_i_arg", default_value="100")
    # ax_d = LaunchConfiguration("ax_d_arg", default_value="0")
    # pge = LaunchConfiguration("pge_arg", default_value="0")
    # roll = LaunchConfiguration("roll_arg", default_value="low")

    # launch simulator if play_rosbag_arg is false
    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=f'{get_package_share_directory("amzsim_simulator")}/launch/amzsim_simulator.launch.py'
        ),
        condition=UnlessCondition(LaunchConfiguration("play_rosbag_arg")),
        launch_arguments={
            "discipline_name": LaunchConfiguration("discipline_name_arg"),
            "track_path": LaunchConfiguration("track_path_arg"),
            "control_node_name": LaunchConfiguration("control_node_name_arg"),
            "ve_gt": LaunchConfiguration("ve_gt_arg"),
            "perception_gt": LaunchConfiguration("perception_gt_arg"),
            "estimation_gt": LaunchConfiguration("estimation_gt_arg"),
            "grip_estimation": LaunchConfiguration("grip_estimation_arg"),
            "pipeline_id": LaunchConfiguration("pipeline_id_arg"),
            "tv_ff": LaunchConfiguration("tv_ff_arg"),
            "tv_exp": LaunchConfiguration("tv_exp_arg"),
            "tv_p": LaunchConfiguration("tv_p_arg"),
            "tv_i": LaunchConfiguration("tv_i_arg"),
            "tv_d": LaunchConfiguration("tv_d_arg"),
            "ax_m": LaunchConfiguration("ax_m_arg"),
            "ax_q": LaunchConfiguration("ax_q_arg"),
            "ax_p": LaunchConfiguration("ax_p_arg"),
            "ax_i": LaunchConfiguration("ax_i_arg"),
            "ax_d": LaunchConfiguration("ax_d_arg"),
            "pge": LaunchConfiguration("pge_arg"),
            "roll": LaunchConfiguration("roll_arg"),
        }.items(),
    )

    # launch rviz if rviz_on_arg is true
    rviz_handler = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=f'{get_package_share_directory("amzsim_rviz_handler")}/launch/amzsim_rviz_handler.launch.py'
        ),
        launch_arguments={
            "gui_value": LaunchConfiguration("rviz_on_arg"),
            "show_lap_info": LaunchConfiguration("show_lap_info_arg"),
            "show_velocity": LaunchConfiguration("show_velocity_arg"),
            "show_track_length": LaunchConfiguration("show_track_length_arg"),
            "show_collision_detect": LaunchConfiguration("show_collision_detect_arg"),
            "show_fov_cones": LaunchConfiguration("show_fov_cones_arg"),
            "show_ego_frame": LaunchConfiguration("show_ego_frame_arg"),
            "show_veh_frame": LaunchConfiguration("show_veh_frame_arg"),
            "show_online_map": LaunchConfiguration("show_online_map_arg"),
            "show_bounded_path": LaunchConfiguration("show_bounded_path_arg"),
            "show_delaunay": LaunchConfiguration("show_delaunay_arg"),
            "show_show_control_bounds": LaunchConfiguration("show_control_bounds_arg"),
        }.items(),
    )

    # launch car visualization
    car_visualization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=f'{get_package_share_directory("amzsim_car_visualization")}/launch/amzsim_car_visualization.launch.py'
        )
    )

    # launch track visualization
    track_visualization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=f'{get_package_share_directory("amzsim_tracks_visualization")}/launch/amzsim_tracks_visualization.launch.py'
        ),
    )

    # TODO: implement rosbag recording
    # record_bag =  TimerAction(
    #         period=2.0,
    #         actions=[
    #             ExecuteProcess(
    #                 cmd=[
    #                     "ros2",
    #                     "bag",
    #                     "record",
    #                     "-a",
    #                     "-s",
    #                     "mcap",
    #                 ],
    #                 output="screen",
    #             ),
    #         ],
    #     ),

    launch_config_info = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=f'{get_package_share_directory("amzsim_config_info_pub")}/launch/amzsim_config_info_pub.launch.py'
        ),
        launch_arguments={
            "discipline_name": LaunchConfiguration("discipline_name_arg"),
            "track_path": LaunchConfiguration("track_path_arg"),
            "control_node_name": LaunchConfiguration("control_node_name_arg"),
            "lapopt_launched": LaunchConfiguration("lapopt_launched_arg"),
            "record_rosbag": LaunchConfiguration("record_rosbag_arg"),
            "play_rosbag": LaunchConfiguration("play_rosbag_arg"),
            "rosbag_path": LaunchConfiguration("rosbag_path_arg"),
            "ve_gt": LaunchConfiguration("ve_gt_arg"),
            "perception_gt": LaunchConfiguration("perception_gt_arg"),
            "estimation_gt": LaunchConfiguration("estimation_gt_arg"),
            "grip_estimation": LaunchConfiguration("grip_estimation_arg"),
        }.items(),
    )

    return LaunchDescription(
        [
            sim_time,
            use_sim_time,
            track_path,
            discipline_name,
            control_node_name,
            lapopt_launched,
            ve_gt,
            perception_gt,
            estimation_gt,
            grip_estimation,
            pipeline_id,
            record_rosbag,
            play_rosbag,
            rosbag_path,
            rviz_on,
            show_lap_info,
            show_velocity,
            show_track_length,
            show_collision_detection,
            show_fov_cones,
            show_ego_frame,
            show_veh_frame,
            show_online_map,
            show_bounded_path,
            show_delaunay,
            show_control_bounds,
            simulator,
            rviz_handler,
            car_visualization,
            track_visualization,
            launch_config_info,
            tv_ff,
            tv_exp,
            tv_p,
            tv_i,
            tv_d,
            ax_m,
            ax_q,
            ax_p,
            ax_i,
            ax_d,
            pge,
            roll,
            # record_bag,
            TimerAction(
                period=2.0,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "ros2",
                            "bag",
                            "record",
                            "-a",
                            "-s",
                            "mcap",
                        ],
                        output="screen",
                        condition=IfCondition(LaunchConfiguration("record_rosbag_arg")),
                    ),
                ],
            ),
        ]
    )
