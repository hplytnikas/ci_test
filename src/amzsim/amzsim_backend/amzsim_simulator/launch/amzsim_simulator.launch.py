from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    DeclareLaunchArgument("autosend_res", default_value="true"),
    control_node = DeclareLaunchArgument("control_node_name", default_value="mpc4")
    discipline_name = DeclareLaunchArgument("discipline_name", default_value="skidpad")
    track_path = DeclareLaunchArgument(
        "track_path"
    )  # , default_value=f'{get_package_share_directory("amzsim_tracks")}/tracks_csv/standard/skidpad/Skidpad.csv')
    perception_gt = DeclareLaunchArgument("perception_gt", default_value="false")
    estimation_gt = DeclareLaunchArgument("estimation_gt", default_value="false")
    ve_gt = DeclareLaunchArgument("ve_gt", default_value="false")
    grip_estimation = DeclareLaunchArgument("grip_estimation", default_value="false")
    pipeline_id = DeclareLaunchArgument("pipeline_id")
    tv_ff = DeclareLaunchArgument("tv_ff", default_value="0.6")
    tv_exp = DeclareLaunchArgument("tv_exp", default_value="2.45")
    tv_p = DeclareLaunchArgument("tv_p", default_value="325.0")
    tv_i = DeclareLaunchArgument("tv_i", default_value="0.0")
    tv_d = DeclareLaunchArgument("tv_d", default_value="0.0")
    ax_m = DeclareLaunchArgument("ax_m", default_value="0.6")
    ax_q = DeclareLaunchArgument("ax_q", default_value="2.45")
    ax_p = DeclareLaunchArgument("ax_p", default_value="600.0")
    ax_i = DeclareLaunchArgument("ax_i", default_value="100.0")
    ax_d = DeclareLaunchArgument("ax_d", default_value="0.0")
    pge = DeclareLaunchArgument("pge", default_value="0.0")
    roll = DeclareLaunchArgument("roll", default_value="false")

    vehicle_model = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=f'{get_package_share_directory("amzsim_codegen_model")}/launch/model.launch.py'
        ),
        launch_arguments={
            "tv_ff": LaunchConfiguration("tv_ff"),
            "tv_exp": LaunchConfiguration("tv_exp"),
            "tv_p": LaunchConfiguration("tv_p"),
            "tv_i": LaunchConfiguration("tv_i"),
            "tv_d": LaunchConfiguration("tv_d"),
            "ax_m": LaunchConfiguration("ax_m"),
            "ax_q": LaunchConfiguration("ax_q"),
            "ax_p": LaunchConfiguration("ax_p"),
            "ax_i": LaunchConfiguration("ax_i"),
            "ax_d": LaunchConfiguration("ax_d"),
            "pge": LaunchConfiguration("pge"),
            "roll": LaunchConfiguration("roll"),
        }.items(),
    )

    pipeline_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=f'{get_package_share_directory("amzsim_simulator")}/launch/pipeline.launch.py'
        ),
        launch_arguments={
            "discipline_name": LaunchConfiguration("discipline_name"),
            "track_path": LaunchConfiguration("track_path"),
            "control_node_name": LaunchConfiguration("control_node_name"),
            "ve_gt": LaunchConfiguration("ve_gt"),
            "perception_gt": LaunchConfiguration("perception_gt"),
            "estimation_gt": LaunchConfiguration("estimation_gt"),
            "grip_estimation": LaunchConfiguration("grip_estimation"),
            "pipeline_id": LaunchConfiguration("pipeline_id"),
        }.items(),
    )

    # Publishes the state of the car
    state_publisher = Node(
        package="amzsim_simulator",
        executable="state_publisher",
        name="simulator",
        output="screen",
    )

    return LaunchDescription(
        [
            discipline_name,
            track_path,
            control_node,
            ve_gt,
            perception_gt,
            estimation_gt,
            grip_estimation,
            pipeline_id,
            vehicle_model,
            pipeline_launch,
            state_publisher,
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
        ]
    )
