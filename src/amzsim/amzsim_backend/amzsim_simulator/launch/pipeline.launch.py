from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
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

    launch_estimation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=f'{get_package_share_directory("amzsim_estimation")}/launch/amzsim_estimation.launch.py'
        ),
        launch_arguments={
            "discipline_name": LaunchConfiguration("discipline_name"),
            "estimation_gt": LaunchConfiguration("estimation_gt"),
            "grip_estimation": LaunchConfiguration("grip_estimation"),
        }.items(),
    )

    launch_can = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=f'{get_package_share_directory("amzsim_ve")}/launch/amzsim_ve_interface.launch.py'
        ),
        launch_arguments={
            "ve_gt": LaunchConfiguration("ve_gt"),
        }.items(),
    )

    launch_perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=f'{get_package_share_directory("amzsim_perception")}/launch/amzsim_perception.launch.py'
        ),
        launch_arguments={
            "track_path": LaunchConfiguration("track_path"),
            "perception_gt": LaunchConfiguration("perception_gt"),
            "estimation_gt": LaunchConfiguration("estimation_gt"),
            "pipeline_id": LaunchConfiguration("pipeline_id"),
        }.items(),
    )

    # Launch Controller when controller launch files are fixed
    launch_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=f'{get_package_share_directory("amzsim_control")}/launch/amzsim_control.launch.py'
        ),
        launch_arguments={
            "discipline_name": LaunchConfiguration("discipline_name"),
            "control_node_name": LaunchConfiguration("control_node_name"),
        }.items(),
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
            launch_can,
            launch_estimation,
            launch_perception,
            launch_controller,
        ]
    )
