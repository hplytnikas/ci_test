import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
    TimerAction,
    ExecuteProcess,
)
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import UnlessCondition, IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    publish_moving = ExecuteProcess(
        cmd=[
            "ros2",
            "topic",
            "pub",
            "/moving",
            "std_msgs/msg/Bool",
            '"{data: true}"',
        ],
        output="screen",
        shell=True,
    )

    # Delayed publish of res go
    delayed_publish_moving = TimerAction(period=5.0, actions=[publish_moving])

    launch_estimation = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                launch_file_path=f'{get_package_share_directory("tf_publisher")}/launch/tf_publisher.launch.py'
            ),
            condition=UnlessCondition(LaunchConfiguration("estimation_gt")),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                launch_file_path=f'{get_package_share_directory("lap_counter")}/launch/lap_counter.launch.py'
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                launch_file_path=f'{get_package_share_directory("slam_frontend")}/launch/slam_frontend_autocross.launch.py'
            ),
            # condition=UnlessCondition(LaunchConfiguration("estimation_gt")),
            condition=IfCondition(
                PythonExpression(
                    [
                        "'",
                        LaunchConfiguration("discipline_name"),
                        "' == 'autocross' and ",
                        LaunchConfiguration("estimation_gt"),
                        " == False",
                    ]
                )
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                launch_file_path=f'{get_package_share_directory("slam_frontend")}/launch/slam_frontend_acceleration.launch.py'
            ),
            # condition=UnlessCondition(LaunchConfiguration("estimation_gt")),
            condition=IfCondition(
                PythonExpression(
                    [
                        "'",
                        LaunchConfiguration("discipline_name"),
                        "' == 'acceleration' and ",
                        LaunchConfiguration("estimation_gt"),
                        " == False",
                    ]
                )
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                launch_file_path=f'{get_package_share_directory("slam_frontend")}/launch/slam_frontend_skidpad.launch.py'
            ),
            # condition=UnlessCondition(LaunchConfiguration("estimation_gt")),
            condition=IfCondition(
                PythonExpression(
                    [
                        "'",
                        LaunchConfiguration("discipline_name"),
                        "' == 'skidpad' and ",
                        LaunchConfiguration("estimation_gt"),
                        " == False",
                    ]
                )
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                launch_file_path=f'{get_package_share_directory("slam_frontend")}/launch/slam_frontend_trackdrive.launch.py'
            ),
            # condition=UnlessCondition(LaunchConfiguration("estimation_gt")),
            condition=IfCondition(
                PythonExpression(
                    [
                        "'",
                        LaunchConfiguration("discipline_name"),
                        "' == 'trackdrive' and ",
                        LaunchConfiguration("estimation_gt"),
                        " == False",
                    ]
                )
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                launch_file_path=f'{get_package_share_directory("boundary_estimation")}/launch/boundary_estimation.launch.py'
            ),
            launch_arguments={"perception_mode": "sensor_fusion"}.items(),
            condition=UnlessCondition(
                PythonExpression(
                    [
                        "'",
                        LaunchConfiguration("discipline_name"),
                        "' == 'skidpad' or '",
                        LaunchConfiguration("discipline_name"),
                        "' == 'acceleration'",
                    ]
                )
            ),
        ),
        delayed_publish_moving,
        Node(
            package="amzsim_estimation",
            executable="fake_estimation",
            name="faked_estimation",
            output="screen",
            condition=IfCondition(LaunchConfiguration("estimation_gt")),
        ),
        Node(
            package="amzsim_ve",
            executable="sensor_tf",
            name="sensor_tf_faker",
            output="screen",
            condition=IfCondition(LaunchConfiguration("estimation_gt")),
        ),
    ]
    return LaunchDescription(launch_estimation)
