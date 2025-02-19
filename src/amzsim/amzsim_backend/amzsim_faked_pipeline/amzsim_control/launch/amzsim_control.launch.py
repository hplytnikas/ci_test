import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import UnlessCondition
from launch.conditions import IfCondition
from launch.actions import ExecuteProcess, TimerAction


def generate_launch_description():
    discipline_name = DeclareLaunchArgument("discipline_name", default_value="skidpad")
    control_node_name = DeclareLaunchArgument("control_node_name", default_value="mpc2")

    # Controller only starts when res go is pressed
    # Publishing this emulates the res go being pressed
    # Executes
    # ros2 topic pub /vcu_msgs/res_state vcu_msgs/msg/ResState "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""} emergency: false, on_off_switch: false, push_button: true, communication_interrupted: false}"
    # It is ///" because it is first escaped by python and then by the shell
    publish_res_go = ExecuteProcess(
        cmd=[
            "ros2",
            "topic",
            "pub",
            "/vcu_msgs/res_state",
            "vcu_msgs/msg/ResState",
            '"{header: {stamp: {sec: 0, nanosec: 0}, frame_id: \\"\\"}, emergency: false, on_off_switch: false, push_button: true, communication_interrupted: false}"',
        ],
        output="screen",
        shell=True,
    )

    # Delayed publish of res go
    delayed_publish_res_go = TimerAction(period=5.0, actions=[publish_res_go])

    # Fix control launch when control launch files are added
    publish_control = [
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         launch_file_path=f'{get_package_share_directory("boundary_to_reference")}/launch/boundary_to_reference.launch.py'
        #     ),
        #     condition=IfCondition(
        #         PythonExpression(
        #             [
        #                 "'",
        #                 LaunchConfiguration("control_node_name"),
        #                 "' == 'purepursuit' and '",
        #                 LaunchConfiguration("discipline_name"),
        #                 "' == 'autocross'",
        #             ]
        #         )
        #     ),
        # ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                launch_file_path=f'{get_package_share_directory("pure_pursuit_controller")}/launch/pure_pursuit_controller_autocross.launch.py'
            ),
            condition=IfCondition(
                PythonExpression(
                    [
                        "'",
                        LaunchConfiguration("control_node_name"),
                        "' == 'purepursuit' and ( '",
                        LaunchConfiguration("discipline_name"),
                        "' == 'autocross' or '",
                        LaunchConfiguration("discipline_name"),
                        "' == 'trackdrive')",
                    ]
                )
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                launch_file_path=f'{get_package_share_directory("pure_pursuit_controller")}/launch/pure_pursuit_controller_skidpad.launch.py'
            ),
            condition=IfCondition(
                PythonExpression(
                    [
                        "'",
                        LaunchConfiguration("control_node_name"),
                        "' == 'purepursuit' and '",
                        LaunchConfiguration("discipline_name"),
                        "' == 'skidpad'",
                    ]
                )
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                launch_file_path=f'{get_package_share_directory("pure_pursuit_controller")}/launch/pure_pursuit_controller_acceleration.launch.py'
            ),
            condition=IfCondition(
                PythonExpression(
                    [
                        "'",
                        LaunchConfiguration("control_node_name"),
                        "' == 'purepursuit' and '",
                        LaunchConfiguration("discipline_name"),
                        "' == 'acceleration'",
                    ]
                )
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                launch_file_path=f'{get_package_share_directory("autocross_handler")}/launch/autocross_handler.launch.py'
            ),
            condition=IfCondition(
                PythonExpression(
                    [
                        "'",
                        LaunchConfiguration("discipline_name"),
                        "' == 'autocross'",
                    ]
                )
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                launch_file_path=f'{get_package_share_directory("skidpad_handler")}/launch/skidpad_handler.launch.py'
            ),
            condition=IfCondition(
                # PythonExpression(
                #     [
                #         "'",
                #         LaunchConfiguration("discipline_name"),
                #         "' == 'skidpad' and '",
                #         LaunchConfiguration("control_node_name"),
                #         "' != 'none'",
                #     ]
                # )
                PythonExpression(
                    [
                        "'",
                        LaunchConfiguration("discipline_name"),
                        "' == 'skidpad'",
                    ]
                )
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                launch_file_path=f'{get_package_share_directory("acceleration_handler")}/launch/acceleration_handler.launch.py'
            ),
            condition=IfCondition(
                PythonExpression(
                    [
                        "'",
                        LaunchConfiguration("discipline_name"),
                        "' == 'acceleration' and '",
                        LaunchConfiguration("control_node_name"),
                        "' != 'none'",
                    ]
                )
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                launch_file_path=f'{get_package_share_directory("acceleration_path_planner_basic")}/launch/acceleration_path_planner.launch.py'
            ),
            condition=IfCondition(
                PythonExpression(
                    [
                        "'",
                        LaunchConfiguration("discipline_name"),
                        "' == 'acceleration' and '",
                        LaunchConfiguration("control_node_name"),
                        "' != 'none'",
                    ]
                )
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                launch_file_path=f'{get_package_share_directory("trackdrive_handler")}/launch/trackdrive_handler.launch.py'
            ),
            condition=IfCondition(
                PythonExpression(
                    [
                        "'",
                        LaunchConfiguration("discipline_name"),
                        "' == 'trackdrive'",
                    ]
                )
            ),
        ),
        delayed_publish_res_go,
        # MPC CONTROLLER Autocross and Trackdrive
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                launch_file_path=f'{get_package_share_directory("mpc_controller")}/launch/mpc_controller.launch.py'
            ),
            launch_arguments={"mpc_mode": "safe"}.items(),
            condition=IfCondition(
                PythonExpression(
                    [
                        "'",
                        LaunchConfiguration("control_node_name"),
                        "' == 'mpc' and '",
                        LaunchConfiguration("discipline_name"),
                        "' == 'autocross' or '",
                        LaunchConfiguration("discipline_name"),
                        "' == 'trackdrive'",
                    ]
                )
            ),
        ),
        # MPC CONTROLLER Skidpad
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                launch_file_path=f'{get_package_share_directory("mpc_controller")}/launch/mpc_controller.launch.py'
            ),
            launch_arguments={"mpc_mode": "skidpad"}.items(),
            condition=IfCondition(
                PythonExpression(
                    [
                        "'",
                        LaunchConfiguration("control_node_name"),
                        "' == 'mpc' and '",
                        LaunchConfiguration("discipline_name"),
                        "' == 'skidpad'",
                    ]
                )
            ),
        ),
        # LOCAL PLANNER MPC
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                launch_file_path=f'{get_package_share_directory("local_planner")}/launch/local_planner.launch.py'
            ),
            launch_arguments={"controller_mode": "mpc_safe"}.items(),
            condition=IfCondition(
                PythonExpression(
                    [
                        "('",
                        LaunchConfiguration("discipline_name"),
                        "' == 'autocross' or '",
                        LaunchConfiguration("discipline_name"),
                        "' == 'trackdrive') and '",
                        LaunchConfiguration("control_node_name"),
                        "' == 'mpc'",
                    ]
                )
            ),
        ),
        # LOCAL PLANNER PURE PURSUIT
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                launch_file_path=f'{get_package_share_directory("local_planner")}/launch/local_planner.launch.py'
            ),
            launch_arguments={"controller_mode": "pure_pursuit"}.items(),
            condition=IfCondition(
                PythonExpression(
                    [
                        "('",
                        LaunchConfiguration("discipline_name"),
                        "' == 'autocross' or '",
                        LaunchConfiguration("discipline_name"),
                        "' == 'trackdrive') and '",
                        LaunchConfiguration("control_node_name"),
                        "' == 'purepursuit'",
                    ]
                )
            ),
        ),
        # =====================
        # GLOBAL PLANNER MPC
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                launch_file_path=f'{get_package_share_directory("global_planner")}/launch/global_planner.launch.py'
            ),
            launch_arguments={"controller_mode": "mpc_safe"}.items(),
            condition=IfCondition(
                PythonExpression(
                    [
                        "('",
                        LaunchConfiguration("discipline_name"),
                        "' == 'trackdrive') and '",
                        LaunchConfiguration("control_node_name"),
                        "' == 'mpc'",
                    ]
                )
            ),
        ),
        # GLOBAL PLANNER PURE PURSUIT
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                launch_file_path=f'{get_package_share_directory("global_planner")}/launch/global_planner.launch.py'
            ),
            launch_arguments={"controller_mode": "pure_pursuit"}.items(),
            condition=IfCondition(
                PythonExpression(
                    [
                        "('",
                        LaunchConfiguration("discipline_name"),
                        "' == 'trackdrive') and '",
                        LaunchConfiguration("control_node_name"),
                        "' == 'purepursuit'",
                    ]
                )
            ),
        ),
    ]

    return LaunchDescription(publish_control)
