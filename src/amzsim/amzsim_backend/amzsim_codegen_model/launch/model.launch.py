from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    config_path = f'{get_package_share_directory("amzsim_codegen_model")}/config/codegen_model_config.yaml'

    vehicle_model = Node(
        package="amzsim_codegen_model",
        executable="model",
        name="amzsim_model",
        output="screen",
        parameters=[
            {"tv_ff": LaunchConfiguration("tv_ff")},
            {"tv_exp": LaunchConfiguration("tv_exp")},
            {"tv_p": LaunchConfiguration("tv_p")},
            {"tv_i": LaunchConfiguration("tv_i")},
            {"tv_d": LaunchConfiguration("tv_d")},
            {"ax_m": LaunchConfiguration("ax_m")},
            {"ax_q": LaunchConfiguration("ax_q")},
            {"ax_p": LaunchConfiguration("ax_p")},
            {"ax_i": LaunchConfiguration("ax_i")},
            {"ax_d": LaunchConfiguration("ax_d")},
            {"pge": LaunchConfiguration("pge")},
            {"roll": LaunchConfiguration("roll")},
            {config_path},
        ],
    )

    return LaunchDescription(
        [
            vehicle_model,
        ]
    )
