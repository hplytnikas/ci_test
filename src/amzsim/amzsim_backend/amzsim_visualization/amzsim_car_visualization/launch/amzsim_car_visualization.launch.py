import os
import xacro
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare Launch Arguments
    robot_name_arg = DeclareLaunchArgument("robot_name", default_value="castor_2023")
    car_dimensions_arg = DeclareLaunchArgument(
        "car_dimensions",
        default_value=os.path.join(
            get_package_share_path("amzsim_car_visualization"),
            "cars/castor_2023/config/distances.yaml",
        ),
    )

    # Use OpaqueFunction to defer the execution and access the launch arguments
    generate_robot_state_publisher_node = OpaqueFunction(
        function=lambda context: launch_robot_state_publisher(
            context.launch_configurations["robot_name"],
            context.launch_configurations["car_dimensions"],
        )
    )

    return LaunchDescription(
        [robot_name_arg, car_dimensions_arg, generate_robot_state_publisher_node]
    )


def launch_robot_state_publisher(robot_name, car_dimensions):
    # Generate robot description
    robot_description = generate_robot_description(robot_name, car_dimensions)

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"publish_frequency": 100.0, "robot_description": robot_description}
        ],
    )

    return [robot_state_publisher_node]


def generate_robot_description(robot_name, car_dimensions):
    car_xacro_path = os.path.join(
        get_package_share_path("amzsim_car_visualization"), "urdf/vehicle/car.xacro"
    )

    # Processing the xacro file with provided arguments
    doc = xacro.process_file(
        car_xacro_path,
        mappings={"robot_name": robot_name, "car_dimensions": car_dimensions},
    )
    robot_description = doc.toprettyxml(indent="  ")

    return robot_description
