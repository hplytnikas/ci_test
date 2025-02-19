import os
import sys

from launch.actions import TimerAction

# Required to add this folder in the PYTHONPATH, can also be done once
dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(dir_path)
from nodes_launcher_structs import LidarMode, PerceptionMode, ControllerMode, CarMode
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    OpaqueFunction,
    DeclareLaunchArgument,
    ExecuteProcess,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Declare a launch argument for specifying camera or lidar
    perception_mode_arg = DeclareLaunchArgument(
        "perception_mode",
        default_value="sensor_fusion",
        description="Mode to launch: camera_only, lidar_only or sensor_fusion",
    )

    lidar_mode_arg = DeclareLaunchArgument(
        "lidar_mode",
        default_value="hesai",
        description="Type of lidar to launch: ouster, hesai",
    )

    controller_mode_arg = DeclareLaunchArgument(
        "controller_mode",
        default_value="pure_pursuit",
        description="Mode to launch: mpc, pure_pursuit",
    )

    car_mode_arg = DeclareLaunchArgument(
        "car_mode",
        default_value="dufour",
        description="Mode to launch: castor, dufour",
    )

    # Share directory of the packages
    boundary_estimation_pkg_share_dir = get_package_share_directory(
        "boundary_estimation"
    )
    slam_pkg_share_dir = get_package_share_directory("slam_frontend")
    camera_launcher_pkg_share_dir = get_package_share_directory("camera_launcher")
    static_tf_publisher_pkg_share_dir = get_package_share_directory(
        "static_tf_publisher"
    )
    yolo_camera_detector_pkg_share_dir = get_package_share_directory(
        "yolo_camera_detector"
    )
    lidar_cone_detector_pkg_share_dir = get_package_share_directory(
        "lidar_cone_detector"
    )
    depth_estimation_pkg_share_dir = get_package_share_directory("depth_estimation")
    sensor_fusion_pkg_share_dir = get_package_share_directory("sensor_fusion")
    cone_fusion_pkg_share_dir = get_package_share_directory("cone_fusion")
    tf_publisher_pkg_share_dir = get_package_share_directory("tf_publisher")
    lap_counter_pkg_share_dir = get_package_share_directory("lap_counter")
    # vcu_comm_interface_pkg_share_dir = get_package_share_directory("vcu_comm_interface")
    pure_pursuit_pkg_share_dir = get_package_share_directory("pure_pursuit_controller")
    mpc_pkg_share_dir = get_package_share_directory("mpc_controller")
    hesai_pkg_share_dir = get_package_share_directory("hesai_ros_driver")

    # Get the path to the included launch file
    hesai_driver_launch = os.path.join(hesai_pkg_share_dir, "launch", "start.py")
    # Boundary Estimation Node launch file
    boundary_estimation_launch = os.path.join(
        boundary_estimation_pkg_share_dir, "launch", "boundary_estimation.launch.py"
    )
    # SLAM Node launch file
    slam_launch = os.path.join(
        slam_pkg_share_dir, "launch", "slam_frontend_default.launch.py"
    )
    # Static Tf Publisher Ouster lidar launch file
    static_tf_publisher_ouster_launch = os.path.join(
        static_tf_publisher_pkg_share_dir,
        "launch",
        "static_tf_publisher_castor_ouster.launch.py",
    )
    # Static Tf Publisher Hesai lidar launch file
    # Static Tf Publisher Hesai castor lidar launch file
    static_tf_publisher_hesai_castor_launch = os.path.join(
        static_tf_publisher_pkg_share_dir,
        "launch",
        "static_tf_publisher_castor_hesai.launch.py",
    )
    # Static Tf Publisher Hesai Dufour lidar launch file
    static_tf_publisher_hesai_dufour_launch = os.path.join(
        static_tf_publisher_pkg_share_dir,
        "launch",
        "static_tf_publisher_dufour_hesai.launch.py",
    )
    # Camera Launcher launch file
    camera_launcher_launch = os.path.join(
        camera_launcher_pkg_share_dir, "launch", "camera_launcher.launch.py"
    )
    # Yolo Camera Detector launch file
    yolo_camera_detector_launch = os.path.join(
        yolo_camera_detector_pkg_share_dir, "launch", "yolo_camera_detector.launch.py"
    )

    # Lidar Cone Detector launch file
    lidar_cone_detector_launch = os.path.join(
        lidar_cone_detector_pkg_share_dir,
        "launch",
        "lidar_cone_detector.launch.py",
    )

    # Depth Estimation launch file
    depth_estimation_launch = os.path.join(
        depth_estimation_pkg_share_dir, "launch", "depth_estimation.launch.py"
    )

    # Sensor Fusion launch file
    sensor_fusion_launch = os.path.join(
        sensor_fusion_pkg_share_dir, "launch", "sensor_fusion.launch.py"
    )

    # Cone Fusion launch file
    cone_fusion_launch = os.path.join(
        cone_fusion_pkg_share_dir, "launch", "cone_fusion.launch.py"
    )

    # Tf Publisher launch file
    tf_publisher_launch = os.path.join(
        tf_publisher_pkg_share_dir, "launch", "tf_publisher.launch.py"
    )

    # Lap Counter launch file
    lap_counter_launch = os.path.join(
        lap_counter_pkg_share_dir, "launch", "lap_counter.launch.py"
    )
    # Vcu Comm Interface launch file
    # vcu_comm_interface_launch = os.path.join(
    # vcu_comm_interface_pkg_share_dir, "launch", "vcu_comm_interface.launch.py"
    # )

    # Pure Pursuit launch file
    pure_pursuit_launch = os.path.join(
        pure_pursuit_pkg_share_dir,
        "launch",
        "pure_pursuit_controller_autocross.launch.py",
    )

    # MPC launch file
    mpc_launch = os.path.join(mpc_pkg_share_dir, "launch", "mpc_controller.launch.py")

    def generate_common_launch_description(perception_mode):
        # Launch description
        common_launch_description = [
            # IncludeLaunchDescription(
            # PythonLaunchDescriptionSource(vcu_comm_interface_launch)
            # ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(tf_publisher_launch)
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(boundary_estimation_launch),
                launch_arguments={"perception_mode": perception_mode}.items(),
            ),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(slam_launch)),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(lap_counter_launch)),
        ]
        return common_launch_description

    def launch_camera():
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(camera_launcher_launch)
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(yolo_camera_detector_launch)
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(depth_estimation_launch)
            ),
        ]

    def get_static_tf_publisher_launch(lidar_mode, car_mode):
        if lidar_mode == LidarMode.OUSTER.value:
            if car_mode == CarMode.CASTOR.value:
                return static_tf_publisher_ouster_launch
        elif lidar_mode == LidarMode.HESAI.value:
            if car_mode == CarMode.CASTOR.value:
                return static_tf_publisher_hesai_castor_launch
            elif car_mode == CarMode.DUFOUR.value:
                return static_tf_publisher_hesai_dufour_launch

    def launch_lidar(lidar_mode, car_mode):
        static_tf = get_static_tf_publisher_launch(lidar_mode, car_mode)
        if lidar_mode == LidarMode.OUSTER.value:
            return [
                IncludeLaunchDescription(PythonLaunchDescriptionSource(static_tf)),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(lidar_cone_detector_launch),
                    launch_arguments={"lidar_mode": lidar_mode}.items(),
                ),
            ]
        elif lidar_mode == LidarMode.HESAI.value:
            return [
                IncludeLaunchDescription(PythonLaunchDescriptionSource(static_tf)),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(hesai_driver_launch)
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(lidar_cone_detector_launch),
                    launch_arguments={"lidar_mode": lidar_mode}.items(),
                ),
            ]
        else:
            raise ValueError("Invalid lidar mode")

    def launch_sensor_fusion(lidar_mode, car_mode):
        # need to include the sensor fusion package once it's on main
        return (
            launch_camera()
            + launch_lidar(lidar_mode, car_mode)
            + [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(sensor_fusion_launch),
                    launch_arguments={"lidar_mode": lidar_mode}.items(),
                )
            ]
        )

    def launch_cone_fusion(perception_mode):
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(cone_fusion_launch),
                launch_arguments={"perception_mode": perception_mode}.items(),
            )
        ]

    def launch_pure_pursuit():
        return [
            IncludeLaunchDescription(PythonLaunchDescriptionSource(pure_pursuit_launch))
        ]

    def launch_mpc():
        return [IncludeLaunchDescription(PythonLaunchDescriptionSource(mpc_launch))]

    # Create a Python function to generate the launch description based on the sensor argument
    def generate_launch_descriptions(context):
        perception_mode = context.launch_configurations["perception_mode"]
        controller_mode = context.launch_configurations["controller_mode"]
        lidar_mode = context.launch_configurations["lidar_mode"]
        car_mode = context.launch_configurations["car_mode"]

        perception_launch_description = launch_cone_fusion(perception_mode)
        controller_launch_description = []
        if perception_mode == PerceptionMode.CAMERA_ONLY.value:
            perception_launch_description = (
                perception_launch_description + launch_camera()
            )
        elif perception_mode == PerceptionMode.LIDAR_ONLY.value:
            perception_launch_description = (
                perception_launch_description + launch_lidar(lidar_mode, car_mode)
            )
        elif perception_mode == PerceptionMode.SENSOR_FUSION.value:
            perception_launch_description = (
                perception_launch_description
                + launch_sensor_fusion(lidar_mode, car_mode)
            )
        else:
            raise ValueError("Invalid perception mode")

        if controller_mode == ControllerMode.PURE_PURSUIT.value:
            controller_launch_description = launch_pure_pursuit()
        elif controller_mode == ControllerMode.MPC.value:
            controller_launch_description = launch_mpc()
        else:
            raise ValueError("Invalid controller mode")

        common_launch_description = generate_common_launch_description(perception_mode)

        return (
            common_launch_description
            + perception_launch_description
            + controller_launch_description
        )

    # Launch description
    launch_description = [
        perception_mode_arg,
        controller_mode_arg,
        lidar_mode_arg,
        car_mode_arg,  # Add the argument to the launch description
        OpaqueFunction(function=generate_launch_descriptions),
        TimerAction(
            period=10.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        "ros2",
                        "bag",
                        "record",
                        "-a",
                        "-s",
                        "mcap",
                        # "-x",
                        # "(/lidar_points|/forward_camera/pylon_ros2_camera_node/image_raw)",
                    ],
                    output="screen",
                ),
            ],
        ),
    ]

    return LaunchDescription(launch_description)
