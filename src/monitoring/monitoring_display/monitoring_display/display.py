#!/usr/bin/env python3

# AMZ Driverless Project
#
# Copyright (c) 2023-2024 Authors:
#   - Jonas Ohnemus <johnemus@ethz.ch>
#
# All rights reserved.
#
# Unauthorized copying of this file, via any medium is strictly prohibited
# Proprietary and confidential

import os
import datetime
import subprocess
import time
import socket
import math
import threading
import collections
from urllib.parse import urlparse
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from termcolor import colored

from vcu_msgs.msg import ResState, MissionSelect, LvBattery, StateMachine, EbsPressures
from monitoring_msgs.msg import PipelineStatus
from monitoring_msgs.msg import PCResourceStat
from autonomous_msgs.msg import IntStamped, DoubleStamped

from monitoring_display.display_items import (
    FULL_ROW_WIDTH,
    DisplayLine,
    DisplayTitle,
    EmptyCol,
    DisplaySet,
    color_threshold,
    bool_color,
)


class DisplayBlock:

    def __init__(self):
        self.update_timeout = 2
        self.msg = None
        self.update_time = time.time()
        self.title = ""

    def get_title(self, latest_update, update_timeout=2):
        row_list = []
        if latest_update > update_timeout:
            row_list.append(
                DisplayTitle(
                    f"{self.title}-not updated {round(latest_update, 1)}s", color="red"
                )
            )
        else:
            row_list.append(DisplayTitle(self.title))
        row_list.append(EmptyCol())
        return row_list

    def msg_update(self, msg, topic):
        pass


class CarStartup(DisplayBlock):

    def __init__(self, title="Car Startup"):
        super().__init__()
        self.title = title

    def assemble(self):
        output = self.get_title(0)
        output.extend(
            [
                DisplayLine("EBS status", "", "red"),
            ]
        )
        return output


def check_rosnode_output(obj):
    while True:
        try:
            rosnode_output = subprocess.getoutput("ros2 node list")
            obj.output = rosnode_output.split("\n")
            obj.update_time = time.time()
        except Exception:  # pylint: disable=broad-except
            pass
        time.sleep(2)


class RosNodeListDisplay(DisplayBlock):

    def __init__(self, title="Nodes"):
        super().__init__()
        self.output = []
        self.update_timeout = 4
        self.update_time = time.time()
        self.title = title

        thread_1 = threading.Thread(target=check_rosnode_output, args=(self,))
        thread_1.daemon = True
        thread_1.start()

    def assemble(self):
        latest_update = time.time() - self.update_time
        output = self.get_title(latest_update, update_timeout=4)
        if self.output:
            output.extend(
                [
                    DisplayLine(
                        (
                            "Recording large rosbag"
                            if "/rosbag_record_topics" in self.output
                            else "Not recording large rosbag"
                        ),
                        "",
                        bool_color("/rosbag_record_topics" in self.output),
                    ),
                    DisplayLine(
                        (
                            "Recording small rosbag"
                            if "/rosbag_record_topics_small" in self.output
                            else "Not recording small rosbag"
                        ),
                        "",
                        bool_color("/rosbag_record_topics_small" in self.output),
                    ),
                ]
            )
        return output


class PipelineDisplay(DisplayBlock):

    def __init__(self, title="Pipeline"):
        super().__init__()
        self.title = title
        self.mission_name = "Not selected"
        self.mission_color = "red"
        self.lap_counter = "0"
        self.lap_counter_color = "red"

        self.lap_time = collections.deque(maxlen=4)
        self.lap_colors = collections.deque(maxlen=4)
        for _ in range(self.lap_time.maxlen):
            self.lap_time.append(0.00)
        for _ in range(self.lap_colors.maxlen):
            self.lap_colors.append("green")

    def assemble(self):
        latest_update = time.time() - self.update_time
        output = self.get_title(latest_update)
        if self.msg is not None:
            output.extend(
                [
                    DisplayLine("Mission", self.mission_name, self.mission_color),
                    DisplayLine(
                        "Current lap count", self.lap_counter, self.lap_counter_color
                    ),
                    DisplayLine(
                        "Lap time of previous lap",
                        f"{self.lap_time[-1]:.2f}s",
                        self.lap_colors[-1],
                    ),
                    DisplayLine(
                        "Lap time of lap -2",
                        f"{self.lap_time[-2]:.2f}s",
                        self.lap_colors[-2],
                    ),
                    DisplayLine(
                        "Lap time of lap -3",
                        f"{self.lap_time[-3]:.2f}s",
                        self.lap_colors[-3],
                    ),
                    DisplayLine(
                        "Lap time of lap -4",
                        f"{self.lap_time[-4]:.2f}s",
                        self.lap_colors[-4],
                    ),
                    EmptyCol(),
                    DisplayLine(
                        "Camera Pipeline",
                        f"{self.msg.num_cones_camera} cones {self.msg.camera_pipeline.msg_hz_1s}Hz",
                        color_threshold(
                            self.msg.camera_pipeline.msg_hz_1s, 5, 8, direction="le"
                        ),
                    ),
                    DisplayLine(
                        "LiDAR Pipeline",
                        f"{self.msg.num_cones_lidar} cones {self.msg.lidar_pipeline.msg_hz_1s}Hz",
                        color_threshold(
                            self.msg.lidar_pipeline.msg_hz_1s, 5, 8, direction="le"
                        ),
                    ),
                    DisplayLine(
                        "Fusion Pipeline",
                        f"{self.msg.num_cones_fusion} cones {self.msg.fusion_pipeline.msg_hz_1s}Hz",
                        color_threshold(
                            self.msg.fusion_pipeline.msg_hz_1s, 5, 8, direction="le"
                        ),
                    ),
                    DisplayLine(
                        "PER Output",
                        f"{self.msg.num_cones_per} cones {self.msg.per_pipeline.msg_hz_1s}Hz",
                        color_threshold(
                            self.msg.per_pipeline.msg_hz_1s, 5, 8, direction="le"
                        ),
                    ),
                    DisplayLine(
                        "Online map",
                        f"{self.msg.num_cones_map} cones {self.msg.online_map.msg_hz_1s}Hz",
                        color_threshold(
                            self.msg.online_map.msg_hz_1s, 10, 15, direction="le"
                        ),
                    ),
                    DisplayLine(
                        "Boundary estimation",
                        f"{self.msg.boundary_estimation.msg_hz_1s}Hz",
                        color_threshold(
                            self.msg.boundary_estimation.msg_hz_1s,
                            10,
                            15,
                            direction="le",
                        ),
                    ),
                    DisplayLine(
                        "Control",
                        f"{self.msg.control.msg_hz_1s}Hz",
                        color_threshold(
                            self.msg.control.msg_hz_1s, 10, 15, direction="le"
                        ),
                    ),
                ]
            )
        return output

    def msg_update(self, msg, topic):
        if topic == "/monitoring/pipeline_status":
            self.msg = msg
            self.update_time = time.time()
        elif topic == "/vcu_msgs/mission_select":
            self.mission_name = msg.mission_name
            self.mission_color = "green"
        elif topic == "/lap_counter":
            self.lap_counter = str(msg.data)
            self.lap_counter_color = "green"
        elif topic == "/lap_time":
            self.lap_time.append(msg.data)
            if float(self.lap_time[-1]) <= float(self.lap_time[-2]):
                self.lap_colors.append("green")
            else:
                self.lap_colors.append("red")


class BaseVehicleDisplay(DisplayBlock):

    def __init__(self, title="Base Vehicle"):
        super().__init__()
        self.title = title
        self.lv_battery_msg = None
        self.state_machines_msg = None
        self.ebs_pressure_msg = None

    def assemble(self):
        latest_update = time.time() - self.update_time
        output = self.get_title(latest_update, update_timeout=self.update_timeout)
        if self.lv_battery_msg is not None:
            output.extend(
                [
                    DisplayLine(
                        "LV SoC",
                        f"{(self.lv_battery_msg.lv_soc * 100):.0f} %",
                        color_threshold(
                            self.lv_battery_msg.lv_soc * 100, 20, 30, direction="le"
                        ),
                    )
                ]
            )
        if self.ebs_pressure_msg is not None:
            color_air_pressure_front = color_threshold(
                self.ebs_pressure_msg.air_pressure_front, [6.5, 7.5], [6.8, 7.2]
            )
            color_air_pressure_rear = color_threshold(
                self.ebs_pressure_msg.air_pressure_rear, [6.5, 7.5], [6.8, 7.2]
            )
            output.extend(
                [
                    DisplayLine(
                        "EBS air pressure front",
                        f"{self.ebs_pressure_msg.air_pressure_front:.2f} bar",
                        color_air_pressure_front,
                    ),
                    DisplayLine(
                        "EBS air pressure rear",
                        f"{self.ebs_pressure_msg.air_pressure_rear:.2f} bar",
                        color_air_pressure_rear,
                    ),
                    DisplayLine(
                        "Brake presssure front",
                        f"{self.ebs_pressure_msg.brake_pressure_front:.2f} bar",
                        "green",
                    ),
                    DisplayLine(
                        "Brake pressure rear",
                        f"{self.ebs_pressure_msg.brake_pressure_rear:.2f} bar",
                        "green",
                    ),
                ]
            )
        if self.state_machines_msg is not None:
            ebs_state = self.state_machines_msg.ebs_state
            as_state = self.state_machines_msg.as_state

            if ebs_state is None:
                ebs_state = "unknown"

            if as_state is None:
                as_state = "unknown"

            output.extend(
                [
                    DisplayLine(
                        "EBS state",
                        f"{self.state_machines_msg.ebs_state} ({ebs_state})",
                    ),
                    DisplayLine(
                        "AS state", f"{self.state_machines_msg.as_state} ({as_state})"
                    ),
                ]
            )
        return output

    def msg_update(self, msg, topic):
        if topic == "/vcu_msgs/lv_battery":
            self.lv_battery_msg = msg
            self.update_time = time.time()
        elif topic == "/vcu_msgs/state_machine":
            self.state_machines_msg = msg
            self.update_time = time.time()
        elif topic == "/vcu_msgs/ebs_pressures":
            self.ebs_pressure_msg = msg
            self.update_time = time.time()


class ResDisplay(DisplayBlock):

    def __init__(self, title="RES"):
        super().__init__()
        self.title = title

    def assemble(self):
        latest_update = time.time() - self.update_time
        output = self.get_title(latest_update)
        if self.msg is not None:
            output.extend(
                [
                    DisplayLine("Emergency", "", bool_color(self.msg.emergency)),
                    DisplayLine(
                        "On/Off switch", "", bool_color(self.msg.on_off_switch)
                    ),
                    DisplayLine("Push button", "", bool_color(self.msg.push_button)),
                    DisplayLine(
                        "Communication interrupted",
                        "",
                        bool_color(self.msg.communication_interrupted),
                    ),
                ]
            )
        return output

    def msg_update(self, msg, topic):
        if topic == "/vcu_msgs/res_state":
            self.msg = msg
            self.update_time = time.time()


class SensorsDisplay(DisplayBlock):

    def __init__(self, title="Sensors"):
        super().__init__()
        self.title = title
        self.sensor_states = {"ass": False, "imu": False, "ins": False}

    def assemble(self):
        latest_update = time.time() - self.update_time
        output = self.get_title(latest_update)
        if self.msg is not None:
            output.extend(
                [
                    DisplayLine(
                        "LiDAR",
                        f"{self.msg.sensor_lidar.msg_hz_1s}Hz",
                        color_threshold(
                            self.msg.sensor_lidar.msg_hz_1s, 5, 8, direction="le"
                        ),
                    ),
                    DisplayLine(
                        "Camera",
                        f"{self.msg.sensor_camera.msg_hz_1s}Hz",
                        color_threshold(
                            self.msg.sensor_camera.msg_hz_1s, 5, 8, direction="le"
                        ),
                    ),
                    DisplayLine(
                        "VE",
                        f"{self.msg.velocity_estimation.msg_hz_1s}Hz",
                        color_threshold(
                            self.msg.velocity_estimation.msg_hz_1s,
                            150,
                            190,
                            direction="le",
                        ),
                    ),
                    DisplayLine(
                        "SteeringFeedback",
                        f"{self.msg.steering_feedback.msg_hz_1s}Hz",
                        color_threshold(
                            self.msg.steering_feedback.msg_hz_1s,
                            150,
                            190,
                            direction="le",
                        ),
                    ),
                    EmptyCol(),
                ]
            )

            def sensor_state_str(sensor_state):
                if sensor_state:
                    return ""
                return "not"

            output.extend(
                [
                    DisplayLine(
                        "IMU data",
                        f'{sensor_state_str(self.sensor_states["imu"])} present',
                        bool_color(self.sensor_states["imu"]),
                    ),
                    DisplayLine(
                        "ASS data",
                        f'{sensor_state_str(self.sensor_states["ass"])} present',
                        bool_color(self.sensor_states["ass"]),
                    ),
                    DisplayLine(
                        "INS data",
                        f'{sensor_state_str(self.sensor_states["ins"])} present',
                        bool_color(self.sensor_states["ins"]),
                    ),
                ]
            )
        return output

    def msg_update(self, msg, topic):
        if topic == "/monitoring/pipeline_status":
            self.msg = msg
            self.update_time = time.time()
        elif topic == "/vcu_msgs/state_machine":
            self.sensor_states["ass"] = not msg.ass_fail
            self.sensor_states["imu"] = not msg.imu_fail
            self.sensor_states["ins"] = not msg.ins_fail


class ComputerDisplay(DisplayBlock):

    def __init__(self, title="Computer CB"):
        super().__init__()
        self.title = title

    def assemble(self):
        latest_update = time.time() - self.update_time
        output = self.get_title(latest_update)
        if self.msg is not None:
            date = datetime.datetime.fromtimestamp(
                self.msg.header.stamp.sec + self.msg.header.stamp.nanosec * 1e-9
            )
            output.extend(
                [
                    DisplayLine(
                        "CPU usage avg",
                        f"{self.msg.cpu_percent_avg:.2f}%",
                        color_threshold(
                            self.msg.cpu_percent_avg, 80, 50, direction="ge"
                        ),
                    ),
                    DisplayLine(
                        "CPU usage max",
                        f"{self.msg.cpu_percent_max:.2f}%",
                        color_threshold(
                            self.msg.cpu_percent_max, 80, 50, direction="ge"
                        ),
                    ),
                    DisplayLine(
                        "CPU temp max",
                        f"{self.msg.cpu_temp_max:.2f} deg",
                        color_threshold(self.msg.cpu_temp_max, 90, 80, direction="ge"),
                    ),
                    DisplayLine("GPU percent", f"{self.msg.gpu_percent:.2f}%", "green"),
                    DisplayLine("GPU temp", f"{self.msg.gpu_temp:.2f} deg", "green"),
                    DisplayLine(
                        "GPU mem usage",
                        f"{self.msg.gpu_memory_usage_percent:.2f}%",
                        "green",
                    ),
                    DisplayLine(
                        "Memory usage",
                        f"{self.msg.memory_usage_percent:.2f}%",
                        color_threshold(
                            self.msg.memory_usage_percent, 80, 40, direction="ge"
                        ),
                    ),
                    DisplayLine(
                        "Disk usage percent",
                        f"{self.msg.disk_usage_percent:.2f}%",
                        "green" if self.msg.disk_usage_percent < 80 else "red",
                    ),
                    DisplayLine(
                        "Disk free",
                        f"{self.msg.disk_free_gb:.2f}GB",
                        "green" if self.msg.disk_free_gb > 300 else "red",
                    ),
                    DisplayLine(
                        "Disk read",
                        f"{self.msg.disk_read_mbs:.2f}MBs",
                        color_threshold(self.msg.disk_read_mbs, 80, 50, direction="ge"),
                    ),
                    DisplayLine(
                        "Disk write",
                        f"{self.msg.disk_write_mbs:.2f}MBs",
                        color_threshold(
                            self.msg.disk_write_mbs, 80, 50, direction="ge"
                        ),
                    ),
                    DisplayLine(
                        "Disk temp",
                        f"{self.msg.ssd_temp:.2f} deg",
                        color_threshold(self.msg.ssd_temp, 79, 76, direction="ge"),
                    ),
                    DisplayLine(
                        "Disk status",
                        f"{self.msg.ssd_critical_warning}",
                        "green" if self.msg.ssd_critical_warning == "0" else "red",
                    ),
                    DisplayLine(
                        "Network down",
                        f"{self.msg.network_down_mbs:.2f}MBs",
                        color_threshold(
                            self.msg.network_down_mbs, 150, 50, direction="ge"
                        ),
                    ),
                    DisplayLine(
                        "Network up",
                        f"{self.msg.network_up_mbs:.2f}MBs",
                        color_threshold(
                            self.msg.network_up_mbs, 150, 50, direction="ge"
                        ),
                    ),
                    DisplayLine(
                        "(CB) Computer time", f"{date.strftime('%b %d %H:%M:%S')}s"
                    ),
                ]
            )
        return output

    def msg_update(self, msg, topic):
        if topic == "/monitoring/pc_resource_stats":
            self.msg = msg
            self.update_time = time.time()


def flush_screen():
    print(chr(27) + "[2J")
    os.system("clear")


class Display:

    def __init__(self, columns=2):
        self.displays = [
            ComputerDisplay(),
            PipelineDisplay(),
            SensorsDisplay(),
            ResDisplay(),
            BaseVehicleDisplay(),
            RosNodeListDisplay(),
        ]
        self.columns = columns
        self.screen_updates = 0
        self.text_length = int(self.columns * FULL_ROW_WIDTH - 54)

        self.counter = 0
        self.colors = ["red", "yellow", "green", "blue", "cyan", "magenta"]
        self.texts = ["AMZ", "Driverless", "Mission", "Monitoring", "Utility"]

    def create_title(self):
        title = int(self.text_length / 2) * " "
        for i, _ in enumerate(self.texts):
            title += (
                colored(
                    self.texts[i],
                    self.colors[(self.counter + i) % 6],
                    attrs=["bold", "dark"],
                )
                + " "
            )
        title += colored("Display", attrs=["bold", "dark"])
        return title

    def update_screen(self):
        output = ""
        output += self.create_title()
        output += "\n\n"
        rows_to_create = int(math.ceil(len(self.displays) / float(self.columns)))
        for row_num in range(rows_to_create):
            cols = []
            for col_num in range(self.columns):
                item = row_num * self.columns + col_num
                if item < len(self.displays):
                    cols.append(self.displays[item].assemble())
                else:
                    cols.append([])
            output += DisplaySet(cols).render()
        output += f"\nCurrent Display time {datetime.datetime.now()}\n"

        # Get the hostname of the current machine
        host = socket.gethostname()
        output += f"Display is launched on: {host}\n"

        flush_screen()
        print(output)
        self.counter += 1

    def run(self):
        self.screen_updates += 1
        self.update_screen()

    def msg_update(self, message, topic):
        for display in self.displays:
            display.msg_update(message, topic)


class MonitoringDisplayNode(Node):

    def __init__(self, display):
        super().__init__("monitoring_display_node")
        self.display = display
        qos_profile = QoSProfile(depth=10)

        self.subscribers = [
            self.create_subscription(
                PCResourceStat,
                "/monitoring/pc_resource_stats",
                self.callback,
                qos_profile,
            ),
            self.create_subscription(
                PipelineStatus,
                "/monitoring/pipeline_status",
                self.callback,
                qos_profile,
            ),
            self.create_subscription(
                ResState, "/vcu_msgs/res_state", self.callback, qos_profile
            ),
            self.create_subscription(
                LvBattery, "/vcu_msgs/lv_battery", self.callback, qos_profile
            ),
            self.create_subscription(
                StateMachine, "/vcu_msgs/state_machine", self.callback, qos_profile
            ),
            self.create_subscription(
                EbsPressures, "/vcu_msgs/ebs_pressures", self.callback, qos_profile
            ),
            self.create_subscription(
                MissionSelect, "/vcu_msgs/mission_select", self.callback, qos_profile
            ),
            self.create_subscription(
                IntStamped, "/lap_counter", self.callback, qos_profile
            ),
            self.create_subscription(
                DoubleStamped, "/lap_time", self.callback, qos_profile
            ),
        ]

    def callback(self, msg):
        topic = self.get_subscription_topic_name(msg)
        self.display.msg_update(msg, topic)

    def get_subscription_topic_name(self, msg):
        for subscriber in self.subscribers:
            if subscriber.msg_type == type(msg):
                return subscriber.topic_name
        return ""


def main(args=None):
    rclpy.init(args=args)
    display = Display()

    monitoring_display_node = MonitoringDisplayNode(display)

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(monitoring_display_node)

    try:
        while rclpy.ok():
            rclpy.spin_once(monitoring_display_node, timeout_sec=0.1)
            display.run()
    except KeyboardInterrupt:
        pass
    finally:
        monitoring_display_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
