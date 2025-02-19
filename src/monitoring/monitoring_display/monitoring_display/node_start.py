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

import sys
import argparse
import threading
import time
import queue
from urllib.parse import urlparse

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from vcu_msgs.msg import ResState, MissionSelect, LvBattery, StateMachine, EbsPressures
from monitoring_msgs.msg import PipelineStatus, PCResourceStat
from autonomous_msgs.msg import IntStamped, DoubleStamped
from std_msgs.msg import Float32, Int32

from monitoring_display.display import Display
from monitoring_display.display_items import colored, bool_color


class RosConnection(Node):

    def __init__(self, subscriber_configs, msg_queue):
        super().__init__("monitoring_display")

        # Get parameter values
        self.update_rate = 2.0

        self.subscriber_configs = subscriber_configs
        self.msg_queue = msg_queue
        self.init_subscribers()

    def get_update_rate(self):
        return max(self.update_rate, 0.1)

    def init_subscribers(self):
        for topic, msg_type in self.subscriber_configs:
            self.create_subscription(
                msg_type, topic, lambda msg, t=topic: self.receive_msg(msg, t), 10
            )

    def receive_msg(self, msg, topic):
        # self.get_logger().info(f"Received message on topic: {topic}")
        self.msg_queue.put((msg, topic))


def check_ros_master():
    while True:
        try:
            rclpy.init(args=None)
            node = rclpy.create_node("master_check")
            node.get_logger().info("ROS Master is online.")
            node.destroy_node()
            rclpy.shutdown()
            return True
        except Exception as e:
            print(e)
            time.sleep(2.0)


def main(columns=2):
    print("Trying to connect to master...")
    if not check_ros_master():
        sys.exit()

    rclpy.init(args=None)

    msg_queue = queue.Queue()
    ros_connection = RosConnection(
        subscriber_configs=[
            ["/monitoring/pc_resource_stats", PCResourceStat],
            ["/monitoring/pipeline_status", PipelineStatus],
            ["/vcu_msgs/res_state", ResState],
            ["/vcu_msgs/lv_battery", LvBattery],
            ["/vcu_msgs/state_machine", StateMachine],
            ["/vcu_msgs/ebs_pressures", EbsPressures],
            ["/vcu_msgs/mission_select", MissionSelect],
            ["/lap_counter", Int32],
            ["/lap_time", Float32],
        ],
        msg_queue=msg_queue,
    )
    executor = MultiThreadedExecutor()
    executor.add_node(ros_connection)

    display = Display(columns=columns)

    def process_messages():
        while rclpy.ok():
            try:
                msg, topic = msg_queue.get(timeout=1.0)
                display.msg_update(msg, topic)
            except queue.Empty:
                continue

    msg_thread = threading.Thread(target=process_messages, daemon=True)
    msg_thread.start()

    def display_updates():
        while rclpy.ok():
            display.run()
            time.sleep(1.0 / ros_connection.get_update_rate())

    display_thread = threading.Thread(target=display_updates, daemon=True)
    display_thread.start()

    try:
        while rclpy.ok():
            rclpy.spin_once(ros_connection, timeout_sec=1.0)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="GUI for monitoring",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--columns", help="Number of columns max(4)", default=2, type=int
    )
    args = parser.parse_args()
    main(args.columns)
