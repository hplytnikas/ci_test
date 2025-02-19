#!/usr/bin/env python3
# AMZ Driverless Project
#
# Copyright (c) 2020-2023 Authors:
#   - Yilun Wu <wuyil@ethz.ch>
#   - Antonio Arbues <aarbues@ethz.ch>
#   - Stanislaw Piasecki <stanislaw.piasecki@inf.ethz.ch>
#
# All rights reserved.
#
# Unauthorized copying of this file, via any medium is strictly prohibited
# Proprietary and confidential
#

import re
import subprocess
import time

import rclpy
from rclpy.node import Node
import psutil  # >= 5.6.2
from monitoring_msgs.msg import PCResourceStat

try:
    from pynvml import (
        nvmlInit,
        nvmlDeviceGetCount,
        nvmlDeviceGetHandleByIndex,
        nvmlDeviceGetUtilizationRates,
        nvmlDeviceGetTemperature,
        nvmlDeviceGetMemoryInfo,
        NVMLError,
    )

    NVML_INSTALLED = True
except ImportError:
    NVML_INSTALLED = False


def ssd_temperature():
    # change /etc/sudoers file using "pkexec visudo" and add this line
    #  amz ALL = NOPASSWD: /usr/sbin/nvme smart-log /dev/nvme0n1
    # if you can't authorize, read this: https://askubuntu.com/a/1053617
    try:
        ssd_data = subprocess.check_output(
            ["sudo", "nvme", "smart-log", "/dev/nvme0n1"]
        )
    except subprocess.CalledProcessError:
        return 0, ""
    try:
        ssd_temp = int(
            re.search(r"temperature\s+:\s(\d+)\sC", ssd_data.decode()).group(1)
        )
    except AttributeError:
        return 0, ""
    try:
        ssd_warning = re.search(r"critical_warning\s+:\s(.+)", ssd_data.decode()).group(
            1
        )
    except AttributeError:
        return 0, ""
    return ssd_temp, ssd_warning


def get_cpu_stats(msg):
    msg.cpu_percent = psutil.cpu_percent(percpu=True)
    msg.cpu_percent_avg = psutil.cpu_percent(percpu=False)
    msg.cpu_percent_max = max(msg.cpu_percent)
    msg.cpu_freq = [o.current for o in psutil.cpu_freq(percpu=True)]
    msg.cpu_freq_avg = psutil.cpu_freq(percpu=False).current
    msg.cpu_freq_max = max(msg.cpu_freq)
    msg.load_1min = psutil.getloadavg()[0]
    msg.load_5min = psutil.getloadavg()[1]
    msg.load_15min = psutil.getloadavg()[2]
    # The 'k10temp' variable might need to be replaced depending on the CPU, 'coretemp'
    # >> python3
    # >> import psutil
    # >> psutil.sensors_temperatures()
    # --> look at the dictionary
    msg.cpu_temp = [
        float(o.current) for o in psutil.sensors_temperatures().get("coretemp", [])
    ]
    msg.cpu_temp_max = float(max(msg.cpu_temp, default=0.0))


def get_gpu_stats(msg):
    # detect GPU info
    report_gpu = False

    if NVML_INSTALLED:
        try:
            nvmlInit()
            if nvmlDeviceGetCount() > 0:
                handle = nvmlDeviceGetHandleByIndex(0)
                report_gpu = True
        except NVMLError as err:
            print(err)

    if report_gpu:
        try:
            msg.gpu_percent = float(nvmlDeviceGetUtilizationRates(handle).gpu)
            msg.gpu_temp = nvmlDeviceGetTemperature(handle, 0)
            gpu_mem_info = nvmlDeviceGetMemoryInfo(handle)
            msg.gpu_memory_total_mb = int(gpu_mem_info.total / (2**20))
            msg.gpu_memory_available_mb = int(gpu_mem_info.free / (2**20))
            msg.gpu_memory_usage_percent = float(gpu_mem_info.free) / float(
                gpu_mem_info.total
            )
        except NVMLError as err:
            print(err)


class ResourceMonitor(Node):
    def __init__(self):
        super().__init__("monitoring_resources_node")

        # Declare parameters
        self.declare_parameter("publish_frequency", 1.0)
        self.declare_parameter("topic_name", "/monitoring/pc_resource_stats")

        # Get parameter values
        self.publish_frequency = (
            self.get_parameter("publish_frequency").get_parameter_value().double_value
        )
        self.topic_name = (
            self.get_parameter("topic_name").get_parameter_value().string_value
        )

        self.pub = self.create_publisher(PCResourceStat, self.topic_name, 5)
        self.timer = self.create_timer(1.0 / self.publish_frequency, self.publish_stats)
        self.last_disk_read_bytes = psutil.disk_io_counters().read_bytes
        self.last_disk_write_bytes = psutil.disk_io_counters().write_bytes
        self.last_network_down_bytes = psutil.net_io_counters().bytes_recv
        self.last_network_up_bytes = psutil.net_io_counters().bytes_sent
        self.last_call = time.time()
        self.get_logger().info(
            f"Start PC resource monitoring with frequency: {self.publish_frequency} Hz and topic: {self.topic_name}"
        )

    def publish_stats(self):
        msg = PCResourceStat()
        msg.header.stamp = self.get_clock().now().to_msg()

        get_cpu_stats(msg)

        msg.memory_available_mb = int(psutil.virtual_memory().available / (2**20))
        msg.memory_usage_percent = psutil.virtual_memory().percent
        msg.swap_available_mb = int(psutil.swap_memory().free / (2**20))
        msg.swap_usage_percent = psutil.swap_memory().percent

        msg.disk_free_gb = int(psutil.disk_usage("/").free / (2**30))
        msg.disk_usage_percent = psutil.disk_usage("/").percent

        d_time = time.time() - self.last_call
        self.last_call = time.time()

        rate, self.last_disk_read_bytes = update_rate(
            d_time, psutil.disk_io_counters().read_bytes, self.last_disk_read_bytes
        )
        msg.disk_read_mbs = rate / (2**20)

        rate, self.last_disk_write_bytes = update_rate(
            d_time, psutil.disk_io_counters().write_bytes, self.last_disk_write_bytes
        )
        msg.disk_write_mbs = rate / (2**20)

        rate, self.last_network_down_bytes = update_rate(
            d_time, psutil.net_io_counters().bytes_recv, self.last_network_down_bytes
        )
        msg.network_down_mbs = rate / (2**20)

        rate, self.last_network_up_bytes = update_rate(
            d_time, psutil.net_io_counters().bytes_sent, self.last_network_up_bytes
        )
        msg.network_up_mbs = rate / (2**20)

        msg.ssd_temp, msg.ssd_critical_warning = ssd_temperature()

        get_gpu_stats(msg)

        self.pub.publish(msg)


def update_rate(d_time, new_value, prev_value):
    rate = (new_value - prev_value) / d_time
    return rate, new_value


def main(args=None):
    rclpy.init(args=args)
    resource_monitor = ResourceMonitor()
    rclpy.spin(resource_monitor)
    resource_monitor.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
