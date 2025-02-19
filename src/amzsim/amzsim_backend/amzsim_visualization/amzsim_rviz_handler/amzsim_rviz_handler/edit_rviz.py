#!/usr/bin/env python3

# AMZ Driverless Project
#
# Copyright (c) 2023 Authors:
# - Bartosz Mila <bamila@student.ethz.ch>
#
# All rights reserved.
#
# Unauthorized copying of this file, via any medium is strictly prohibited
# Proprietary and confidential
"""
This script allows to edit "enabled" options in rviz file to
set default options before running rviz visualization
"""
import os
import yaml
import rclpy
from rclpy.node import Node


class RvizConfigEditor(Node):
    def __init__(self):
        super().__init__("rviz_config_editor")

        self.declare_parameter("show_lap_info", True)
        self.declare_parameter("show_velocity", True)
        self.declare_parameter("show_track_length", True)
        self.declare_parameter("show_collision_detect", True)
        self.declare_parameter("show_fov_cones", False)
        self.declare_parameter("show_ego_frame", True)
        self.declare_parameter("show_veh_frame", True)
        self.declare_parameter("show_online_map", True)
        self.declare_parameter("show_bounded_path", False)
        self.declare_parameter("show_delaunay", False)
        self.declare_parameter("show_control_bounds", True)
        self.declare_parameter("rvizconfig", "Not_the_path_you_are_looking_for")

        self.edit_dict = {
            "Perception": {"ConeArray": False},
            "Estimation": {
                "Egomotion": True,
                "Local map": True,
                "Bounded path": False,
                "BE-2019": False,
                "Vehicle": True,
            },
            "Control": {
                "Speed [m/s]": True,
                "Lap Information": True,
                "Boundaries": True,
            },
        }

        # Retrieve parameters from ROS2 parameter server
        self.edit_dict["Control"]["Lap Information"] = self.get_parameter(
            "show_lap_info"
        ).value
        self.edit_dict["Control"]["Speed [m/s]"] = self.get_parameter(
            "show_velocity"
        ).value
        self.edit_dict["Perception"]["ConeArray"] = self.get_parameter(
            "show_fov_cones"
        ).value
        self.edit_dict["Estimation"]["Egomotion"] = self.get_parameter(
            "show_ego_frame"
        ).value
        self.edit_dict["Estimation"]["Vehicle"] = self.get_parameter(
            "show_veh_frame"
        ).value
        self.edit_dict["Estimation"]["Local map"] = self.get_parameter(
            "show_online_map"
        ).value
        self.edit_dict["Estimation"]["Bounded path"] = self.get_parameter(
            "show_bounded_path"
        ).value
        self.edit_dict["Estimation"]["BE-2019"] = self.get_parameter(
            "show_delaunay"
        ).value
        self.edit_dict["Control"]["Boundaries"] = self.get_parameter(
            "show_control_bounds"
        ).value
        self.rviz_path = self.get_parameter("rvizconfig").value

    def edit_yaml(self):
        with open(self.rviz_path, "r", encoding="ascii") as stream:
            data_loaded = yaml.safe_load(stream)

        number_of_groups = len(data_loaded["Visualization Manager"]["Displays"])
        for i in range(1, number_of_groups - 1):
            group = data_loaded["Visualization Manager"]["Displays"][i]
            group_name = group["Name"]
            group_content = group["rviz_common/Displays"]
            self.edit_group_dict(group_name, group_content)

        with open(self.rviz_path, "w", encoding="ascii") as file:
            yaml.dump(data_loaded, file)

    def edit_group_dict(self, group_name, group_content):
        for elem in group_content:
            class_name = elem["Name"]
            elem["Enabled"] = self.edit_dict[group_name].get(class_name, False)


def main(args=None):
    rclpy.init(args=None)
    node = RvizConfigEditor()
    node.edit_yaml()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
