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
This script allows to generate csvtrack files in the same format as other tracks
from trackdrive_latest bags. This way we can run simulation during tests on the similar tracks
as in reality by extracting online map from succesful autocross run.
To generate new track simply run the script track_bag_csv_parses.py and pass arguments
bag_name - name of trackdrive latest bag to parse into standard track csv format
csv_name - name of the track after parsing
discipline - discipline where csv should be saved, for example 'skidpad'
example: python3 track_bag_csv_parser.py
--bag_name="Test_21.04"
--csv_name="Test_21.04"
--discipline="autocross_trackdrive"
This will generate Test_21.04.csv file in tracks_csv/global_map/autocross_trackdrive directory
from tracks_bags/Test_21.04.bag.
For this to work run the script from the same directory where script is located.
IMPORTANT: Global map is not exactly 1 to 1 with cones in reality, as there is always some
noise resulting from perception and SLAM, so the track is not exactly the same as in reality
"""

import argparse
import csv
import rosbag

# pylint: disable= import-error

parser = argparse.ArgumentParser(description="Script to generate tracks from bag files")
parser.add_argument("--bag_name", type=str)
parser.add_argument("--csv_name", type=str)
parser.add_argument("--track_type", type=str)
parser.add_argument("--discipline", type=str)

args = parser.parse_args()

bag_name = args.bag_name
csv_name = args.csv_name
track_type = args.track_type
discipline = args.discipline


def bag_to_csv_parse():
    """
    Parses rosbag cone array to csv cone array.
    """

    cones = get_cones_from_bag()
    write_cones_to_csv(cones=cones)


def get_cones_from_bag():
    """
    Gets cones from rosbag and saves them in the variable.
    """

    path_to_bag = "../tracks_bags/" + bag_name + ".bag"
    bag = rosbag.Bag(path_to_bag)
    messages = []
    cones = []
    for full_msg in bag.read_messages(topics=["/estimation/global_map"]):
        msg = full_msg[1]
        messages.append(msg)
    cone_array = messages[-1]
    for cone in cone_array.cones:
        cone_color = __get_cone_color(cone=cone)
        cone_x = cone.position.x
        cone_y = cone.position.y
        cones.append([cone_color, cone_x, cone_y])
    bag.close()
    return cones


def __get_cone_color(cone):
    """
    Gets cone color based on probability color.
    """

    dict_probs = {}
    dict_probs["orange_big"] = cone.prob_type.orange_big
    dict_probs["orange"] = cone.prob_type.orange
    dict_probs["yellow"] = cone.prob_type.yellow
    dict_probs["blue"] = cone.prob_type.blue
    return max(dict_probs, key=dict_probs.get)


def write_cones_to_csv(cones):
    """
    Writes cone list to csv file.
    """

    path_to_file = "../tracks_csv/global_map/" + discipline + "/" + csv_name + ".csv"

    with open(path_to_file, "w", newline="", encoding="utf-8") as csv_file:
        fieldnames = ["tag", "x", "y"]
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        for cone in cones:
            row_csv = {"tag": cone[0], "x": cone[1], "y": cone[2]}
            writer.writerow(row_csv)


if __name__ == "__main__":
    bag_to_csv_parse()
