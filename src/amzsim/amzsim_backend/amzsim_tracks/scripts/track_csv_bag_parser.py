#!/usr/bin/env python3

# AMZ Driverless Project
#
# Copyright (c) 2023 Authors:
# - Bartosz Mila <bamila@student.ethz.ch>
# - Jonas Grütter <jgruette@student.ethz.ch>
#
# All rights reserved.
#
# Unauthorized copying of this file, via any medium is strictly prohibited
# Proprietary and confidential

# This script allows to generate bag track files from csv files in same format as slam does it.
# This bag file is necessary for trackdrive to work, it "simulates" the first lap
# where usually track mapping with slam would occur
# To generate new track simply run the script track_csv_bag_parser.py and pass arguments
# csv_name - name of track to parse in csv_file
# example: python track_csv_bag_parser.py --csv_name="FSG"
# This will generate FSG.bag file in tracks_bags directory from FSG.csv in tracks_csv directory
# For this to work run the script from the same directory where script is located

# SCRIPT CURRENTLY NOT USED BUT MIGHT BE USEFUL FOR FUTURE! PROBABLY REQUIRES UPDATE

import argparse
import csv
import rosbag

# pylint: disable= import-error
from autonomous_msgs.msg import ConeArray, Cone

parser = argparse.ArgumentParser(
    description="Script to generate bag tracks from csv files"
)
parser.add_argument("--csv_name", type=str)

args = parser.parse_args()

csv_name = args.csv_name


def csv_to_bag_parse():
    cone_array = create_cone_array_from_csv()
    cone_msg_array = create_cone_msg_array(cone_array)
    write_cone_msg_array_to_bag(cone_msg_array)


def create_cone_array_from_csv():
    cone_array = []
    with open("../tracks_csv/" + csv_name + ".csv", "r", encoding="utf-8") as csv_file:
        csv_reader = csv.reader(csv_file)
        next(csv_reader, None)
        for row in csv_reader:  # each row is a list
            cone_array.append(row[1:])
    return cone_array


def create_cone_msg_array(cone_array) -> ConeArray:
    cone_msg_list = []
    cone_msg_array = ConeArray()
    for cone in cone_array:
        cone_msg = Cone()
        cone_msg.id_cone = 0
        cone_msg.pipeline = 0
        cone_msg.prob_cone = 1.0
        cone_msg.is_observed = True
        cone_msg.position.x = float(cone[0])
        cone_msg.position.y = float(cone[1])
        cone_msg.position.z = 0.0
        cone_msg_list.append(cone_msg)
    cone_msg_array.header.frame_id = "world"
    cone_msg_array.cones = cone_msg_list
    return cone_msg_array


def write_cone_msg_array_to_bag(cone_array):
    try:
        bag = rosbag.Bag("../tracks_bags/" + csv_name + ".bag", "w")
        bag.write("/estimation/global_map", cone_array)
    finally:
        bag.close()


if __name__ == "__main__":
    csv_to_bag_parse()
