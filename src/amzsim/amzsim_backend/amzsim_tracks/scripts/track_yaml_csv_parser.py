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

# This script allows to generate csv track files from YAML files in desired format for simulator.
# To generate new track simply run the script track_yaml_csv_parser.py and pass arguments
# yaml_name - name of track to parse in yaml_file
# csv_name - name of track csv file after parse
# track_type - track type where csv should be saved, for example 'standard'
# discipline - discipline where csv should be saved, for example 'skidpad'
# example: python3 track_yaml_csv_parser.py
# --track_type="standard"
# --discipline="autocross_trackdrive"
# --yaml_name="FSG"
# --csv_name="FSG"
# This will generate FSG.csv file in tracks_csv/standard/autocross_trackdrive directory
# from FSG.yaml in tracks_yaml directory
# For this to work run the script from the same directory where script is located

import argparse
import csv
import yaml

parser = argparse.ArgumentParser(
    description="Script to generate csv tracks from yaml files"
)
parser.add_argument("--yaml_name", type=str)
parser.add_argument("--csv_name", type=str)
parser.add_argument("--track_type", type=str)
parser.add_argument("--discipline", type=str)

args = parser.parse_args()

yaml_name = args.yaml_name
csv_name = args.csv_name
track_type = args.track_type
discipline = args.discipline


def yaml_to_csv_parse():
    with open(
        "../tracks_yaml/" + yaml_name + ".yaml", "r", encoding="utf-8"
    ) as yaml_file:
        yaml_loaded = yaml.safe_load(yaml_file)

    cones_orange_big = yaml_loaded.get("cones_orange_big")
    cones_orange = yaml_loaded.get("cones_orange")
    blue_cones = yaml_loaded.get("cones_left")
    yellow_cones = yaml_loaded.get("cones_right")

    if discipline is not None:
        path_to_file = (
            "../tracks_csv/" + track_type + "/" + discipline + "/" + csv_name + ".csv"
        )
    else:
        path_to_file = "../tracks_csv/" + track_type + "/" + csv_name + ".csv"

    with open(path_to_file, "w", newline="", encoding="utf-8") as csv_file:
        fieldnames = ["tag", "x", "y"]
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        write_cones_to_csv(csv_writer=writer, cones=cones_orange_big, tag="orange_big")
        write_cones_to_csv(csv_writer=writer, cones=cones_orange, tag="orange")
        write_cones_to_csv(csv_writer=writer, cones=blue_cones, tag="blue")
        write_cones_to_csv(csv_writer=writer, cones=yellow_cones, tag="yellow")


def write_cones_to_csv(csv_writer, cones, tag):
    for cone_x_y in cones:
        row_csv = {"tag": tag, "x": cone_x_y[0], "y": cone_x_y[1]}
        csv_writer.writerow(row_csv)


if __name__ == "__main__":
    yaml_to_csv_parse()
