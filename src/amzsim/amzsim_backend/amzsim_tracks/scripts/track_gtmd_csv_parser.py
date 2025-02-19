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

# This script allows to generate csv track files in the same format as other tracks
# from csv gtmd track. This way we can run simulation during tests on the same tracks
# as in reality with the usage of gtmd. To generate new track simply run the script
# track_gtmd_csv_parser.py and pass arguments
# gtmd_name - name of gtmd track to parse into standard track csv format
# csv_name - name of the track after parsing
# discipline - discipline where csv should be saved, for example 'skidpad'
# example: python3 track_gtmd_csv_parser.py
# --gtmd_name="Test_21.04"
# --csv_name="Test_21.04"
# --discipline="autocross_trackdrive"
# This will generate Test_21.04.csv file in tracks_csv/gtmd/autocross_trackdrive directory
# from track_gtmd_raw/Test_21.04.csv.
# For this to work run the script from the same directory where script is located.

import argparse
import math
import numpy as np
import pandas as pd

parser = argparse.ArgumentParser(
    description="Script to generate csv tracks from gtmd csv files"
)
parser.add_argument("--gtmd_name", type=str)
parser.add_argument("--csv_name", type=str)
parser.add_argument("--discipline", type=str)

args = parser.parse_args()

gtmd_name = args.gtmd_name
csv_name = args.csv_name
discipline = args.discipline


def gtmd_to_standard_csv_parse():
    cone_df = pd.read_csv(
        "../tracks_gtmd_raw/" + gtmd_name + ".csv",
        names=["color", "latitude", "longitude", "x", "y", "is_origin", "distance"],
    )
    cone_df = transform_df_format(cone_df=cone_df)
    cone_df = __flip_track(cone_df=cone_df)
    cone_df = transform_cones(cone_df=cone_df)
    cone_df = rotate_cones(cone_df=cone_df)
    cone_df = move_car_back(cone_df=cone_df)

    path_to_file = "../tracks_csv/gtmd/" + discipline + "/" + csv_name + ".csv"
    cone_df.to_csv(path_to_file, index=False)


def transform_df_format(cone_df):
    cone_df = cone_df.drop(["latitude", "longitude", "is_origin", "distance"], axis=1)
    cone_df = cone_df.rename(columns={"color": "tag"})
    cone_df["tag"] = cone_df["tag"].replace("bigorange", "orange_big")
    return cone_df


def transform_cones(cone_df):
    middle_point = __find_middle_point(cone_df=cone_df)
    mp_x, mp_y = middle_point["x"], middle_point["y"]
    cone_df["x"] = cone_df["x"] - mp_x
    cone_df["y"] = cone_df["y"] - mp_y
    return cone_df


def rotate_cones(cone_df):
    rotation_matrix = __calculate_rotation_matrix(cone_df=cone_df)
    rotated_positions = pd.DataFrame(cone_df[["x", "y"]].values).dot(rotation_matrix)
    cone_df[["x", "y"]] = rotated_positions
    return cone_df


def move_car_back(cone_df):
    cone_df["x"] = cone_df["x"] + 3
    return cone_df


def __calculate_rotation_matrix(cone_df):
    point = __find_positive_orange_cone(cone_df)
    p_x, p_y = point["x"], point["y"]
    angle = math.atan2(p_y, p_x) + 3 / 8 * math.pi
    return pd.DataFrame(
        [[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]]
    )


def __find_middle_point(cone_df):
    big_orange_cones = cone_df.get(cone_df["tag"] == "orange_big")
    big_orange_cones = big_orange_cones.drop(["tag"], axis=1)
    middle_point = big_orange_cones.mean()
    return middle_point


def __find_positive_orange_cone(cone_df):
    big_orange_cones = cone_df.get(cone_df["tag"] == "orange_big")
    big_orange_cones = big_orange_cones.drop(["tag"], axis=1)
    big_orange_cones = big_orange_cones.sort_values(by=["x"], ascending=False)
    orange_cone = big_orange_cones.iloc[0]
    return orange_cone


def __flip_track(cone_df):
    cone_df[["x", "y"]] = np.fliplr(cone_df[["x", "y"]].values)
    return cone_df


if __name__ == "__main__":
    gtmd_to_standard_csv_parse()
