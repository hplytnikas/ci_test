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

# This script generates the boundary for the skidpad track

import os
import math
import matplotlib.pyplot as plt
from rosbag import Bag

# pylint: disable= import-error
from autonomous_msgs.msg import Boundary
from autonomous_msgs.msg import PointWithConfidence
import rospkg
import numpy as np
from matplotlib.widgets import Slider

# Variable Parameters
RIGHT_CIRCLES = 2
LEFT_CIRCLES = 2
RUNOFF_CONES = 5
RUNOFF_PITCH = 3
RUNUP_CONES = 1
RUNUP_PITCH = 4.0
BS_PITCH = 4.0

# Fixed/derived Parameters
TRACK_START = -15.0
TRACK_END = 13.0
TRACK_LENGTH = TRACK_END - TRACK_START
RUNUP_LENGTH = RUNUP_PITCH * (RUNUP_CONES - 1)
RUNOFF_LENGTH = RUNOFF_PITCH * (RUNOFF_CONES - 1)
BS_LENGTH = RUNOFF_LENGTH + RUNUP_LENGTH + TRACK_LENGTH
BS_CONES = math.floor(BS_LENGTH / BS_PITCH) + 1

# Official Track Parts, those are (x,y) coordinates of the cones
right_boundary_entry = [
    (-18.0, -1.65),
    (-7.619075567285051, -1.5059244327149521),
    (-2.8605586569290504, -2.2190004944781334),
    (0, -1.5),
]
left_boundary_entry = [
    (-18.0, 1.65),
    (-7.619075567285051, 1.5059244327149521),
    (-2.8605586569290504, 2.2190004944781334),
    (0, 1.5),
]
right_boundary_right_circle = [
    (2.860558656929046, -2.2190004944781316),
    (5.285623189369442, -3.839376810630557),
    (6.905999505521868, -6.264441343070954),
    (7.475, -9.125),
    (6.905999505521868, -11.985558656929046),
    (5.285623189369443, -14.410623189369442),
    (2.860558656929047, -16.03099950552187),
    (9.154234823626465e-16, -16.6),
    (-2.860558656929045, -16.03099950552187),
    (-5.285623189369442, -14.410623189369444),
    (-6.905999505521867, -11.98555865692905),
    (-7.475, -9.125000000000002),
    (-6.9059995055218675, -6.264441343070953),
    (-5.285623189369444, -3.839376810630559),
    (-2.8605586569290504, -2.2190004944781334),
    (0, -1.5),
]
right_boundary_left_circle = [
    (2.860558656929046, -2.2190004944781316),
    (7.619075567285051, 1.5059244327149521),
    (9.954801962809114, 5.001586016266155),
    (10.775, 9.125000000000002),
    (9.954801962809112, 13.248413983733847),
    (7.619075567285049, 16.74407556728505),
    (4.123413983733841, 19.079801962809114),
    (-1.3195569260812732e-15, 19.9),
    (-4.123413983733844, 19.079801962809114),
    (-7.6190755672850505, 16.744075567285048),
    (-9.954801962809116, 13.248413983733842),
    (-10.775, 9.125),
    (-9.954801962809116, 5.001586016266157),
    (-7.619075567285049, 1.5059244327149495),
    (-2.8605586569290504, -2.2190004944781334),
    (0, -1.5),
]
left_boundary_right_circle = [
    (2.8605586569290504, 2.2190004944781334),
    (7.619075567285049, -1.5059244327149495),
    (9.954801962809116, -5.001586016266157),
    (10.775, -9.125),
    (9.954801962809116, -13.248413983733842),
    (7.6190755672850505, -16.744075567285048),
    (4.123413983733844, -19.079801962809114),
    (1.3195569260812732e-15, -19.9),
    (-4.123413983733841, -19.079801962809114),
    (-7.619075567285049, -16.74407556728505),
    (-9.954801962809112, -13.248413983733847),
    (-10.775, -9.125000000000002),
    (-9.954801962809114, -5.001586016266155),
    (-7.619075567285051, -1.5059244327149521),
    (-2.860558656929046, 2.2190004944781316),
    (0, 1.5),
]
left_boundary_left_circle = [
    (2.8605586569290504, 2.2190004944781334),
    (5.285623189369444, 3.839376810630559),
    (6.9059995055218675, 6.264441343070953),
    (7.475, 9.125000000000002),
    (6.905999505521867, 11.98555865692905),
    (5.285623189369442, 14.410623189369444),
    (2.860558656929045, 16.03099950552187),
    (-9.154234823626465e-16, 16.6),
    (-2.860558656929047, 16.03099950552187),
    (-5.285623189369443, 14.410623189369442),
    (-6.905999505521868, 11.985558656929046),
    (-7.475, 9.125),
    (-6.905999505521868, 6.264441343070954),
    (-5.285623189369442, 3.839376810630557),
    (-2.860558656929046, 2.2190004944781316),
    (0, 1.5),
]
right_boundary_exit = [(2.860558656929046, -2.2190004944781316)]
left_boundary_exit = [(2.860558656929046, 2.2190004944781316)]
# Runoff Part
r_boundary_runoff = []
l_boundary_runoff = []
for i in range(RUNOFF_CONES):
    r_boundary_runoff.append((TRACK_END + i * RUNOFF_PITCH, -1.65))
    l_boundary_runoff.append((TRACK_END + i * RUNOFF_PITCH, 1.65))
# Backstraight Part
r_bs_straight = []
l_bs_straight = []
for i in range(BS_CONES):
    l_bs_straight.append((TRACK_END + RUNOFF_LENGTH - i * BS_PITCH, 30))
    r_bs_straight.append((TRACK_END + RUNOFF_LENGTH - i * BS_PITCH, 33.5))
# Runup Part
r_boundary_runup = []
l_boundary_runup = []
for i in range(RUNUP_CONES):
    r_boundary_runup.append((TRACK_START - RUNUP_LENGTH + i * RUNUP_PITCH, -1.65))
    l_boundary_runup.append((TRACK_START - RUNUP_LENGTH + i * RUNUP_PITCH, 1.65))
# Backstraight Curve Parts
l_curve = [
    (5, 3),
    (8, 6),
    (11, 9),
    (13, 12),
    (14, 15),
    (13, 18),
    (11, 21.2),
    (8, 24.5),
    (5, 27),
]
r_curve = [
    (7.5, 0),
    (11, 3),
    (14, 7),
    (16, 10),
    (17.5, 15),
    (16, 20),
    (14, 23.2),
    (11, 27.5),
    (7.5, 30),
]
l_bs_curve_r = [(TRACK_END + RUNOFF_LENGTH + ele[0], ele[1]) for ele in l_curve]
r_bs_curve_r = [(TRACK_END + RUNOFF_LENGTH + ele[0], ele[1]) for ele in r_curve]
l_bs_curve_l = [(TRACK_START - RUNUP_LENGTH - ele[0], ele[1]) for ele in l_curve[::-1]]
r_bs_curve_l = [(TRACK_START - RUNUP_LENGTH - ele[0], ele[1]) for ele in r_curve[::-1]]

# Combine Parts
right_boundary = (
    right_boundary_entry
    + right_boundary_right_circle * RIGHT_CIRCLES
    + right_boundary_left_circle * LEFT_CIRCLES
    + right_boundary_exit
    + r_boundary_runoff
    + r_bs_curve_r
    + r_bs_straight
    + r_bs_curve_l
    + r_boundary_runup
)
left_boundary = (
    left_boundary_entry
    + left_boundary_right_circle * RIGHT_CIRCLES
    + left_boundary_left_circle * LEFT_CIRCLES
    + left_boundary_exit
    + l_boundary_runoff
    + l_bs_curve_r
    + l_bs_straight
    + l_bs_curve_l
    + l_boundary_runup
)

# Middle Line
middle_line = []
for i, boundary_val in enumerate(right_boundary):
    middle_line_point = (
        (boundary_val[0] + left_boundary[i][0]) / 2,
        (boundary_val[1] + left_boundary[i][1]) / 2,
    )
    middle_line.append(middle_line_point)

rx = np.array([pos[0] for pos in right_boundary[:]])
ry = np.array([pos[1] for pos in right_boundary[:]])
lx = np.array([pos[0] for pos in left_boundary[:]])
ly = np.array([pos[1] for pos in left_boundary[:]])
mx = np.array([pos[0] for pos in middle_line[:]])
my = np.array([pos[1] for pos in middle_line[:]])
fig, ax = plt.subplots()
plt.subplots_adjust(left=0.25, bottom=0.25)
ax.set_xlim((TRACK_START - RUNUP_LENGTH - 20, TRACK_END + RUNOFF_LENGTH + 20))
plt.scatter(rx, ry)
plt.scatter(lx, ly)
plt.scatter(mx, my)
(line1,) = plt.plot(rx, ry)
(line2,) = plt.plot(lx, ly)
(line3,) = plt.plot(mx, my)
end_ax = plt.axes([0.1, 0.25, 0.0225, 0.63])
end_slider = Slider(
    ax=end_ax, label="end", valmin=1, valmax=len(middle_line), orientation="vertical"
)


def end_update():
    line1.set_xdata(rx[: math.floor(end_slider.val)])
    line1.set_ydata(ry[: math.floor(end_slider.val)])
    line2.set_xdata(lx[: math.floor(end_slider.val)])
    line2.set_ydata(ly[: math.floor(end_slider.val)])
    line3.set_xdata(mx[: math.floor(end_slider.val)])
    line3.set_ydata(my[: math.floor(end_slider.val)])
    fig.canvas.draw_idle()


end_slider.on_changed(end_update)
plt.grid()
plt.show()


# Generates boundary for skidpad map
def generate_skidpad_map():
    boundary = Boundary()
    for cone in right_boundary:
        point = PointWithConfidence()
        point.position.x = cone[0] + 16.69
        point.position.y = cone[1]
        point.confidence = 1.0
        boundary.right_boundary.append(point)
    for cone in left_boundary:
        point = PointWithConfidence()
        point.position.x = cone[0] + 16.69
        point.position.y = cone[1]
        point.confidence = 1.0
        boundary.left_boundary.append(point)
    for cone in middle_line:
        point = PointWithConfidence()
        point.position.x = cone[0] + 16.69
        point.position.y = cone[1]
        point.confidence = 1.0
        boundary.middle_line.append(point)
    boundary.header.frame_id = "/world"
    return boundary


if __name__ == "__main__":
    # First, find the path to the maps folder.
    rospack = rospkg.RosPack()
    package_path = rospack.get_path("amzsim_tracks")
    output_path = os.path.join(package_path, "maps", "skidpad_laptimeopt.bag")
    bag = Bag(output_path, "w")
    try:
        boundary_map = generate_skidpad_map()
        bag.write("/estimation/global_map_ground_truth", boundary_map)
    finally:
        bag.close()
