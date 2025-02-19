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

from termcolor import colored

ROW_WIDTH = 37
FULL_ROW_WIDTH = 39


def color_threshold_ge(item, red, yellow):
    if item > red:
        return "red"
    if yellow and item > yellow:
        return "yellow"
    return "green"


def color_threshold_le(item, red, yellow):
    if item < red:
        return "red"
    if yellow and item < yellow:
        return "yellow"
    return "green"


def color_threshold(item, red, yellow=None, direction=""):
    # dir --> direction: greater or smaller
    if direction == "ge":
        return color_threshold_ge(item, red, yellow)
    if direction == "le":
        return color_threshold_le(item, red, yellow)
    if isinstance(red, list) and isinstance(yellow, list):
        if item < yellow[0]:
            return color_threshold(item, red[0], yellow[0], "le")
        return color_threshold(item, red[1], yellow[1], "ge")
    return ""


def bool_color(msg):
    if msg:
        return "green"
    return "red"


def bool_color_gray(msg):
    if msg:
        return "green"
    return ""


class DisplayLine:
    """
    Returns 40 character line
    """

    def __init__(self, left_side, right_side, color=None):
        self.left_side = left_side
        self.right_side = right_side
        self.color = color

    def render(self):
        left = self.left_side.ljust(ROW_WIDTH - len(self.right_side), " ")
        right = self.right_side
        assert len(left) + len(right) == ROW_WIDTH
        if self.color:
            return colored(left + right, self.color)
        return left + right


class EmptyCol:

    def render(self):
        return " " * ROW_WIDTH


class DisplayTitle:

    def __init__(self, title, color=None):
        assert len(title) <= ROW_WIDTH
        self.title = " " + title + " "
        self.color = color

    def render(self):
        if self.color:
            return colored(self.title.center(ROW_WIDTH, "="), self.color)
        return self.title.center(ROW_WIDTH, "=")


class DisplaySet:

    def __init__(self, columns):
        assert len(columns) <= 4
        self.columns = columns
        self.num_rows = max(len(c) for c in columns)
        self.num_cols = len(columns)
        self.divider = "||"
        self.last_line = ((" " * ROW_WIDTH + self.divider) * self.num_cols) + "\n"
        self.empty_row = " " * ROW_WIDTH + self.divider

    def render(self):
        output = ""
        for i in range(self.num_rows):
            row = ""
            for col in self.columns:
                if i < len(col):
                    row += col[i].render() + self.divider
                else:
                    row += self.empty_row
            output += row + "\n"
        return output
