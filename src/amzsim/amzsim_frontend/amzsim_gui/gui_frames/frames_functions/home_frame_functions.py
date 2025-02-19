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


class HomeFrameFunctions:
    """
    Class that is used to create and define functions for HomeFrame components.

    Attributes
    ----------
    home_frame : HomeFrame
        Class HomeFrame for which components functions will be defined.
    """

    def __init__(self, home_frame):
        self.home_frame = home_frame

    def launch_stop_sim(self):
        """
        Changes text on launch stop sim button and adjusts progress bar.
        """
        if self.home_frame.button_launch_sim.cget("text") == "Launch simulation":
            self.home_frame.button_launch_sim.configure(text="Stop simulation")
            self.home_frame.progressbar_sim_works.start()
        else:
            self.home_frame.button_launch_sim.configure(text="Launch simulation")
            self.home_frame.progressbar_sim_works.stop()
            self.home_frame.progressbar_sim_works.set(0)

    def set_config_info_textbox(self):
        """
        Insert text to info textbox
        """
        self.home_frame.textbox_config_info.configure(state="normal")
        self.home_frame.textbox_config_info.insert("0.0")
        self.home_frame.textbox_config_info.configure(state="disabled")
