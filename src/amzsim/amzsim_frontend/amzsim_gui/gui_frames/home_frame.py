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

# pylint: disable=too-many-instance-attributes;
# pylint: disable=too-many-ancestors;
# In GUI each element (checkbox, button etc.) is instance attribute
# so usually there are more than five of them, as each frames inherits CtkFrame
# there is no way to change the number of ancestors, members are created by builders

# pylint: disable=import-error

import customtkinter
from .frames_builders.home_frame_builder import HomeFrameBuilder
from .frames_functions.home_frame_functions import HomeFrameFunctions


class HomeFrame(customtkinter.CTkFrame):
    """
    Class that inherits from CTkFrame. It is visible on the right side of the app window,
    when it's selected from the menu. It contains simulation info textbox, progress bar
    and launch stop sim button. It enables to launch/stop the simulation.

    Attributes
    ----------
    home_frame_builder : HomeFrameBuilder
        Class that takes home frame as parameter in constructor and builds the frame components.
    home_frame_functions : HomeFrameFunctions
        Class that takes home frame as parameter in constructor and defines the frame components
        functions.
    button_launch_sim : CTkButton
        CTkButton that allows to launch/stop simulation.
    progressbar_sim_works : CTkProgressBar
        CTkProgressBar which moves when simulation is running.
    textbox_config_info : CTkTextbox
        CTkTextbox which displays configuration with which sim was run.
    """

    def __init__(self, parent):
        super().__init__(parent)
        self.button_launch_sim = None
        self.progressbar_sim_works = None
        self.textbox_config_info = None

        self.home_frame_builder = HomeFrameBuilder(home_frame=self)
        self.home_frame_builder.build_home_frame()

        self.home_frame_functions = HomeFrameFunctions(home_frame=self)

        self.bind_buttons()

    def bind_buttons(self):
        """
        Binds frame components to functions defined in home frame functions.
        """
        self.button_launch_sim.configure(
            command=self.home_frame_functions.launch_stop_sim
        )
