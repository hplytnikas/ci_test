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

# pylint: disable= import-error
import customtkinter


class HomeFrameBuilder:
    """
    Class that is used to create and instantiate HomeFrame components, such as checkboxes,
    optionmenu etc. and puts them in correct places.

    Attributes
    ----------
    home_frame : HomeFrame
        Class HomeFrame for which components will be created.
    """

    def __init__(self, home_frame):
        self.home_frame = home_frame

    def build_home_frame(self):
        """
        Builds all of the frame components.
        """

        self.__configure_home_frame()
        self.__setup_home_frame()

    def __configure_home_frame(self):
        """
        Configures home frame object and its grid.
        """

        self.home_frame.configure(corner_radius=0, fg_color="transparent")
        self.home_frame.grid_columnconfigure(0, weight=1)
        self.home_frame.grid_rowconfigure(0, weight=1)

    def __setup_home_frame(self):
        """
        Setups home frame components.
        """

        # Create home frame info textbox.
        self.home_frame.textbox_config_info = customtkinter.CTkTextbox(
            self.home_frame, width=250, state="disabled"
        )
        self.home_frame.textbox_config_info.grid(
            row=0, column=0, padx=20, pady=20, sticky="nsew"
        )

        # Create home frame sim progress bar.
        self.home_frame.progressbar_sim_works = customtkinter.CTkProgressBar(
            self.home_frame, determinate_speed=3
        )
        self.home_frame.progressbar_sim_works.grid(
            row=1, column=0, padx=20, pady=(0, 30), sticky="nsew"
        )
        self.home_frame.progressbar_sim_works.set(0)

        # Create home frame button launch stop.
        self.home_frame.button_launch_sim = customtkinter.CTkButton(
            self.home_frame,
            text="Launch simulation",
            width=280,
            height=56,
            font=customtkinter.CTkFont(size=20, weight="bold"),
        )
        self.home_frame.button_launch_sim.grid(row=2, column=0, padx=20, pady=(0, 30))
