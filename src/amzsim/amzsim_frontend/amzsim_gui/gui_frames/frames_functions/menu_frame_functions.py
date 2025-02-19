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


class MenuFrameFunctions:
    """
    Class that is used to create and define functions for MenuFrame components.

    Attributes
    ----------
    menu_frame : MenuFrame
        Class MenuFrame for which components functions will be defined.
    """

    def __init__(self, menu_frame):
        self.menu_frame = menu_frame

    def home_button_event(self):
        """
        Calls select_frame_by_name with argument name="home".
        Frame on the right of app window will change to HomeFrame.
        """
        self.select_frame_by_name("home")

    def configuration_button_event(self):
        """
        Calls select_frame_by_name with argument name="configuration".
        Frame on the right of app window will change to ConfigFrame.
        """
        self.select_frame_by_name("configuration")

    # def vehicle_button_event(self):
    #     """
    #     Calls select_frame_by_name with argument name="vehicle".
    #     Frame on the right of app window will change to VehicleFrame.
    #     """
    #     self.select_frame_by_name("vehicle")

    def select_frame_by_name(self, name):
        """
        Changes frame on the right side of app window to the one with given name.

        Parameters
        ----------
        name : str
            Name of the frame to which switch the right side of the app window.
            Available options: "home", "configuration", "vehicle"
        """
        self.menu_frame.button_home.configure(
            fg_color=("gray75", "gray25") if name == "home" else "transparent"
        )
        self.menu_frame.button_config.configure(
            fg_color=("gray75", "gray25") if name == "configuration" else "transparent"
        )
        # self.menu_frame.button_vehicle.configure(
        #     fg_color=("gray75", "gray25") if name == "vehicle" else "transparent"
        # )

        if name == "home":
            self.menu_frame.home_frame.grid(row=0, column=1, sticky="nsew")
        else:
            self.menu_frame.home_frame.grid_forget()
        if name == "configuration":
            self.menu_frame.config_frame.grid(row=0, column=1, sticky="nsew")
        else:
            self.menu_frame.config_frame.grid_forget()
        # if name == "vehicle":
        #     self.menu_frame.vehicle_frame.grid(row=0, column=1, sticky="nsew")
        # else:
        #     self.menu_frame.vehicle_frame.grid_forget()

    def change_appearance_mode_event(self, new_appearance_mode):
        """
        Changes app theme to dark or light.

        Parameters
        ----------
        new_appearance_mode : str
            Name of theme to change the app. Available options: "Dark", "Light".
        """

        if new_appearance_mode == "Dark":
            self.menu_frame.button_home.configure(image=self.menu_frame.img_home_dark)
            self.menu_frame.button_config.configure(
                image=self.menu_frame.img_config_dark
            )
            # self.menu_frame.button_vehicle.configure(
            #     image=self.menu_frame.img_home_dark
            # )
        if new_appearance_mode == "Light":
            self.menu_frame.button_home.configure(image=self.menu_frame.img_home_light)
            self.menu_frame.button_config.configure(
                image=self.menu_frame.img_config_light
            )
            # self.menu_frame.button_vehicle.configure(
            #     image=self.menu_frame.img_home_light
            # )
        customtkinter.set_appearance_mode(new_appearance_mode)

    def start(self, frame_home, frame_config):
        """
        Assigns home and config frame to menu and sets default theme.

        Parameters
        ----------
        home_frame : HomeFrame
            CTkFrame assigned as "home" frame.
        frame_config : ConfigFrame
            CTkFrame assigned as "configuration" frame.
        """

        self.menu_frame.home_frame = frame_home
        self.menu_frame.config_frame = frame_config
        # self.menu_frame.vehicle_frame = frame_vehicle
        self.select_frame_by_name("home")
        self.change_appearance_mode_event("Dark")
