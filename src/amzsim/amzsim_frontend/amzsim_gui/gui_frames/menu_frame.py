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

import customtkinter
from .frames_builders.menu_frame_builder import MenuFrameBuilder
from .frames_functions.menu_frame_functions import MenuFrameFunctions


class MenuFrame(customtkinter.CTkFrame):
    """
    Class that inherits from CTkFrame. It is visible on the left side of the app window.
    This frame enables to change frame displayed on the right side of the window and
    change the color theme of the application.

    Attributes
    ----------
    menu_frame_builder : MenuFrameBuilder
        Class that takes menu frame as parameter in constructor and builds the frame components.
    menu_frame_functions : MenuFrameFunctions
        Class that takes menu frame as parameter in constructor and defines the frame components
        functions.
    img_sim : CTkImage
        Image of amzsim logo which is displayed in label_amzsim_img.
    img_home_light : CTkImage
        Image of home icon which is displayed in light mode.
    img_home_dark : CTkImage
        Image of home icon which is displayed in dark mode.
    img_config_light : CTkImage
        Image of config icon which is displayed in light mode.
    img_config_dark : CTkImage
        Image of config icon which is displayed in dark mode.
    label_amzsim_img : CTkLabel
        Label which displays amzsim logo.
    button_home : CTkButton
        Button which allows to change frame on the right to home frame.
    button_config : CTkButton
        Button which allows to change frame on the right to config frame.
    appearance_mode_menu : CTkOptionMenu
        Option menu that allows to select light or dark mode.
    home_frame : HomeFrame
        HomeFrame to which app will change right side after clicking button_home.
    frame_config : ConfigFrame
        ConfigFrame to which app will change right side after clicking button_config.
    """

    # Parameter parent is GUI in which frame is placed.
    def __init__(self, parent):
        super().__init__(parent)

        # Frame components
        self.img_sim = None
        self.img_home_light = None
        self.img_home_dark = None
        self.img_config_light = None
        self.img_config_dark = None
        self.label_amzsim_img = None
        self.button_home = None
        self.button_config = None
        # self.button_vehicle = None
        self.appearance_mode_menu = None

        self.menu_frame_builder = MenuFrameBuilder(menu_frame=self)
        self.menu_frame_builder.build_menu_frame()

        self.menu_frame_functions = MenuFrameFunctions(menu_frame=self)

        self.bind_buttons()

    def bind_buttons(self):
        """
        Binds frame components to functions defined in menu frame functions.
        """
        self.button_home.configure(command=self.menu_frame_functions.home_button_event)
        self.button_config.configure(
            command=self.menu_frame_functions.configuration_button_event
        )
        # self.button_vehicle.configure(
        #     command=self.menu_frame_functions.vehicle_button_event
        # )
        self.appearance_mode_menu.configure(
            command=self.menu_frame_functions.change_appearance_mode_event
        )

    def start(self, frame_home, frame_config):
        """
        Assigns home and config frame to menu and sets default theme.

        Parameters
        ----------
        frame_home : HomeFrame
            Home frame which will be displayed after clicking home_button
        frame_config : ConfigFrame
            Config frame which will be displayed after clicking config_button
        """
        self.menu_frame_functions.start(frame_home, frame_config)
