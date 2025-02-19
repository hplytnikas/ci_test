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

# pylint: disable-all


import os
import customtkinter
from .gui_frames.home_frame import HomeFrame
from .gui_frames.config_frame import ConfigFrame
from .gui_frames.menu_frame import MenuFrame
from .gui_frames.vehicle_frame import VehicleFrame


class GUI(customtkinter.CTk):
    """
    Class used to represent an main class responsible for how the graphical interface looks,
    it inherits from customtkinter.Ctk class. It can be seen as the main window of the application.
    Inside this window, many "frames" are placed to display different things and group components.
    For more information about customtkinter library, check: github.com/TomSchimansky/CustomTkinter
    and tkinter library.

    Attributes
    ----------
    menu_frame : MenuFrame
        CTkFrame visible on the left side of the window. This frame enables to change frame
        displayed on the right side of the window and change the color theme of the application.
    home_frame : HomeFrame
        CTkFrame which can be selected from menu frame to be visible on the right side of
        the application.vIt has a textfield in which information about current launched
        simulation config is displayed and button which allows to launch and stop simulation.
    config_frame : ConfigFrame
        CTkFrame which can be selected from menu frame to be visible on the right side of
        the application. It has option menus, checkboxes, and textfields which allow to
        launch simulation with different configuration, for example different disciplines,
        tracks, pipeline setups, visualization options.
    """

    def __init__(self):
        # Runs customtkinter.CTk constructor
        super().__init__()

        # The title of application, its window size and path to color theme of components
        self.title("AMZ SIM")
        self.geometry("1540x1000")
        theme_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "themes")
        customtkinter.set_default_color_theme(
            os.path.join(theme_path, "amz_theme.json")
        )

        # Main grid layout of application - set main grid layout 1x2. To understand how grid
        # works in tkinter, check grid geometry in tkinter library.
        self.grid_rowconfigure(0, weight=1)
        self.grid_columnconfigure(1, weight=1)

        # Menu frame setup
        self.menu_frame = MenuFrame(parent=self)

        # Home frame setup
        self.home_frame = HomeFrame(parent=self)

        # Configuration frame setup
        self.config_frame = ConfigFrame(parent=self)

        # self.vehicle_frame = VehicleFrame(parent=self)

        # Assign home and config frame to menu and set starting frame and appearance mode
        self.menu_frame.start(
            frame_home=self.home_frame,
            frame_config=self.config_frame,
            # frame_vehicle=self.vehicle_frame,
        )
