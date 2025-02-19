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
#
# pylint: disable=import-error;

from amzsim_frontend.amzsim_launcher.launcher import Launcher
from amzsim_frontend.amzsim_gui.gui import GUI


class Presenter:
    """
    Presenter part of MVP design pattern. It is responsible for connecting GUI with Launcher and
    interaction between frontend and backend of the application.
    It's start_app() method launches the application.

    Attributes
    ----------
    launcher : Launcher
        Class that allows to run "roslaunch amzsim_interface" with different arguments
        depending on selected config in GUI.
    gui : GUI
        Graphical interface of the application. Responsible only for how application looks and
        interaction between different GUI components (checkboxes, optionmenu etc.).
    """

    def __init__(self):
        self.launcher = Launcher()
        self.gui = GUI()
        self.bind_buttons()

    def start_app(self):
        """
        Turns on the application.
        """

        self.gui.mainloop()

    def launch_stop_simulator(self):
        """
        Loads values of GUI components and passes it to launcher "sim_config".
        These will be used as arguments for "roslaunch amzsim_interface" command.
        """

        if not self.launcher.get_sim_launched():  # Launch simulator if not launched
            sim_config = self.gui.config_frame.get_sim_config()
            self.launcher.launch_simulator(sim_config)
        else:
            self.launcher.stop_simulation()

    def bind_buttons(self):
        """
        Binds launch_stop_simulator method to launch stop button inside the GUI.
        """

        self.gui.home_frame.button_launch_sim.bind(
            "<Button-1>", lambda event: self.launch_stop_simulator()
        )
