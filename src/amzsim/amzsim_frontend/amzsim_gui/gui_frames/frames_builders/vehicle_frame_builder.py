# AMZ Driverless Project
#
# Copyright (c) 2023 Authors:
# - Romir Damle <rdamle@student.ethz.ch>
#
# All rights reserved.
#
# Unauthorized copying of this file, via any medium is strictly prohibited
# Proprietary and confidential

import customtkinter


class VehicleFrameBuilder:
    """
    Class that is used to create and instantiate Vehilce components, such as checkboxes, optionmenu
    internal frames etc. and puts them in correct places.

    Attributes
    ----------
    vehilce_frame : VehicleFrame
        Class VehicleFrame for which components will be created.
    """

    def __init__(self, vehicle_frame):
        self.vehicle_frame = vehicle_frame

    def build_vehicle_frame(self):
        """
        Builds all of the frame components.
        """

        self.__configure_vehicle_frame()
        self.__setup_vehicle_frame()

    def __configure_vehicle_frame(self):
        """
        Configures vehicle frame object and its grid.
        """

        self.vehicle_frame.configure(corner_radius=0, fg_color="transparent")
        self.vehicle_frame.grid_columnconfigure(0, weight=1)
        self.vehicle_frame.grid_rowconfigure(0, weight=1)

    def __setup_vehicle_frame(self):
        """
        Setups vehicle frame components.
        """

        # Create vehicle frame info textbox.
        self.vehicle_frame.textbox_config_info = customtkinter.CTkTextbox(
            self.vehicle_frame, width=250, state="disabled"
        )
        self.vehicle_frame.textbox_config_info.grid(
            row=0, column=0, padx=20, pady=20, sticky="nsew"
        )
