#!/usr/bin/env python3

# AMZ Driverless Project
#
# Copyright (c) 2023 Authors:
# - Romir Damle <rdamle@student.ethz.ch>
#
# All rights reserved.
#
# Unauthorized copying of this file, via any medium is strictly prohibited
# Proprietary and confidential

# In GUI each element (checkbox, button etc.) is instance attribute
# so usually there are more than five of them, as each frames inherits CtkFrame
# there is no way to change the number of ancestors, members are created by builders

# pylint: disable=import-error

import customtkinter
from .frames_builders.config_frame_builder import ConfigFrameBuilder
from .frames_functions.config_frame_functions import ConfigFrameFunctions


class VehicleFrame(customtkinter.CTkFrame):
    """
    Class that inherits from CTkFrame. It is visible on the right side of the app window,
    when it's selected from the menu. It contains simulation info textbox, progress bar
    and launch stop sim button. It enables to launch/stop the simulation.

    Attributes
    ----------
    config_frame_builder : ConfigFrameBuilder
        Class that takes config frame as parameter in constructor and builds the frame components.
    config_frame_functions : ConfigFrameFunctions
        Class that takes config frame as parameter in constructor and defines the frame components
        functions.
    """

    def __init__(self, parent):
        super().__init__(parent)

        # self.frame_tracks_disc = None
        # self.label_discipline = None
        # self.om_discipline = None
        # self.label_track_type = None
        # self.om_track_type = None
        # self.label_track = None
        # self.om_track = None
        # self.label_control_node = None
        # self.om_control_node = None
        # self.checkbox_lap_opt = None
        # self.frame_rosbag = None
        # self.label_rosbag = None
        # self.checkbox_rosbag_record = None
        # self.checkbox_rosbag_play = None
        # self.entry_rosbag_path = None
        # self.button_path = None
        # self.frame_pipeline = None
        # self.label_pipeline = None
        # self.checkbox_ve_gt = None
        # self.checkbox_perception_gt = None
        # self.om_perception_config = None
        # self.checkbox_estimation_gt = None
        # self.checkbox_grip_estimation = None
        # self.checkbox_use_sim_time = None
        # self.frame_optionals = None
        # self.checkbox_rviz = None
        # self.checkbox_lapinfo = None
        # self.checkbox_vel = None
        # self.checkbox_track_length = None
        # self.checkbox_cones_col = None
        # self.checkbox_cones_fov = None
        # self.checkbox_ego_frame = None
        # self.checkbox_vehicle_frame = None
        # self.checkbox_online_map = None
        # self.checkbox_bounded_path = None
        # self.checkbox_delaunay = None
        # self.checkbox_control_boundaries = None

        # self.config_frame_builder = ConfigFrameBuilder(config_frame=self)
        # self.config_frame_builder.build_configuration_frame()

        # self.config_frame_functions = ConfigFrameFunctions(config_frame=self)
        # self.config_frame_functions.select_default_options()

        # self.bind_buttons()

    # def bind_buttons(self):
    #     """
    #     Binds frame components to functions defined in config frame functions.
    #     """
    #     self.om_discipline.configure(
    #         command=self.config_frame_functions.adjust_checkboxes_for_discipline
    #     )
    #     self.om_control_node.configure(
    #         command=lambda x: self.config_frame_functions.adjust_available_lapopt(
    #             self.om_discipline.get()
    #         )
    #     )
    #     self.om_track_type.configure(
    #         command=lambda x: self.config_frame_functions.adjust_available_tracks(
    #             self.om_discipline.get()
    #         )
    #     )
    #     self.checkbox_rosbag_record.configure(
    #         command=self.config_frame_functions.record_rosbag_on_off
    #     )
    #     self.button_path.configure(command=self.config_frame_functions.select_rosbag)
    #     self.checkbox_rosbag_play.configure(
    #         command=self.config_frame_functions.play_rosbag_on_off
    #     )
    #     self.checkbox_ve_gt.configure(
    #         command=lambda: self.config_frame_functions.adjust_available_pipeline(
    #             self.om_discipline.get()
    #         )
    #     )
    #     self.checkbox_perception_gt.configure(
    #         command=lambda: self.config_frame_functions.adjust_available_pipeline(
    #             self.om_discipline.get()
    #         )
    #     )
    #     self.om_perception_config.configure(
    #         command=self.config_frame_functions.adjust_available_pipeline(
    #             self.om_discipline.get()
    #         )
    #     )
    #     self.checkbox_estimation_gt.configure(
    #         command=lambda: self.config_frame_functions.adjust_available_pipeline(
    #             self.om_discipline.get()
    #         )
    #     )
    #     self.checkbox_grip_estimation.configure(
    #         command=lambda: self.config_frame_functions.adjust_available_pipeline(
    #             self.om_discipline.get()
    #         )
    #     )
    #     self.checkbox_use_sim_time.configure(
    #         command=lambda: self.config_frame_functions.adjust_available_pipeline(
    #             self.om_discipline.get()
    #         )
    #     )

    # def get_sim_config(self):
    #     """
    #     Get values of GUI components and return dictionary which contains them.

    #     Returns
    #     -------
    #     sim_config_args : dict
    #         Dictionary containing values selected by user in configuration.
    #     """
    #     sim_config_args = {
    #         "discipline": self.om_discipline.get(),
    #         "track path": self.config_frame_functions.get_full_track_path(
    #             discipline=self.om_discipline.get(),
    #             track_type=self.om_track_type.get().lower(),
    #             track_name=self.om_track.get(),
    #         ),
    #         "control node": self.om_control_node.get(),
    #         "lap_opt": self.checkbox_lap_opt.get(),
    #         "rosbag_record": self.checkbox_rosbag_record.get(),
    #         "rosbag_play": self.checkbox_rosbag_play.get(),
    #         "rosbag_path": self.entry_rosbag_path.cget("placeholder_text"),
    #         "ve gt": self.checkbox_ve_gt.get(),
    #         "perception gt": self.checkbox_perception_gt.get(),
    #         "perception config": self.om_perception_config.get(),
    #         "estimation gt": self.checkbox_estimation_gt.get(),
    #         "grip estimation": self.checkbox_grip_estimation.get(),
    #         "use sim time": self.checkbox_use_sim_time.get(),
    #         "rviz": self.checkbox_rviz.get(),
    #         "show lap info": self.checkbox_lapinfo.get(),
    #         "show velocity": self.checkbox_vel.get(),
    #         "show track length": self.checkbox_track_length.get(),
    #         "show collision detect": self.checkbox_cones_col.get(),
    #         "show fov cones": self.checkbox_cones_fov.get(),
    #         "show ego frame": self.checkbox_ego_frame.get(),
    #         "show veh frame": self.checkbox_vehicle_frame.get(),
    #         "show online map": self.checkbox_online_map.get(),
    #         "show bounded path": self.checkbox_bounded_path.get(),
    #         "show delaunay": self.checkbox_delaunay.get(),
    #         "show control bounds": self.checkbox_control_boundaries.get(),
    #     }
    #     return sim_config_args
