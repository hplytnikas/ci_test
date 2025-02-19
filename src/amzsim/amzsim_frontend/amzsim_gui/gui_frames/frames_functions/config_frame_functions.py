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
# pylint: disable= missing-module-docstring
# pylint: disable= missing-function-docstring

from tkinter import filedialog as fd
import os
import ament_index_python.packages


class ConfigFrameFunctions:
    """
    Class that is used to create and define functions for ConfigFrame components.

    Attributes
    ----------
    config_frame : ConfigFrame
        Class ConfigFrame for which components functions will be defined.
    """

    def __init__(self, config_frame):
        self.config_frame = config_frame

    def select_default_options(self):
        """
        Selects initial default options after running the GUI.
        """
        self.config_frame.om_discipline.set("Autocross")
        self.config_frame.checkbox_ve_gt.select()
        self.config_frame.checkbox_perception_gt.select()
        self.config_frame.checkbox_estimation_gt.select()
        self.adjust_checkboxes_for_discipline("Autocross")

    def adjust_checkboxes_for_discipline(self, discipline):
        """
        Turns on/off checkboxes and displays different tracks/nodes depending on discipline.

        Parameters
        ----------
        discipline : str
            Discipline chosen.
        """
        self.adjust_available_tracks(discipline=discipline)
        self.adjust_available_control_node(discipline=discipline)
        # self.adjust_available_lapopt(discipline=discipline)
        self.adjust_available_pipeline(discipline=discipline)
        # self.adjust_available_optionals(discipline=discipline)

    def adjust_available_tracks(self, discipline):
        """
        Adjust visible tracks based on discipline.

        Parameters
        ----------
        discipline : str
            Discipline chosen.
        """
        tracks_available = []
        track_type = self.config_frame.om_track_type.get()
        tracks_path = self.__get_track_path(
            discipline=discipline, track_type=track_type
        )
        tracks_available = self.__extract_track_names_from_directory(
            tracks_directory=tracks_path
        )
        self.config_frame.om_track.configure(values=tracks_available)
        if len(tracks_available) > 0:
            self.config_frame.om_track.set(tracks_available[0])
        else:
            self.config_frame.om_track.set("")

    def get_full_track_path(self, discipline, track_type, track_name):
        track_path = self.__get_track_path(discipline=discipline, track_type=track_type)
        track_path = track_path + "/" + track_name + ".csv"
        return track_path

    def __get_track_path(self, discipline, track_type):
        package_path = ament_index_python.packages.get_package_share_directory(
            "amzsim_tracks"
        )
        tracks_base_path = package_path + "/tracks_csv"
        # tracks_base_path = rclpy.RosPack().get_path('amzsim_tracks') + "/tracks_csv"
        track_type_lower = track_type.lower()
        tracks_path = tracks_base_path + "/" + track_type_lower
        if track_type_lower == "custom":
            return tracks_path
        if discipline in ("Autocross", "Trackdrive"):
            tracks_path = tracks_path + "/autocross_trackdrive"
        if discipline == "Skidpad":
            tracks_path = tracks_path + "/skidpad"
        if discipline == "Acceleration":
            tracks_path = tracks_path + "/acceleration"
        return tracks_path

    def __extract_track_names_from_directory(self, tracks_directory):
        track_names = []
        track_files_in_dir = os.listdir(tracks_directory)
        for track_file in track_files_in_dir:
            track_name = os.path.splitext(track_file)[0]
            track_names.append(track_name)
        track_names.sort()
        return track_names

    def adjust_available_control_node(self, discipline):
        """
        Adjust visible node used for controls based on discipline.

        Parameters
        ----------
        discipline : str
            Discipline chosen.
        """
        node_available = []
        if discipline == "Autocross":
            node_available = ["PurePursuit", "MPC", "None"]
        if discipline == "Trackdrive":
            node_available = ["PurePursuit", "MPC", "None"]
        if discipline == "Skidpad":
            node_available = ["PurePursuit", "MPC", "None"]
        if discipline == "Acceleration":
            node_available = ["PurePursuit", "None"]
        self.config_frame.om_control_node.configure(values=node_available)
        self.config_frame.om_control_node.set(node_available[0])

    # def adjust_available_lapopt(self, discipline):
    #     """
    #     Turn lapopt checkbox on/off depending on discipline.

    #     Parameters
    #     ----------
    #     discipline : str
    #         Discipline chosen.
    #     """
    #     if (
    #         discipline in ("Trackdrive", "Skidpad")
    #         and self.config_frame.om_control_node.get() != "Pure pursuit"
    #     ):
    #         self.config_frame.checkbox_lap_opt.configure(state="normal")
    #     else:
    #         self.config_frame.checkbox_lap_opt.deselect()
    #         self.config_frame.checkbox_lap_opt.configure(state="disabled")

    def adjust_available_pipeline(self, discipline):
        """
        Adjust enabled/disabled and selected checkboxes based on
        discipline and selected pipeline checkboxes.

        Parameters
        ----------
        discipline : str
            Discipline to change the pipeline.
        """
        perception_gt = self.config_frame.checkbox_perception_gt.get()
        self.__enable_disable_pipeline_checkboxes(state="normal")

        # Enable grip estimation checkbox once it works
        # self.config_frame.checkbox_grip_estimation.configure(state="disabled")

        if not perception_gt:
            self.config_frame.checkbox_estimation_gt.deselect()
            self.config_frame.checkbox_estimation_gt.configure(state="disabled")
        # if discipline in (["Acceleration"]):
        #     self.config_frame.checkbox_estimation_gt.deselect()
        #     self.config_frame.checkbox_estimation_gt.configure(state="disabled")

        if perception_gt:
            self.config_frame.om_perception_config.set("Fusion")
            self.config_frame.om_perception_config.configure(state="disabled")

    def __enable_disable_pipeline_checkboxes(self, state):
        """
        Enable disable checkboxes.

        Parameters
        ----------
        state : str
            State to change, available "disabled", "normal".
        """
        self.config_frame.checkbox_estimation_gt.configure(state=state)
        # self.config_frame.checkbox_grip_estimation.configure(state=state)
        self.config_frame.checkbox_perception_gt.configure(state=state)
        self.config_frame.om_perception_config.configure(state=state)
        self.config_frame.checkbox_ve_gt.configure(state=state)

    def adjust_available_optionals(self, discipline):
        """
        Adjust available optional checkboxes based on discipline.

        Parameters
        ----------
        discipline : str
            Discipline chosen by user.
        """
        if discipline == "Acceleration":
            self.adjust_acceleration_optionals()
        if discipline == "Autocross":
            self.adjust_autocross_optionals()
        if discipline == "Trackdrive":
            self.adjust_trackdrive_optionals()
        if discipline == "Skidpad":
            self.adjust_skidpad_optionals()

    def adjust_acceleration_optionals(self):
        self.config_frame.checkbox_lapinfo.configure(state="normal")
        self.config_frame.checkbox_vel.configure(state="normal")
        self.config_frame.checkbox_track_length.configure(state="disabled")
        self.config_frame.checkbox_cones_col.configure(state="disabled")
        self.config_frame.checkbox_cones_fov.configure(state="normal")
        self.config_frame.checkbox_ego_frame.configure(state="normal")
        self.config_frame.checkbox_vehicle_frame.configure(state="normal")
        self.config_frame.checkbox_online_map.configure(state="normal")
        self.config_frame.checkbox_bounded_path.configure(state="normal")
        self.config_frame.checkbox_delaunay.configure(state="disabled")
        self.config_frame.checkbox_control_boundaries.configure(state="disabled")
        self.config_frame.checkbox_lapinfo.select()
        self.config_frame.checkbox_vel.select()
        self.config_frame.checkbox_track_length.deselect()
        self.config_frame.checkbox_cones_col.deselect()
        self.config_frame.checkbox_cones_fov.select()
        self.config_frame.checkbox_ego_frame.select()
        self.config_frame.checkbox_vehicle_frame.select()
        self.config_frame.checkbox_online_map.select()
        self.config_frame.checkbox_bounded_path.select()
        self.config_frame.checkbox_delaunay.deselect()
        self.config_frame.checkbox_control_boundaries.deselect()

    def adjust_autocross_optionals(self):
        self.config_frame.checkbox_lapinfo.configure(state="normal")
        self.config_frame.checkbox_vel.configure(state="normal")
        self.config_frame.checkbox_track_length.configure(state="disabled")
        self.config_frame.checkbox_cones_col.configure(state="disabled")
        self.config_frame.checkbox_cones_fov.configure(state="normal")
        self.config_frame.checkbox_ego_frame.configure(state="normal")
        self.config_frame.checkbox_vehicle_frame.configure(state="normal")
        self.config_frame.checkbox_online_map.configure(state="normal")
        self.config_frame.checkbox_bounded_path.configure(state="normal")
        self.config_frame.checkbox_delaunay.configure(state="normal")
        self.config_frame.checkbox_control_boundaries.configure(state="normal")
        self.config_frame.checkbox_lapinfo.select()
        self.config_frame.checkbox_vel.select()
        self.config_frame.checkbox_track_length.deselect()
        self.config_frame.checkbox_cones_col.deselect()
        self.config_frame.checkbox_cones_fov.select()
        self.config_frame.checkbox_ego_frame.select()
        self.config_frame.checkbox_vehicle_frame.select()
        self.config_frame.checkbox_online_map.select()
        self.config_frame.checkbox_bounded_path.deselect()
        self.config_frame.checkbox_delaunay.deselect()
        self.config_frame.checkbox_control_boundaries.select()

    def adjust_trackdrive_optionals(self):
        self.config_frame.checkbox_lapinfo.configure(state="normal")
        self.config_frame.checkbox_vel.configure(state="normal")
        self.config_frame.checkbox_track_length.configure(state="disabled")
        self.config_frame.checkbox_cones_col.configure(state="disabled")
        self.config_frame.checkbox_cones_fov.configure(state="normal")
        self.config_frame.checkbox_ego_frame.configure(state="normal")
        self.config_frame.checkbox_vehicle_frame.configure(state="normal")
        self.config_frame.checkbox_online_map.configure(state="disabled")
        self.config_frame.checkbox_bounded_path.configure(state="disabled")
        self.config_frame.checkbox_delaunay.configure(state="disabled")
        self.config_frame.checkbox_control_boundaries.configure(state="normal")
        self.config_frame.checkbox_lapinfo.select()
        self.config_frame.checkbox_vel.select()
        self.config_frame.checkbox_track_length.deselect()
        self.config_frame.checkbox_cones_col.deselect()
        self.config_frame.checkbox_cones_fov.select()
        self.config_frame.checkbox_ego_frame.select()
        self.config_frame.checkbox_vehicle_frame.select()
        self.config_frame.checkbox_online_map.deselect()
        self.config_frame.checkbox_bounded_path.deselect()
        self.config_frame.checkbox_delaunay.deselect()
        self.config_frame.checkbox_control_boundaries.select()
        if self.config_frame.checkbox_estimation_gt.get():
            self.config_frame.checkbox_cones_fov.configure(state="disabled")
            self.config_frame.checkbox_cones_fov.deselect()

    def adjust_skidpad_optionals(self):
        self.config_frame.checkbox_lapinfo.configure(state="normal")
        self.config_frame.checkbox_vel.configure(state="normal")
        self.config_frame.checkbox_track_length.configure(state="disabled")
        self.config_frame.checkbox_cones_col.configure(state="disabled")
        self.config_frame.checkbox_cones_fov.configure(state="normal")
        self.config_frame.checkbox_ego_frame.configure(state="normal")
        self.config_frame.checkbox_vehicle_frame.configure(state="normal")
        self.config_frame.checkbox_online_map.configure(state="disabled")
        self.config_frame.checkbox_bounded_path.configure(state="disabled")
        self.config_frame.checkbox_delaunay.configure(state="disabled")
        self.config_frame.checkbox_control_boundaries.configure(state="normal")
        self.config_frame.checkbox_lapinfo.select()
        self.config_frame.checkbox_vel.select()
        self.config_frame.checkbox_track_length.deselect()
        self.config_frame.checkbox_cones_col.deselect()
        self.config_frame.checkbox_cones_fov.select()
        self.config_frame.checkbox_ego_frame.select()
        self.config_frame.checkbox_vehicle_frame.select()
        self.config_frame.checkbox_online_map.deselect()
        self.config_frame.checkbox_bounded_path.deselect()
        self.config_frame.checkbox_delaunay.deselect()
        self.config_frame.checkbox_control_boundaries.select()
        if self.config_frame.checkbox_estimation_gt.get():
            self.config_frame.checkbox_cones_fov.configure(state="disabled")
            self.config_frame.checkbox_cones_fov.deselect()

    def record_rosbag_on_off(self):
        """
        Adjust available checkboxes for recording rosbag.
        """
        if self.config_frame.checkbox_rosbag_record.get():
            self.config_frame.checkbox_rosbag_play.configure(state="disabled")
            self.config_frame.checkbox_rosbag_play.deselect()
            self.config_frame.button_path.configure(state="normal")
        else:
            self.config_frame.checkbox_rosbag_play.configure(state="normal")
            self.config_frame.button_path.configure(state="disabled")
            self.input_to_path_entry("")

    def play_rosbag_on_off(self):
        """
        Adjust available checkboxes for playing from rosbag.
        """
        if self.config_frame.checkbox_rosbag_play.get():
            self.__enable_disable_for_play(state="disabled")
            self.__deselect_for_play()
            self.config_frame.button_path.configure(state="normal")
            self.__enable_and_select_optionals()
        else:
            self.__enable_disable_for_play(state="normal")
            self.config_frame.button_path.configure(state="disabled")
            self.input_to_path_entry("")
            self.adjust_available_optionals(
                discipline=self.config_frame.om_discipline.get()
            )

    def __enable_disable_for_play(self, state):
        self.config_frame.om_discipline.configure(state=state)
        self.config_frame.om_track_type.configure(state=state)
        self.config_frame.om_track.configure(state=state)
        self.config_frame.om_control_node.configure(state=state)
        self.config_frame.om_control_node.configure(state=state)
        # self.config_frame.checkbox_use_sim_time.configure(state=state)
        self.config_frame.checkbox_rosbag_record.configure(state=state)
        self.__enable_disable_pipeline_checkboxes(state=state)
        discipline = self.config_frame.om_discipline.get()
        # if discipline in ["Trackdrive", "Skidpad"]:
        #     self.config_frame.checkbox_lap_opt.configure(state=state)

    def __deselect_for_play(self):
        # self.config_frame.checkbox_lap_opt.deselect()
        # self.config_frame.checkbox_use_sim_time.deselect()
        self.config_frame.checkbox_rosbag_record.deselect()

    def __enable_and_select_optionals(self):
        self.config_frame.checkbox_lapinfo.configure(state="normal")
        self.config_frame.checkbox_vel.configure(state="normal")
        self.config_frame.checkbox_track_length.configure(state="normal")
        self.config_frame.checkbox_cones_col.configure(state="normal")
        self.config_frame.checkbox_cones_fov.configure(state="normal")
        self.config_frame.checkbox_ego_frame.configure(state="normal")
        self.config_frame.checkbox_vehicle_frame.configure(state="normal")
        self.config_frame.checkbox_online_map.configure(state="normal")
        self.config_frame.checkbox_bounded_path.configure(state="normal")
        self.config_frame.checkbox_delaunay.configure(state="normal")
        self.config_frame.checkbox_control_boundaries.configure(state="normal")
        self.config_frame.checkbox_lapinfo.select()
        self.config_frame.checkbox_vel.select()
        self.config_frame.checkbox_track_length.select()
        self.config_frame.checkbox_cones_col.select()
        self.config_frame.checkbox_cones_fov.select()
        self.config_frame.checkbox_ego_frame.select()
        self.config_frame.checkbox_vehicle_frame.select()
        self.config_frame.checkbox_online_map.select()
        self.config_frame.checkbox_bounded_path.select()
        self.config_frame.checkbox_delaunay.select()
        self.config_frame.checkbox_control_boundaries.select()

    def select_rosbag(self):
        """
        Open window manager for selecting rosbag.
        """
        if self.config_frame.checkbox_rosbag_record.get():
            filename = fd.asksaveasfilename(
                initialdir="../amzsim_rosbag_handler/rosbags",
                title="Select directory for rosbag",
                filetypes=[("Rosbag", "*.bag")],
            )
        else:
            filename = fd.askopenfilename(
                initialdir="../amzsim_rosbag_handler/rosbags",
                title="Select rosbag",
                filetypes=[("Rosbag", "*.bag")],
            )
        self.input_to_path_entry(filename)

    def input_to_path_entry(self, text):
        self.config_frame.entry_rosbag_path.configure(state="normal")
        self.config_frame.entry_rosbag_path.configure(placeholder_text=text)
        self.config_frame.entry_rosbag_path.configure(state="disabled")

    def adjust_vehicle_settings(self, car):
        if car == "castor":
            self.config_frame.om_pge.configure(state="disabled")
            self.config_frame.checkbox_roll.configure(state="disabled")
            self.config_frame.entry_tv_ff.configure(state="disabled")
            self.config_frame.entry_tv_exp.configure(state="disabled")
            self.config_frame.entry_tv_p.configure(state="disabled")
            self.config_frame.entry_tv_i.configure(state="disabled")
            self.config_frame.entry_tv_d.configure(state="disabled")
            self.config_frame.entry_ax_m.configure(state="disabled")
            self.config_frame.entry_ax_q.configure(state="disabled")
            self.config_frame.entry_ax_p.configure(state="disabled")
            self.config_frame.entry_ax_i.configure(state="disabled")
            self.config_frame.entry_ax_d.configure(state="disabled")
        elif car == "dufour":
            self.config_frame.om_pge.configure(state="normal")
            self.config_frame.checkbox_roll.configure(state="normal")
            self.config_frame.entry_tv_ff.configure(state="normal")
            self.config_frame.entry_tv_exp.configure(state="normal")
            self.config_frame.entry_tv_p.configure(state="normal")
            self.config_frame.entry_tv_i.configure(state="normal")
            self.config_frame.entry_tv_d.configure(state="normal")
            self.config_frame.entry_ax_m.configure(state="normal")
            self.config_frame.entry_ax_q.configure(state="normal")
            self.config_frame.entry_ax_p.configure(state="normal")
            self.config_frame.entry_ax_i.configure(state="normal")
            self.config_frame.entry_ax_d.configure(state="normal")
