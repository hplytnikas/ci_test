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
# pylint: disable= too-few-public-methods
# pylint: disable= missing-module-docstring
import customtkinter


class ConfigFrameBuilder:
    """
    Class that is used to create and instantiate Config components, such as checkboxes, optionmenu
    internal frames etc. and puts them in correct places.

    Attributes
    ----------
    config_frame : ConfigFrame
        Class ConfigFrame for which components will be created.
    """

    def __init__(self, config_frame):
        self.config_frame = config_frame

    def build_configuration_frame(self):
        """
        Builds all of the frame components.
        """
        self.__configure_config_frame()
        self.__build_tracks_frame()
        self.__build_rosbag_frame()
        self.__build_pipeline_frame()
        # self.__build_optionals_frame()
        self.__build_vehicle_frame()

    def __configure_config_frame(self):
        """
        Configures config frame object and its grid.
        """
        self.config_frame.configure(corner_radius=0, fg_color="transparent")
        self.config_frame.grid_columnconfigure(1, weight=1)
        # self.config_frame.grid_rowconfigure(0, weight=1)
        # self.config_frame.grid_rowconfigure(1, weight=1)
        # self.config_frame.grid_rowconfigure(2, weight=1)

    def __build_tracks_frame(self):
        """
        Setups discipline and tracks internal frame and its components inside config frame.
        """
        # Create discipline_tracks frame inside config frame
        self.config_frame.frame_tracks_disc = customtkinter.CTkFrame(self.config_frame)
        self.config_frame.frame_tracks_disc.grid(
            row=0, column=0, padx=(20, 10), pady=(20, 10), sticky="nsew"
        )
        self.config_frame.frame_tracks_disc.grid_columnconfigure(0, weight=1)

        # COLUMN 1
        # Create discipline label
        self.config_frame.label_discipline = customtkinter.CTkLabel(
            self.config_frame.frame_tracks_disc,
            text="Discipline:",
            font=customtkinter.CTkFont(size=18, weight="bold"),
        )
        self.config_frame.label_discipline.grid(
            row=0, column=0, padx=40, pady=(40, 0), sticky="nsew"
        )
        # Create discipline optionmenu
        self.config_frame.om_discipline = customtkinter.CTkOptionMenu(
            self.config_frame.frame_tracks_disc,
            dynamic_resizing=False,
            values=["Acceleration", "Autocross", "Trackdrive", "Skidpad"],
        )
        self.config_frame.om_discipline.grid(row=1, column=0, padx=40, pady=(10, 5))

        # Create control node label
        self.config_frame.label_control_node = customtkinter.CTkLabel(
            self.config_frame.frame_tracks_disc,
            text="Control node:",
            font=customtkinter.CTkFont(size=18, weight="bold"),
        )
        self.config_frame.label_control_node.grid(
            row=2, column=0, padx=40, pady=(20, 0), sticky="nsew"
        )
        # Create control node optionmenu
        self.config_frame.om_control_node = customtkinter.CTkOptionMenu(
            self.config_frame.frame_tracks_disc, dynamic_resizing=False
        )
        self.config_frame.om_control_node.grid(row=3, column=0, padx=40, pady=(10, 5))

        # Create lapopt checkbox
        # self.config_frame.checkbox_lap_opt = customtkinter.CTkCheckBox(
        #     self.config_frame.frame_tracks_disc,
        #     text="Run LapOpt",
        #     font=customtkinter.CTkFont(size=16),
        # )
        # self.config_frame.checkbox_lap_opt.grid(
        #     row=4, column=0, padx=40, pady=20, sticky="nsew"
        # )

        # COLUMN 2
        # Create track type label
        self.config_frame.label_track_type = customtkinter.CTkLabel(
            self.config_frame.frame_tracks_disc,
            text="Track type:",
            font=customtkinter.CTkFont(size=18, weight="bold"),
        )
        self.config_frame.label_track_type.grid(
            row=0, column=1, padx=40, pady=(40, 0), sticky="nsew"
        )
        # Create track type optionmenu
        self.config_frame.om_track_type = customtkinter.CTkOptionMenu(
            self.config_frame.frame_tracks_disc,
            dynamic_resizing=False,
            values=["Standard", "GTMD", "Global_Map", "Custom"],
        )
        self.config_frame.om_track_type.grid(row=1, column=1, padx=40, pady=(10, 5))

        # Create track label
        self.config_frame.label_track = customtkinter.CTkLabel(
            self.config_frame.frame_tracks_disc,
            text="Track:",
            font=customtkinter.CTkFont(size=18, weight="bold"),
        )
        self.config_frame.label_track.grid(
            row=2, column=1, padx=40, pady=(20, 0), sticky="nsew"
        )
        # Create tracks optionmenu
        self.config_frame.om_track = customtkinter.CTkOptionMenu(
            self.config_frame.frame_tracks_disc, dynamic_resizing=False
        )
        self.config_frame.om_track.grid(row=3, column=1, padx=40, pady=(10, 5))

        # Create checkbox sim time
        # self.config_frame.checkbox_use_sim_time = customtkinter.CTkCheckBox(
        #     self.config_frame.frame_tracks_disc,
        #     text="Use sim time",
        #     font=customtkinter.CTkFont(size=16),
        # )
        # self.config_frame.checkbox_use_sim_time.grid(
        #     row=4, column=1, padx=40, pady=20, sticky="nsew"
        # )

    def __build_rosbag_frame(self):
        """
        Setups rosbag internal frame and its components inside config frame.
        """
        # Create rosbag frame inside config frame
        self.config_frame.frame_rosbag = customtkinter.CTkFrame(self.config_frame)
        self.config_frame.frame_rosbag.grid(
            row=0, column=1, padx=(10, 10), pady=(20, 10), sticky="nsew"
        )
        self.config_frame.frame_rosbag.grid_columnconfigure(0, weight=1)

        # Create rosbag frame label
        self.config_frame.label_rosbag = customtkinter.CTkLabel(
            self.config_frame.frame_rosbag,
            text="Rosbags:",
            font=customtkinter.CTkFont(size=18, weight="bold"),
        )
        self.config_frame.label_rosbag.grid(
            row=0, column=0, padx=40, pady=(40, 20), sticky="nsew"
        )

        # Create rosbag record checkbox
        self.config_frame.checkbox_rosbag_record = customtkinter.CTkCheckBox(
            self.config_frame.frame_rosbag,
            text="Rosbag record:",
            font=customtkinter.CTkFont(size=16),
        )
        self.config_frame.checkbox_rosbag_record.grid(
            row=1, column=0, padx=30, pady=10, sticky="nsew"
        )

        # Create rosbag play checkbox
        self.config_frame.checkbox_rosbag_play = customtkinter.CTkCheckBox(
            self.config_frame.frame_rosbag,
            text="Rosbag play:",
            font=customtkinter.CTkFont(size=16),
        )
        self.config_frame.checkbox_rosbag_play.grid(
            row=2, column=0, padx=30, pady=10, sticky="nsew"
        )

        # Create rosbag path entry
        self.config_frame.entry_rosbag_path = customtkinter.CTkEntry(
            self.config_frame.frame_rosbag,
            placeholder_text="path",
            width=200,
            state="disabled",
        )
        self.config_frame.entry_rosbag_path.grid(
            row=3, column=0, padx=30, pady=(10, 5), sticky="nsew"
        )
        self.config_frame.button_path = customtkinter.CTkButton(
            self.config_frame.frame_rosbag,
            text="Select rosbag",
            width=200,
            state="disabled",
        )

        # Create rosbag path button
        self.config_frame.button_path.grid(
            row=4, column=0, padx=30, pady=(0, 40), sticky="nsew"
        )

    def __build_pipeline_frame(self):
        """
        Setups pipeline internal frame and its components inside config frame.
        """
        # Create pipeline frame inside config frame
        self.config_frame.frame_pipeline = customtkinter.CTkFrame(self.config_frame)
        self.config_frame.frame_pipeline.grid(
            row=0, column=2, padx=(10, 20), pady=(20, 10), sticky="nsew"
        )
        self.config_frame.frame_pipeline.grid_columnconfigure(0, weight=1)
        # self.config_frame.frame_pipeline.grid_rowconfigure((0, 1, 2, 3, 4), weight=1)

        # Create pipeline frame label
        self.config_frame.label_pipeline = customtkinter.CTkLabel(
            self.config_frame.frame_pipeline,
            text="Pipeline:",
            font=customtkinter.CTkFont(size=18, weight="bold"),
        )
        self.config_frame.label_pipeline.grid(
            row=0, column=0, columnspan=2, padx=40, pady=(40, 15), sticky="nsew"
        )

        # Create checkbox perception gt
        self.config_frame.checkbox_perception_gt = customtkinter.CTkCheckBox(
            self.config_frame.frame_pipeline,
            text="Perception GT",
            font=customtkinter.CTkFont(size=16),
        )
        self.config_frame.checkbox_perception_gt.grid(
            row=2, column=0, padx=40, pady=10, sticky="nsew"
        )

        # Create checkbox ve gt
        self.config_frame.checkbox_ve_gt = customtkinter.CTkCheckBox(
            self.config_frame.frame_pipeline,
            text="VE GT",
            font=customtkinter.CTkFont(size=16),
        )
        self.config_frame.checkbox_ve_gt.grid(
            row=1, column=0, padx=40, pady=10, sticky="nsew"
        )

        # Create checkbox estimation gt
        self.config_frame.checkbox_estimation_gt = customtkinter.CTkCheckBox(
            self.config_frame.frame_pipeline,
            text="Estimation GT",
            font=customtkinter.CTkFont(size=16),
        )
        self.config_frame.checkbox_estimation_gt.grid(
            row=3, column=0, padx=40, pady=10, sticky="nsew"
        )

        # Create checkbox grip estimation
        # self.config_frame.checkbox_grip_estimation = customtkinter.CTkCheckBox(
        #     self.config_frame.frame_pipeline,
        #     text="Grip estimation",
        #     font=customtkinter.CTkFont(size=16),
        # )
        # self.config_frame.checkbox_grip_estimation.grid(
        #     row=4, column=0, padx=40, pady=(10, 40), sticky="nsew"
        # )

        # Create dropdown perception configuration
        self.config_frame.om_perception_config = customtkinter.CTkOptionMenu(
            self.config_frame.frame_pipeline,
            dynamic_resizing=False,
            values=["Fusion", "Camera", "Lidar"],
        )
        self.config_frame.om_perception_config.grid(
            row=2, column=1, padx=40, pady=(10, 5)
        )

    def __build_optionals_frame(self):
        """
        Setups optionals internal frame and its components inside config frame.
        """
        # Create optionals frame inside config frame
        self.config_frame.frame_optionals = customtkinter.CTkFrame(self.config_frame)
        self.config_frame.frame_optionals.grid(
            row=2, column=0, columnspan=4, padx=(20, 20), pady=(10, 10), sticky="nsew"
        )
        self.config_frame.frame_optionals.grid_columnconfigure((0, 1, 2, 3), weight=1)
        self.config_frame.frame_optionals.grid_rowconfigure((0, 1, 2), weight=1)

        # First column of optionals
        # Create show rviz checkbox
        self.config_frame.checkbox_rviz = customtkinter.CTkCheckBox(
            self.config_frame.frame_optionals,
            text="Show Rviz",
            font=customtkinter.CTkFont(size=16),
        )
        self.config_frame.checkbox_rviz.grid(
            row=0, column=0, padx=(40, 30), pady=(40, 10), sticky="nsew"
        )
        self.config_frame.checkbox_rviz.select()

        # Create show lap info checkbox
        self.config_frame.checkbox_lapinfo = customtkinter.CTkCheckBox(
            self.config_frame.frame_optionals,
            text="Show lap info",
            font=customtkinter.CTkFont(size=16),
        )
        self.config_frame.checkbox_lapinfo.grid(
            row=1, column=0, padx=(40, 30), pady=10, sticky="nsew"
        )

        # Create show velocity checkbox
        self.config_frame.checkbox_vel = customtkinter.CTkCheckBox(
            self.config_frame.frame_optionals,
            text="Show velocity",
            font=customtkinter.CTkFont(size=16),
        )
        self.config_frame.checkbox_vel.grid(
            row=2, column=0, padx=(40, 30), pady=10, sticky="nsew"
        )

        # Create show track length checkbox
        self.config_frame.checkbox_track_length = customtkinter.CTkCheckBox(
            self.config_frame.frame_optionals,
            text="Show track length",
            font=customtkinter.CTkFont(size=16),
        )
        self.config_frame.checkbox_track_length.grid(
            row=0, column=3, padx=30, pady=(40, 10), sticky="nsew"
        )

        # Second column of optionals
        # Create show track length checkbox
        self.config_frame.checkbox_cones_col = customtkinter.CTkCheckBox(
            self.config_frame.frame_optionals,
            text="Show cones collisions",
            font=customtkinter.CTkFont(size=16),
        )
        self.config_frame.checkbox_cones_col.grid(
            row=0, column=1, padx=30, pady=(40, 10), sticky="nsew"
        )

        # Create show cones fov checkbox
        self.config_frame.checkbox_cones_fov = customtkinter.CTkCheckBox(
            self.config_frame.frame_optionals,
            text="Show cones in FOV",
            font=customtkinter.CTkFont(size=16),
        )
        self.config_frame.checkbox_cones_fov.grid(
            row=1, column=1, padx=30, pady=10, sticky="nsew"
        )

        # Create show ego frame checkbox
        self.config_frame.checkbox_ego_frame = customtkinter.CTkCheckBox(
            self.config_frame.frame_optionals,
            text="Show egomotion frame",
            font=customtkinter.CTkFont(size=16),
        )
        self.config_frame.checkbox_ego_frame.grid(
            row=2, column=1, padx=30, pady=10, sticky="nsew"
        )

        # Create show veh frame checkbox
        self.config_frame.checkbox_vehicle_frame = customtkinter.CTkCheckBox(
            self.config_frame.frame_optionals,
            text="Show vehicle frame",
            font=customtkinter.CTkFont(size=16),
        )
        self.config_frame.checkbox_vehicle_frame.grid(
            row=1, column=3, padx=30, pady=10, sticky="nsew"
        )

        # Third column of optionals
        # Create show online map checkbox
        self.config_frame.checkbox_online_map = customtkinter.CTkCheckBox(
            self.config_frame.frame_optionals,
            text="Show online map",
            font=customtkinter.CTkFont(size=16),
        )
        self.config_frame.checkbox_online_map.grid(
            row=0, column=2, padx=30, pady=(40, 10), sticky="nsew"
        )

        # Create show bounded path checkbox
        self.config_frame.checkbox_bounded_path = customtkinter.CTkCheckBox(
            self.config_frame.frame_optionals,
            text="Show bounded path",
            font=customtkinter.CTkFont(size=16),
        )
        self.config_frame.checkbox_bounded_path.grid(
            row=1, column=2, padx=30, pady=10, sticky="nsew"
        )

        # Create show delaunay lines checkbox
        self.config_frame.checkbox_delaunay = customtkinter.CTkCheckBox(
            self.config_frame.frame_optionals,
            text="Show delaunay lines",
            font=customtkinter.CTkFont(size=16),
        )
        self.config_frame.checkbox_delaunay.grid(
            row=2, column=2, padx=30, pady=10, sticky="nsew"
        )

        # Create show control boundaries checkbox
        self.config_frame.checkbox_control_boundaries = customtkinter.CTkCheckBox(
            self.config_frame.frame_optionals,
            text="Show control boundaries",
            font=customtkinter.CTkFont(size=16),
        )
        self.config_frame.checkbox_control_boundaries.grid(
            row=2, column=3, padx=30, pady=10, sticky="nsew"
        )

    def __build_vehicle_frame(self):
        """
        Setup vehicle internal frame and its components inside config frame.
        """
        var = customtkinter.DoubleVar()
        self.config_frame.frame_vehicle = customtkinter.CTkFrame(self.config_frame)
        self.config_frame.frame_vehicle.grid(
            row=1, column=0, columnspan=4, padx=(20, 20), pady=(10, 10), sticky="nsew"
        )
        self.config_frame.frame_vehicle.grid_columnconfigure((0, 1, 2, 3), weight=1)
        # self.config_frame.frame_vehicle.grid_rowconfigure(
        #     (0, 1, 2, 3, 4, 5, 6, 7, 8, 9)
        # )

        # Create Vehicle label
        self.config_frame.label_vehicle = customtkinter.CTkLabel(
            self.config_frame.frame_vehicle,
            text="Vehicle",
            font=customtkinter.CTkFont(size=18, weight="bold"),
        )
        self.config_frame.label_vehicle.grid(
            row=0, column=0, columnspan=4, padx=0, pady=(20, 0), sticky="nsew"
        )

        # Create car optionmenu
        self.config_frame.om_car = customtkinter.CTkOptionMenu(
            self.config_frame.frame_vehicle,
            dynamic_resizing=False,
            values=["dufour"],
            font=customtkinter.CTkFont(size=14, weight="bold"),
        )
        self.config_frame.om_car.grid(
            row=1, column=0, columnspan=4, padx=0, pady=(0, 20)
        )

        self.config_frame.label_pge = customtkinter.CTkLabel(
            self.config_frame.frame_vehicle,
            text="PGE (%)",
            font=customtkinter.CTkFont(size=16, weight="bold"),
        )
        self.config_frame.label_pge.grid(
            row=2, column=0, padx=(20, 20), pady=(15, 15), sticky="nsew"
        )

        self.config_frame.om_pge = customtkinter.CTkOptionMenu(
            self.config_frame.frame_vehicle,
            dynamic_resizing=False,
            values=["0", "25", "50", "75", "100"],
        )
        self.config_frame.om_pge.grid(
            row=3, column=0, padx=(100, 100), pady=(0, 30), sticky="nsew"
        )

        self.config_frame.label_tv_ff = customtkinter.CTkLabel(
            self.config_frame.frame_vehicle,
            text="TV FF Gain",
            font=customtkinter.CTkFont(size=16, weight="bold"),
        )
        self.config_frame.label_tv_ff.grid(
            row=2, column=1, padx=(20, 20), pady=(15, 15), sticky="nsew"
        )

        self.config_frame.entry_tv_ff = customtkinter.CTkEntry(
            self.config_frame.frame_vehicle,
            placeholder_text="1.0",
        )
        self.config_frame.entry_tv_ff.insert(0, "1.0")
        self.config_frame.entry_tv_ff.grid(
            row=3, column=1, padx=(100, 100), pady=(0, 30), sticky="nsew"
        )

        self.config_frame.label_tv_exp = customtkinter.CTkLabel(
            self.config_frame.frame_vehicle,
            text="TV Exp Gain",
            font=customtkinter.CTkFont(size=16, weight="bold"),
        )
        self.config_frame.label_tv_exp.grid(
            row=6, column=0, padx=(20, 20), pady=(15, 15), sticky="nsew"
        )

        self.config_frame.entry_tv_exp = customtkinter.CTkEntry(
            self.config_frame.frame_vehicle,
            placeholder_text="2.45",
        )
        self.config_frame.entry_tv_exp.insert(0, "2.45")
        self.config_frame.entry_tv_exp.grid(
            row=7, column=0, padx=(100, 100), pady=(0, 30), sticky="nsew"
        )

        self.config_frame.label_tv_p = customtkinter.CTkLabel(
            self.config_frame.frame_vehicle,
            text="TV P",
            font=customtkinter.CTkFont(size=16, weight="bold"),
        )
        self.config_frame.label_tv_p.grid(
            row=2, column=2, padx=(20, 20), pady=(15, 15), sticky="nsew"
        )

        self.config_frame.entry_tv_p = customtkinter.CTkEntry(
            self.config_frame.frame_vehicle,
            placeholder_text="325",
        )
        self.config_frame.entry_tv_p.insert(0, "325")
        self.config_frame.entry_tv_p.grid(
            row=3, column=2, padx=(100, 100), pady=(0, 30), sticky="nsew"
        )

        self.config_frame.label_tv_i = customtkinter.CTkLabel(
            self.config_frame.frame_vehicle,
            text="TV I",
            font=customtkinter.CTkFont(size=16, weight="bold"),
        )
        self.config_frame.label_tv_i.grid(
            row=4, column=2, padx=(20, 20), pady=(15, 15), sticky="nsew"
        )

        self.config_frame.entry_tv_i = customtkinter.CTkEntry(
            self.config_frame.frame_vehicle,
            placeholder_text="0",
        )
        self.config_frame.entry_tv_i.insert(0, "0")
        self.config_frame.entry_tv_i.grid(
            row=5, column=2, padx=(100, 100), pady=(0, 30), sticky="nsew"
        )

        self.config_frame.label_tv_d = customtkinter.CTkLabel(
            self.config_frame.frame_vehicle,
            text="TV D",
            font=customtkinter.CTkFont(size=16, weight="bold"),
        )
        self.config_frame.label_tv_d.grid(
            row=6, column=2, padx=(20, 20), pady=(15, 15), sticky="nsew"
        )

        self.config_frame.entry_tv_d = customtkinter.CTkEntry(
            self.config_frame.frame_vehicle,
            placeholder_text="0",
        )
        self.config_frame.entry_tv_d.insert(0, "0")
        self.config_frame.entry_tv_d.grid(
            row=7, column=2, padx=(100, 100), pady=(0, 30), sticky="nsew"
        )

        # Create Rolling Resistance Checkbox
        self.config_frame.checkbox_roll = customtkinter.CTkCheckBox(
            self.config_frame.frame_vehicle,
            text=" High Rolling Resistance",
            font=customtkinter.CTkFont(size=16),
        )
        self.config_frame.checkbox_roll.grid(
            row=4, column=0, rowspan=2, padx=(30, 30), pady=(30, 30), sticky="ns"
        )

        # Create Ax m Label
        self.config_frame.label_ax_m = customtkinter.CTkLabel(
            self.config_frame.frame_vehicle,
            text="Ax m",
            font=customtkinter.CTkFont(size=16, weight="bold"),
        )
        self.config_frame.label_ax_m.grid(
            row=4, column=1, padx=(20, 20), pady=(15, 15), sticky="nsew"
        )

        # Create Ax m Entry
        self.config_frame.entry_ax_m = customtkinter.CTkEntry(
            self.config_frame.frame_vehicle,
            placeholder_text="190",
        )
        self.config_frame.entry_ax_m.insert(0, "190")
        self.config_frame.entry_ax_m.grid(
            row=5, column=1, padx=(100, 100), pady=(0, 30), sticky="nsew"
        )

        # Create Ax q Label
        self.config_frame.label_ax_q = customtkinter.CTkLabel(
            self.config_frame.frame_vehicle,
            text="Ax q",
            font=customtkinter.CTkFont(size=16, weight="bold"),
        )
        self.config_frame.label_ax_q.grid(
            row=6, column=1, padx=(20, 20), pady=(15, 15), sticky="nsew"
        )

        # Create Ax q Entry
        self.config_frame.entry_ax_q = customtkinter.CTkEntry(
            self.config_frame.frame_vehicle,
            placeholder_text="100",
        )
        self.config_frame.entry_ax_q.insert(0, "100")
        self.config_frame.entry_ax_q.grid(
            row=7, column=1, padx=(100, 100), pady=(0, 30), sticky="nsew"
        )

        # Create Ax P Label
        self.config_frame.label_ax_p = customtkinter.CTkLabel(
            self.config_frame.frame_vehicle,
            text="Ax P",
            font=customtkinter.CTkFont(size=16, weight="bold"),
        )
        self.config_frame.label_ax_p.grid(
            row=2, column=3, padx=(20, 20), pady=(15, 15), sticky="nsew"
        )

        # Create Ax P Entry
        self.config_frame.entry_ax_p = customtkinter.CTkEntry(
            self.config_frame.frame_vehicle,
            placeholder_text="250",
        )
        self.config_frame.entry_ax_p.insert(0, "250")
        self.config_frame.entry_ax_p.grid(
            row=3, column=3, padx=(100, 100), pady=(0, 30), sticky="nsew"
        )

        # Create Ax I Label
        self.config_frame.label_ax_i = customtkinter.CTkLabel(
            self.config_frame.frame_vehicle,
            text="Ax I",
            font=customtkinter.CTkFont(size=16, weight="bold"),
        )
        self.config_frame.label_ax_i.grid(
            row=4, column=3, padx=(20, 20), pady=(15, 15), sticky="nsew"
        )

        # Create Ax I Entry
        self.config_frame.entry_ax_i = customtkinter.CTkEntry(
            self.config_frame.frame_vehicle,
            placeholder_text="100",
        )
        self.config_frame.entry_ax_i.insert(0, "100")
        self.config_frame.entry_ax_i.grid(
            row=5, column=3, padx=(100, 100), pady=(0, 30), sticky="nsew"
        )

        # Create Ax D Label
        self.config_frame.label_ax_d = customtkinter.CTkLabel(
            self.config_frame.frame_vehicle,
            text="Ax D",
            font=customtkinter.CTkFont(size=16, weight="bold"),
        )
        self.config_frame.label_ax_d.grid(
            row=6, column=3, padx=(20, 20), pady=(15, 15), sticky="nsew"
        )

        # Create Ax D Entry
        self.config_frame.entry_ax_d = customtkinter.CTkEntry(
            self.config_frame.frame_vehicle,
            placeholder_text="10",
        )
        self.config_frame.entry_ax_d.insert(0, "10")
        self.config_frame.entry_ax_d.grid(
            row=7, column=3, padx=(100, 100), pady=(0, 30), sticky="nsew"
        )
