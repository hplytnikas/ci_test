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
# pylint: disable=too-many-locals;
# pylint: disable=consider-using-with;

import subprocess
import time
import rclpy
import signal
from kill_nodes import kill_active_nodes


class Launcher:
    """
    Model part of MVP design pattern. It is responsible for running roscore and roslaunch commands.
    Essentially this class allows to run roslaunch amzsim_interface with arguments values received
    from presenter, which in turn received it from GUI. In order to not lock the application,
    these commands are run as separate subprocesses with subprocess python library.

    Attributes
    ----------
    sim_launched : bool
        Boolean which tells if sim is currently running.
    roscore_process : Popen
        Popen object responsible for running and terminating commmand "roscore"
        to execute roscore as subprocess.
    lap_opt_process : Popen
        Popen object responsible for running and terminating commmand
        "roslaunch control_laptimeopt some_laptime_opt" to execute some laptimeopt
        node as subprocess.
    clock_process : Popen
        Popen object responsible for running and terminating commmand
        "roslaunch amzsim_time" to execute amzsim_time node as subprocess.
    roslaunch_process : Popen
        Popen object responsible for running and terminating commmand
        "roslaunch amzsim_interface arg1:=smth arg2:=smth to execute amzsim_interface
        with different arguments. This in turns runs the nodes from amzsim_backend
        in different ways based on args.
    """

    def __init__(self):
        self.sim_launched = False
        self.roscore_process = None
        self.lap_opt_process = None
        self.clock_process = None
        self.roslaunch_process = None

    def launch_simulator(self, sim_config):
        """
        Launches all of the sim subprocesses based on the configuration received from Presenter.
        The order of execution is as follows:
        1. Run roscore in the background.
        2. Run laptimeopt node if selected in GUI.
        3. Wait for laptimeopt to finish.
        4. Run clock in the background.
        5. Run roslaunch amzsim_interface with arguments given from sim_config

        Parameters
        ----------
        sim_config : dict
            Dictionary containing simulator configuration selected by the user.
        """

        # launch_lap_opt = bool(sim_config.get("lap_opt"))
        # self.__launch_roscore()
        # if launch_lap_opt:
        #     self.__launch_lap_opt(sim_config)
        #     self.__wait_for_lap_opt_finished()
        # self.__launch_clock()
        self.__launch_amzsim_backend(sim_config)
        self.sim_launched = True

    def stop_simulation(self):
        """
        Stops the simulation and terminates the subprocesses if they are still launched.
        """

        self.sim_launched = False
        # if self.lap_opt_process is not None:
        #     self.lap_opt_process.terminate()
        #     self.lap_opt_process = None
        # if self.clock_process is not None:
        #     self.clock_process.terminate()
        #     self.clock_process = None
        if self.roslaunch_process is not None:
            self.roslaunch_process.send_signal(signal.SIGINT)
            # self.roslaunch_process.terminate()
            self.roslaunch_process = None
            kill_active_nodes()
        # if self.roscore_process is not None:
        #     self.roscore_process.terminate()
        #     self.roscore_process = None

    def get_sim_launched(self):
        """
        Getter for sim_launched attribute.
        """

        return self.sim_launched

    def __get_published_topics():
        rclpy.init()
        node = rclpy.create_node("get_published_topics_node")

        # Get the list of currently published topics
        topic_names_and_types = node.get_topic_names_and_types()

        # Clean up
        node.destroy_node()
        # rclpy.shutdown()

        return topic_names_and_types

    def __launch_roscore(self):
        """
        Runs roscore subprocess.
        """

        launch_roscore = ["ros2daemon"]
        self.roscore_process = subprocess.Popen(launch_roscore)

    def __launch_lap_opt(self, sim_config):
        """
        Runs lapopt subprocess. Depending on whether discipline selected by user
        is skidpad or trackdrive, method will run different laptimeopt nodes.

        Parameters
        ----------
        sim_config : dict
            Dictionary containing simulator configuration selected by the user. Used
            here to retrieve "discipline" selected by user.
        """

        discipline = sim_config.get("discipline").lower()
        if discipline == "skidpad":
            launch_lap_opt = [
                "roslaunch",
                "--wait",
                "control_laptimeopt",
                "laptimeopt-amzsim_skidpad.launch",
            ]
            self.lap_opt_process = subprocess.Popen(launch_lap_opt)
        if discipline == "trackdrive":
            launch_lap_opt = [
                "roslaunch",
                "--wait",
                "control_laptimeopt",
                "laptimeopt-amzsim.launch",
            ]
            self.lap_opt_process = subprocess.Popen(launch_lap_opt)

    def __wait_for_lap_opt_finished(self):
        """
        Waits for laptimeopt subprocess to finish. Essentially it waits till topic
        "/control/optimal_raceline" responsible for lapopt disappears.
        """

        time.sleep(3)
        lap_opt_topic = "/control/optimal_raceline"
        while lap_opt_topic in self.__get_topics():
            time.sleep(0.1)
        print("Lap opt for simulation has finished")

    def __get_topics(self):
        """
        Gets list of current active rostopics
        """

        topics = self.__get_published_topics()
        flattened_topics = [item for sublist in topics for item in sublist]
        return flattened_topics

    def __launch_clock(self):
        """
        Runs clock subprocess.
        """

        launch_clock = ["roslaunch", "--wait", "amzsim_time", "amzsim_time.launch"]
        self.clock_process = subprocess.Popen(launch_clock)

    def __launch_amzsim_backend(self, sim_config):
        """
        Turns on the application.
        """

        launch_list = self.__construct_launch_list_backend(sim_config)
        self.roslaunch_process = subprocess.Popen(launch_list)

    def __construct_launch_list_backend(self, sim_config):
        """
        Runs roslaunch amzsim_interface command. Depending on sim_config,
        different parameters are passed to this command.

        Parameters
        ----------
        sim_config : dict
            Dictionary containing simulator configuration selected by the user. Used
            here to retrieve all of the selected values from checkboxes, optionmenus etc.
        """

        discipline = sim_config.get("discipline").lower()
        track_path = sim_config.get("track path")
        control_node = sim_config.get("control node").lower()
        rosbag_record = str(bool(sim_config.get("rosbag_record")))
        rosbag_play = str(bool(sim_config.get("rosbag_play")))
        rosbag_path = sim_config.get("rosbag_path")
        ve_gt = str(bool(sim_config.get("ve gt")))
        perception_gt = str(bool(sim_config.get("perception gt")))
        estimation_gt = str(bool(sim_config.get("estimation gt")))
        # grip_estimation = str(bool(sim_config.get("grip estimation")))
        grip_estimation = "false"
        use_sim_time = str(bool(sim_config.get("use sim time")))
        # rviz = str(bool(sim_config.get("rviz")))
        # lap_info = str(bool(sim_config.get("show lap info")))
        # velocity = str(bool(sim_config.get("show velocity")))
        # track_length = str(bool(sim_config.get("show track length")))
        # collision_detect = str(bool(sim_config.get("show collision detect")))
        # fov_cones = str(bool(sim_config.get("show fov cones")))
        # ego_frame = str(bool(sim_config.get("show ego frame")))
        # veh_frame = str(bool(sim_config.get("show veh frame")))
        # online_map = str(bool(sim_config.get("show online map")))
        # bounded_path = str(bool(sim_config.get("show bounded path")))
        # delaunay = str(bool(sim_config.get("show delaunay")))
        # control_bounds = str(bool(sim_config.get("show control bounds")))
        rviz = "true"
        lap_info = "false"
        velocity = "false"
        track_length = "false"
        collision_detect = "false"
        fov_cones = "false"
        ego_frame = "false"
        veh_frame = "false"
        online_map = "false"
        bounded_path = "false"
        delaunay = "false"
        control_bounds = "false"
        pipeline_id = sim_config.get("perception config").lower()
        # mpc_llc = str(bool(sim_config.get("mpc llc")))
        tv_ff = str(float(sim_config.get("tv ff")))
        tv_exp = str(float(sim_config.get("tv exp")))
        tv_p = str(float(sim_config.get("tv p")))
        tv_i = str(float(sim_config.get("tv i")))
        tv_d = str(float(sim_config.get("tv d")))
        ax_m = str(float(sim_config.get("ax m")))
        ax_q = str(float(sim_config.get("ax q")))
        ax_p = str(float(sim_config.get("ax p")))
        ax_i = str(float(sim_config.get("ax i")))
        ax_d = str(float(sim_config.get("ax d")))
        pge = str(float(sim_config.get("pge")))
        roll = str(bool(sim_config.get("roll")))

        # If simulation played from rosbag then ignore configuration.
        if rosbag_play == "True":
            launch_list = [
                "ros2",
                "launch",
                "amzsim_interface",
                "amzsim_interface.launch.py",
                "play_rosbag_arg:=" + rosbag_play,
                "rviz_on_arg:=" + rviz,
                "show_lap_info_arg:=" + lap_info,
                "show_velocity_arg:=" + velocity,
                "show_track_length_arg:=" + track_length,
                "show_collision_detect_arg:=" + collision_detect,
                "show_fov_cones_arg:=" + fov_cones,
                "show_ego_frame_arg:=" + ego_frame,
                "show_veh_frame_arg:=" + veh_frame,
                "show_online_map_arg:=" + online_map,
                "show_bounded_path_arg:=" + bounded_path,
                "show_delaunay_arg:=" + delaunay,
                "show_control_bounds_arg:=" + control_bounds,
            ]
        else:
            launch_list = [
                "ros2",
                "launch",
                "amzsim_interface",
                "amzsim_interface.launch.py",
                "discipline_name_arg:=" + discipline,
                "track_path_arg:=" + track_path,
                "control_node_name_arg:=" + control_node,
                "record_rosbag_arg:=" + rosbag_record,
                "ve_gt_arg:=" + ve_gt,
                "perception_gt_arg:=" + perception_gt,
                "estimation_gt_arg:=" + estimation_gt,
                "grip_estimation_gt_arg:=" + grip_estimation,
                "pipeline_id_arg:=" + pipeline_id,
                "use_sim_time_arg:=" + use_sim_time,
                "rviz_on_arg:=" + rviz,
                "show_lap_info_arg:=" + lap_info,
                "show_velocity_arg:=" + velocity,
                "show_track_length_arg:=" + track_length,
                "show_collision_detect_arg:=" + collision_detect,
                "show_fov_cones_arg:=" + fov_cones,
                "show_ego_frame_arg:=" + ego_frame,
                "show_veh_frame_arg:=" + veh_frame,
                "show_online_map_arg:=" + online_map,
                "show_bounded_path_arg:=" + bounded_path,
                "show_delaunay_arg:=" + delaunay,
                "show_control_bounds_arg:=" + control_bounds,
                "tv_ff_arg:=" + tv_ff,
                "tv_exp_arg:=" + tv_exp,
                "tv_p_arg:=" + tv_p,
                "tv_i_arg:=" + tv_i,
                "tv_d_arg:=" + tv_d,
                "ax_m_arg:=" + ax_m,
                "ax_q_arg:=" + ax_q,
                "ax_p_arg:=" + ax_p,
                "ax_i_arg:=" + ax_i,
                "ax_d_arg:=" + ax_d,
                "pge_arg:=" + pge,
                "roll_arg:=" + roll,
            ]
        if not rosbag_path == "":
            launch_list.append("rosbag_path_arg:=" + rosbag_path)
        print("Simulation backend nodes launched with command: " + str(launch_list))
        return launch_list
