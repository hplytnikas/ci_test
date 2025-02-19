#!/bin/bash

SCRIPTS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
REPO_ROOT_DIR="$( realpath $SCRIPTS_DIR/../..)"

source $REPO_ROOT_DIR/install/setup.bash
sudo ldconfig

ros2 launch amzsim_interface amzsim_interface.launch.py \
	discipline_name_arg:=trackdrive \
	track_path_arg:=/home/ubuntu/autonomous_2024/install/amzsim_tracks/share/amzsim_tracks/tracks_csv/standard/autocross_trackdrive/FSG.csv \
	control_node_name_arg:=purepursuit \
	record_rosbag_arg:=False \
	ve_gt_arg:=True \
	perception_gt_arg:=True \
	estimation_gt_arg:=True \
	grip_estimation_gt_arg:=False \
	pipeline_id_arg:=fusion \
	use_sim_time_arg:=False \
	rviz_on_arg:=False \
	show_lap_info_arg:=True \
	show_velocity_arg:=True \
	show_track_length_arg:=False \
	show_collision_detect_arg:=False \
	show_fov_cones_arg:=False \
	show_ego_frame_arg:=True \
	show_veh_frame_arg:=True \
	show_online_map_arg:=False \
	show_bounded_path_arg:=False \
	show_delaunay_arg:=False \
	show_control_bounds_arg:=True \
	tv_ff_arg:=1.0 \
	tv_exp_arg:=2.45 \
	tv_p_arg:=325.0 \
	tv_i_arg:=0.0 \
	tv_d_arg:=0.0 \
	ax_m_arg:=190.0 \
	ax_q_arg:=100.0 \
	ax_p_arg:=250.0 \
	ax_i_arg:=100.0 \
	ax_d_arg:=10.0 \
	pge_arg:=0.0 \
	roll_arg:=False \
	rosbag_path_arg:=path
