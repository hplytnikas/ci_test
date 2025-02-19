# Logging Message for MPC, called after a controller step
std_msgs/Header header

# Solver Information
float64 solvetime_ms
int32 solverflag
int32 solverstatus
int32 solvertype

# Reference Trajectory
float64[] x_refpath
float64[] y_refpath
float64[] vel_reference
float64[] boundary_min_reference
float64[] boundary_max_reference

# Predictions
float64[] n_prediction
float64[] mu_prediction
float64[] vx_prediction
float64[] vy_prediction
float64[] dpsi_prediction
float64[] ax_prediction
float64[] dels_prediction
float64[] dax_prediction
float64[] ddels_prediction

# measured state
float64 n_meas
float64 mu_meas
float64 vx_meas
float64 vy_meas
float64 dpsi_meas
float64 ax_meas
float64 dels_meas

# intiial state of the MPC
float64 n_init
float64 mu_init
float64 vx_init
float64 vy_init
float64 dpsi_init
float64 ax_init
float64 dels_init

# Parameters that change along the horizon
float64[] curvature_horizon
float64[] progress_horizon
float64[] trackbound_min_horizon
float64[] trackbound_max_horizon
float64[] vel_horizon

# Model Parameters
float64 model_mass
float64 model_lf
float64 model_lr
float64 model_cd
float64 model_cr
float64 model_cl
float64 model_b_tire
float64 model_c_tire
float64 model_d_tire
float64 model_iz
float64 model_h_cg
float64 model_car_width
float64 model_l_to_front
float64 model_l_to_rear

# Cost Parameters
float64 cost_q_ds
float64 cost_q_ds_terminal
float64 cost_q_n
float64 cost_q_n_terminal
float64 cost_q_mu
float64 cost_q_mu_terminal
float64 cost_r_dax
float64 cost_r_ddels

# Constraints
float64 constr_vx_max
float64 constr_vx_max_terminal
float64 constr_axt_abs_max
float64 constr_ayt_abs_max
float64 constr_dels_abs_max
float64 constr_dax_min
float64 constr_dax_max
float64 constr_ddels_abs_max
float64 constr_trackbound_shrinkage

# delay compensation parameters
float64 mpc_dt_interp_ax
float64 mpc_dt_interp_dels
float64 mpc_dt_interp_dpsi
float64 tau_ax
float64 tau_dels

# Standstill hardcoded parameters
float64 ax_command_standstill
float64 vel_threshold_standstill

# some other parameters
int32 consecutive_failures_limit
bool enable_visualization
bool enable_logging
bool enable_profiling
bool enable_output_interpolation
bool enable_delay_compensation
float64 controller_frequency
int32 controller_timeout_reset
int32 controller_timeout_fatal
float64 dt_mpc
