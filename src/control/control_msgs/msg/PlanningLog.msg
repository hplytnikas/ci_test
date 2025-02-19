# Logging message for the path planner, published after a planning step
std_msgs/Header header

float64[] x_points_be
float64[] y_points_be

int32 num_points_be
int32 optimization_status
# -1: no optimization done
# 0: all good
# 1: optimization failed (Global Planner: first and last may not be the same here)
# 2: not enough points
# 3: NaN in the solution
# 4: Points ordered incorrectly or duplicated
# 5: Issue checking the curvature
# 6: Missing a point in the path

float64 frequency

bool enable_opt
bool enable_prof
bool enable_viz
bool enable_current_vel_init
bool enable_straight_line_stopper
bool enable_safe_set_learning

float64 safe_velocity_multiplier
float64 safe_velocity_blend

float64 total_safety_dist
int32 nb_iter

float64 max_radius_for_path_from_car

bool first_point_fixed
bool last_point_fixed

int32 sampling_resolution

float64 max_lat_acc
float64 min_longit_speed
float64 max_longit_speed

float64 curv_smoothness
float64 gaussian_kernel_std

float64 straight_line_distance_thresh
float64 straight_line_curvature_thresh

float64 max_curvature
float64 max_distance
float64 kappa_shift

int32 hpipm_mode
int32 hpipm_iter_max
float64 hpipm_alpha_min
float64 hpipm_mu0
float64 hpipm_tol_stat
float64 hpipm_tol_eq
float64 hpipm_tol_ineq
float64 hpipm_tol_comp
float64 hpipm_reg_prim
float64 hpipm_reg_dual
int32 hpipm_warm_start
int32 hpipm_pred_corr
int32 hpipm_split_step
