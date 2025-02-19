std_msgs/Header header

# ---------------------
# Sensors
bool sensor_camera_alive
bool sensor_lidar_alive
bool velocity_estimation_alive
bool steering_feedback_alive

ModuleStatus sensor_camera
ModuleStatus sensor_lidar
ModuleStatus velocity_estimation
ModuleStatus steering_feedback
# ---------------------


# ---------------------
# Perception
bool camera_pipeline_alive
bool lidar_pipeline_alive
bool fusion_pipeline_alive
bool per_pipeline_alive

ModuleStatus camera_pipeline
ModuleStatus lidar_pipeline
ModuleStatus fusion_pipeline
ModuleStatus per_pipeline

uint8 num_cones_camera
uint8 num_cones_lidar
uint8 num_cones_fusion
uint8 num_cones_per
# ---------------------


# ---------------------
# Estimation
bool boundary_estimation_alive
bool online_map_alive

ModuleStatus boundary_estimation
ModuleStatus online_map
uint8 num_cones_map
# ---------------------


# ---------------------
# Control
bool control_alive

ModuleStatus control
# ---------------------
