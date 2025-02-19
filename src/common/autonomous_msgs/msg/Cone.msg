int32 id_cone
int32 pipeline # bitarray where 1 = yolo_only, 2 = lidar_only, 4 = sensor_fusion
autonomous_msgs/ConeTypeProb prob_type
float32 prob_cone
geometry_msgs/Point32 position
autonomous_msgs/Covariance2D position_covariance
bool is_observed # important to test on old rosbag (at least for Path Planning)
