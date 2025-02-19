#vcu_msgs/msg/ResState.msg
# State of the three buttons on the RES
std_msgs/Header header
bool emergency # 0 = OK, 1 = Emergency
bool on_off_switch # 0 = off, 1 = on
bool push_button # 1 = pressed
bool communication_interrupted # 0 = OK, 1 = Interrupted ##Currently not used it is hardcoded to 0 in the VCU
