std_msgs/Header header

float32 ax_target                   # [-32, 32]
float32 ax_measured                 # [-32, 32]
float32 force_rolling_resistance    # [-32768, 32767]
float32 force_feedforward           # [-32768, 32767]
float32 force_drag                  # [-32768, 32767]
float32 force_standstill            # [-32768, 32767]
float32 force_pid                   # [-32768, 32767]
float32 force_total                 # [-32768, 32767]
