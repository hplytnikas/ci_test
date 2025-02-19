std_msgs/Header header

float32[] cpu_percent
float32 cpu_percent_avg
float32 cpu_percent_max
float32[] cpu_freq
float32 cpu_freq_avg
float32 cpu_freq_max
float32 load_1min
float32 load_5min
float32 load_15min
float32[] cpu_temp
float32 cpu_temp_max

# GPU stuff
float32 gpu_percent
uint8 gpu_temp
uint16 gpu_memory_total_mb
uint16 gpu_memory_available_mb
float32 gpu_memory_usage_percent


uint16 memory_available_mb
float32 memory_usage_percent
uint16 swap_available_mb
float32 swap_usage_percent

uint16 disk_free_gb
float32 disk_usage_percent
float32 disk_read_mbs
float32 disk_write_mbs

float32 network_down_mbs
float32 network_up_mbs

# PC time is included in the timestamp of the header
bool time_synced
uint8 sync_state_flag
string sync_state
float32 sync_offset
float32 last_offset

# SSD Temperature
int32 ssd_temp
string ssd_critical_warning
