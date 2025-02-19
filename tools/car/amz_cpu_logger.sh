#!/bin/bash
log_path_cpu_info="/home/amz/cpu_log/cpus.log"
log_path_process_info="/home/amz/cpu_log/processes.log"
while true; do
	printf "\n\n\n\n" >> "$log_path_cpu_info"
	printf "\n\n\n\n" >> "$log_path_process_info"

	date >> "$log_path_cpu_info"
	date >> "$log_path_process_info"

	pidstat | sort -k8 -r | awk '{print  $8, $9, $10, $2}' | column -t | head -40 >> "$log_path_process_info"
	mpstat -P ALL | awk '{print $2, $3, $5, $6, $12}' | column -t >> "$log_path_cpu_info"

	sleep 10  # Sleep for 10 seconds before the next log; adjust as needed
done
