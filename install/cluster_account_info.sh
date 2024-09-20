#!/bin/bash

# Get the number of CPU cores
num_cores=$(nproc)
num_cores2=$(grep -c ^processor /proc/cpuinfo)

# Get the number of GPUs using lshw
num_gpus=$(lshw -C display 2>/dev/null | grep -i 'display' | wc -l)

# Get memory information
mem_info=$(free -h)
total_mem=$(echo "$mem_info" | awk '/^Mem:/{print $2}')
free_mem=$(echo "$mem_info" | awk '/^Mem:/{print $4}')

# Get disk information
disk_info=$(df -h --total | grep 'total')
total_disk=$(echo "$disk_info" | awk '{print $2}')
free_disk=$(echo "$disk_info" | awk '{print $4}')

# Print the results
echo "Number of CPU cores: $num_cores $num_cores2"
echo "Number of GPUs: $num_gpus"
echo "Total Memory: $total_mem"
echo "Free Memory: $free_mem"
echo "Total Disk Space: $total_disk"
echo "Free Disk Space: $free_disk"