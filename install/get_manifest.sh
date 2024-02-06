cat /proc/device-tree/model > _config_pi_model
free -h > _config_memory_pi
# For now I have to do a platformio bui;d to get info about the teensy.