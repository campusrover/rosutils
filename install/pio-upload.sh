#!/bin/bash
# Save the current directory
original_dir=$(pwd)

cw
roscd linorobot/teensy/firmware
platformio run --target upload -e teensy41

# Return to the original directory
cd "$original_dir"