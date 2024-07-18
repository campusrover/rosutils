#!/bin/bash
# Save the current directory
original_dir=$(pwd)

cw
roscd linorobot/teensy/firmware
platformio run -e teensy40

# Return to the original directory
cd "$original_dir"
