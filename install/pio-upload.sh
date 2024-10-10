#!/bin/bash
# Save the current directory
original_dir=$(pwd)


roscd linorobot/teensy/firmware
platformio run --target upload -e teensy40

# Return to the original directory
cd "$original_dir"
