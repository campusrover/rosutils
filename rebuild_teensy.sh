#!/bin/bash
source ~/.bashrc
roscd linorobot/teensy/firmware
platformio run --target upload
