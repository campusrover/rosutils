#!/bin/bash
source ~/.bashrc
roscd linorobot/teensy/firmware
pio run --target upload -e teensy41
