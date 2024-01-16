#!/bin/bash
cw
roscd linorobot/teensy/firmware
platformio run --target upload -e teensy41
