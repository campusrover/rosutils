#!/usr/bin/env bash

#
# Script that installs PJRC UDEV rules for allowing to use Teensy-based devices.
# Downloads the rules file, checks that it matches the SHA256 checksum in this
# script and then moves the downloaded UDEV file
# to "/lib/udev/rules.d/49-teensy.rules".
#
# Note: you need to be able to run sudo to use this script.
# Note: from https://github.com/zsa/docs/issues/14
#
# Run without providing any arguments:
#
# ./install-teensy-udev-rules.sh
#

set -e

UDEV_RULES_NAME="00-teensy.rules"
UDEV_RULES_URL="https://www.pjrc.com/teensy/$UDEV_RULES_NAME"
UDEV_RULES_SHA256="031de0b26991b5a3b19c497d9c0a17f86c40c55d925b9d07d19ab89f2286469d  $UDEV_RULES_NAME"
UDEV_RULES_DEST="/lib/udev/rules.d/00-teensy.rules"

WORK_DIR=$(mktemp -d)

function finish {
  echo ">> Cleaning up work dir"
  popd
  rm -rf "$WORK_DIR"
  echo "-- Clean up done"
}
trap finish EXIT

pushd "$WORK_DIR"

echo ">> Downloading UDEV rules file"

curl "$UDEV_RULES_URL" --output "$UDEV_RULES_NAME"
echo "031de0b26991b5a3b19c497d9c0a17f86c40c55d925b9d07d19ab89f2286469d  $UDEV_RULES_NAME" > "$UDEV_RULES_NAME.sha256"

sha256sum -c "$UDEV_RULES_NAME.sha256"

if [ -f "$UDEV_RULES_DEST" ]; then
    echo "UDEV rule '$UDEV_RULES_DEST' is already installed, quitting."
    exit
fi

echo ">> Installing UDEV rules file"

sudo install -o root -g root -m 0664 "$UDEV_RULES_NAME" "$UDEV_RULES_DEST"
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "-- Installation complete"