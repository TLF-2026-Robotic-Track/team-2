#!/bin/bash
# Copy the pictures saved by the camera node from the robot to this computer.
#
# RUN THIS ON YOUR OWN COMPUTER, not on the robot.
#
#   ./get_images.sh duckie03                    # default folder name
#   ./get_images.sh duckie03 my-repo            # if your repo folder differs
#
# Password is quackquack, unless you set up an SSH key.
set -e

ROBOT=${1:?usage: ./get_images.sh <robot name> [repo folder on the robot]}
REPO=${2:-DuckieExamples}
DEST=./images_from_$ROBOT

mkdir -p "$DEST"
scp "duckie@$ROBOT.local:~/$REPO/images/*.jpg" "$DEST"/
echo "Pictures are in $DEST"
