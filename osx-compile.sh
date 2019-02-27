#!/bin/bash

set -x #Debug
set -e #exit on failure

mkdir -p /tmp/ros2_ws/src
cp -r /tmp/moveit2 /tmp/ros2_ws/src
cd /tmp/ros2_ws && wget https://raw.githubusercontent.com/AcutronicRobotics/moveit2/master/external-repos.repos
vcs import src < external-repos.repos
colcon build --merge-install
