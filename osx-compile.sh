#!/bin/bash

set -x #Debug
set -e #exit on failure

#Prepare for ros2
cd /tmp
wget https://github.com/ros2/ros2/releases/download/release-crystal-20190214/ros2-crystal-20190214-macos-amd64.tar.bz2
tar jxf ros2-crystal-20190214-macos-amd64.tar.bz2

source ros2-osx/setup.bash
mkdir -p /tmp/ros2_ws/src
cp -r /tmp/moveit2 /tmp/ros2_ws/src
cd /tmp/ros2_ws && wget https://raw.githubusercontent.com/AcutronicRobotics/moveit2/master/external-repos.repos
vcs import src < external-repos.repos
colcon build --merge-install
