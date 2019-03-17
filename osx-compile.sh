#!/bin/bash

#set -x #Debug
set -e #exit on failure

#Prepare for ros2
cd /tmp
wget https://github.com/ros2/ros2/releases/download/release-crystal-20190314/ros2-crystal-20190314-macos-amd64.tar.bz2
tar jxf ros2-crystal-20190314-macos-amd64.tar.bz2
# Remove tf2_eigen
find ros2-osx/ -name tf2_eigen | xargs rm -rf

source ros2-osx/setup.bash
mkdir -p /tmp/ros2_ws/src
cp -r /tmp/moveit2 /tmp/ros2_ws/src
cd /tmp/ros2_ws && wget https://raw.githubusercontent.com/AcutronicRobotics/moveit2/master/moveit2.repos
vcs import src < moveit2.repos
export OPENSSL_ROOT_DIR="/usr/local/opt/openssl"
#Ignore packages
touch src/image_common/camera_calibration_parsers/COLCON_IGNORE
touch src/image_common/camera_info_manager/COLCON_IGNORE
colcon build --merge-install
