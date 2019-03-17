#!/bin/bash

set -e

/bin/bash -c "find /opt/ros/crystal/ -name tf2_eigen | xargs rm -rf && source /opt/ros/crystal/setup.bash \
	       && touch /root/ws_moveit/src/image_common/camera_calibration_parsers/COLCON_IGNORE \
              && touch /root/ws_moveit/src/image_common/camera_info_manager/COLCON_IGNORE \
              && colcon build --merge-install"
