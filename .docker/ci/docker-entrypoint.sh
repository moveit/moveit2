#!/bin/bash

set -e

/bin/bash -c "find /opt/ros/crystal/ -name tf2_eigen | xargs rm -rf && source /opt/ros/crystal/setup.bash \
	            && touch /ros2_ws/src/image_common/camera_calibration_parsers/COLCON_IGNORE \
              && touch /ros2_ws/src/image_common/camera_info_manager/COLCON_IGNORE \
              && colcon build --merge-install"
