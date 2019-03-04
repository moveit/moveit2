#!/bin/bash

set -e

/bin/bash -c "find /opt/ros/crystal/ -name tf2_eigen | xargs rm -rf && source /opt/ros/crystal/setup.bash && colcon build --merge-install"
