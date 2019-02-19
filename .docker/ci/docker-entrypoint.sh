#!/bin/bash

set -e

/bin/bash -c "source /opt/ros/crystal/setup.bash && colcon build --merge-install"
