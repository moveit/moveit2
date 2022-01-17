#!/bin/bash

# Script to test ikfast plugin creation and functionality

# We will create ikfast plugins for fanuc and panda from moveit_resources
# using the script auto_create_ikfast_moveit_plugin.sh

set -e # fail script on error

sudo apt-get -qq update
sudo apt-get -qq install python3-lxml python3-yaml

# Clone moveit_resources for URDFs. They are not available before running docker.
git clone -q --depth=1 -b ros2 https://github.com/ros-planning/moveit_resources /tmp/resources
fanuc=/tmp/resources/fanuc_description/urdf/fanuc.urdf
panda=/tmp/resources/panda_description/urdf/panda.urdf

export QUIET=${QUIET:=1}

# Create ikfast plugins for Fanuc and Panda
moveit_kinematics/ikfast_kinematics_plugin/scripts/auto_create_ikfast_moveit_plugin.sh \
	--name fanuc --pkg $PWD/fanuc_ikfast_plugin $fanuc manipulator base_link tool0

moveit_kinematics/ikfast_kinematics_plugin/scripts/auto_create_ikfast_moveit_plugin.sh \
	--name panda --pkg $PWD/panda_ikfast_plugin $panda panda_arm panda_link0 panda_link8

echo "Done."
