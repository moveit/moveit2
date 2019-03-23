#!/bin/bash

export LANG=C.UTF-8
export LC_ALL=C.UTF-8

export RESET="\e[0m"
export CYAN="\e[36m"
export RED="\e[31m"

export ROS_DISTRO=crystal
export ROS_PATH="/opt/ros/crystal"
export WS="/home/$USER/moveit2_ws"

function checkSystem()
{
	if [ ! $(command -v lsb_release) ]; then
		sudo bash -c "apt update && apt install -y lsb-release"
	fi

	system=$(lsb_release -cs)

	if [ "${system}" != "bionic" ]; then
		echo -e "${RED}This script only works under Ubuntu Bionic${RESET}"
		exit 0
        fi
	
}

function installCommonDependencies()
{
	echo -e "${CYAN}Install common dependencies for moveit2...${RESET}"
	sudo bash -c "apt update \
	 	      && apt install  -y \
	  	             build-essential \
	                     cmake \
	                     git \
	                     python3-pip \
	                     wget \
	      	             libboost-all-dev \
	                     libglew-dev \
	                     freeglut3-dev \
	                     pkg-config \
	                     libfcl-dev \
	                     libassimp-dev \
	                     libqhull-dev \
	                     libopencv-dev"
	echo -e "${CYAN}Common dependencies installed!${RESET}"
}

function checkROS()
{
	if [ ! -d "${ROS_PATH}" ]; then
		echo -e "${CYAN}No ROS2 crystal found in your system, we're going to install it${RESET}"
		sleep 2
      		sudo bash -c "apt-get update -qq && apt install -qq -y tzdata dirmngr gnupg2 curl \
                              && curl http://repo.ros2.org/repos.key | apt-key add - \
      		              && echo 'deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main' > /etc/apt/sources.list.d/ros2-latest.list \
      		              && apt-get update && apt-get install -y python3-vcstool python3-colcon-common-extensions \
         		                 	        	ros-$ROS_DISTRO-ros-base \
         			                  	        ros-$ROS_DISTRO-action-msgs \
         				                        ros-$ROS_DISTRO-message-filters \
         				                        ros-$ROS_DISTRO-rclcpp-action \
         				                        ros-$ROS_DISTRO-resource-retriever \
	   			                                ros-$ROS_DISTRO-yaml-cpp-vendor"
		echo -e "${CYAN}Installation process completed!${RESET}"
	else
		echo -e "${CYAN}ROS2 cystal is in your system, updating to fetch the latest version${RESET}"
		sleep 2
		sudo bash -c "apt update && apt install -y \
			                        python3-vcstool \
						python3-colcon-common-extensions \
				                ros-$ROS_DISTRO-ros-base \
			                        ros-$ROS_DISTRO-action-msgs \
			                        ros-$ROS_DISTRO-message-filters \
			                        ros-$ROS_DISTRO-rclcpp-action \
			                        ros-$ROS_DISTRO-resource-retriever \
				                ros-$ROS_DISTRO-yaml-cpp-vendor"
		echo -e "${CYAN}Update process completed!${RESET}"
	fi

}

function prepareWS()
{
	if [ ! -d "${WS}" ]; then
		echo -e "${CYAN}Creating the work space to compile${RESET}"
		sleep 1
		mkdir -p $WS/src
		echo -e "${CYAN}Done!!${RESET}"
	else
		echo -e "${RED}The ws=${WS} exists, change the export on top of this script to avoid the problem\nVariable that you should change: WS${RESET}"
		exit 0
	fi
}

function downloadSource()
{
	echo -e "${CYAN}Downloading the repos to compile moveit2...${RESET}"
	cd $WS
	wget https://raw.githubusercontent.com/AcutronicRobotics/moveit2/master/moveit2.repos
	$(echo -e "  moveit2:\n    type: git\n    url: https://github.com/AcutronicRobotics/moveit2\n    version: master" >> moveit2.repos)
	vcs import src < moveit2.repos
	echo -e "${CYAN}Work space ready to compile!${RESET}"
}

function compileWS()
{
	echo -e "${CYAN}Compilling the work space, this will take a few minutes...${RESET}"
	find $ROS_PATH -name tf2_eigen | xargs rm -rf
	cd $WS
	source $ROS_PATH/setup.bash
	colcon build --merge-install
	result=$?
	
	if [ $result -eq 0 ]; then
		echo -e "${CYAN}MoveIt2 compiled properly${RESET}"
	else
		echo -e "${RED}MoveIt2 is not properly compiled${RESET}"
	fi
}

checkSystem
installCommonDependencies
checkROS
prepareWS
downloadSource
compileWS
