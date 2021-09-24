# ghcr.io/ros-planning/moveit2:${ROS_DISTRO}-release
# Full debian-based install of MoveIt using apt-get

ARG ROS_DISTRO=rolling
FROM ros:${ROS_DISTRO}-ros-base-focal
MAINTAINER Robert Haschke rhaschke@techfak.uni-bielefeld.de

# Commands are combined in single RUN statement with "apt/lists" folder removal to reduce image size
RUN apt-get update -q && \
    apt-get dist-upgrade -q -y && \
    apt-get install -y ros-${ROS_DISTRO}-moveit-* && \
    rm -rf /var/lib/apt/lists/*
