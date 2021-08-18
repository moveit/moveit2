# ghcr.io/ros-planning/moveit2:${ROS_DISTRO}-release
# Full debian-based install of MoveIt using apt-get

ARG ROS_DISTRO=foxy
FROM ros:${ROS_DISTRO}-ros-base-focal
MAINTAINER Dave Coleman dave@picknik.ai

# Commands are combined in single RUN statement with "apt/lists" folder removal to reduce image size
RUN apt-get update && \
    apt-get install -y ros-${ROS_DISTRO}-moveit-* && \
    rm -rf /var/lib/apt/lists/*
