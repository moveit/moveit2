# ghcr.io/ros-planning/moveit2:${ROS_DISTRO}-release
# Full debian-based install of MoveIt using apt-get

ARG ROS_DISTRO=rolling
<<<<<<< HEAD
FROM ros:${ROS_DISTRO}-ros-base-focal
MAINTAINER Robert Haschke rhaschke@techfak.uni-bielefeld.de
=======
FROM ros:${ROS_DISTRO}-ros-base
LABEL maintainer Robert Haschke rhaschke@techfak.uni-bielefeld.de
>>>>>>> cea57649b (Add hadolint for linting dockerfiles (#1306))

# Commands are combined in single RUN statement with "apt/lists" folder removal to reduce image size
RUN apt-get update -q && \
    apt-get upgrade -q -y && \
    apt-get install -y "ros-${ROS_DISTRO}-moveit-*" --no-install-recommends && \
    rm -rf /var/lib/apt/lists/*
