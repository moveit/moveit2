# ghcr.io/moveit/moveit2:${ROS_DISTRO}-release
# Full debian-based install of MoveIt using apt-get

ARG ROS_DISTRO=rolling
FROM ros:${ROS_DISTRO}-ros-base
LABEL maintainer Robert Haschke rhaschke@techfak.uni-bielefeld.de

# Allow non-interactive installation of ros-*-rmw-connextdds
ENV RTI_NC_LICENSE_ACCEPTED yes

# Commands are combined in single RUN statement with "apt/lists" folder removal to reduce image size
RUN apt-get update -q && \
    apt-get upgrade -q -y && \
    apt-get install -y "ros-${ROS_DISTRO}-moveit-*" --no-install-recommends && \
    rm -rf /var/lib/apt/lists/*
