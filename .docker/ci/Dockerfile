# moveit/moveit2:foxy-ci
# ROS base image augmented with all MoveIt dependencies to use for CI on Travis

ARG ROS_DISTRO=foxy
FROM ros:${ROS_DISTRO}-ros-base-focal

MAINTAINER Robert Haschke rhaschke@techfak.uni-bielefeld.de
MAINTAINER Dave Coleman dave@picknik.ai

ENV TERM xterm

WORKDIR /root/ws_moveit

# Commands are combined in single RUN statement with "apt/lists" folder removal to reduce image size
# https://docs.docker.com/develop/develop-images/dockerfile_best-practices/#minimize-the-number-of-layers
RUN \
    # Update apt package list as previous containers clear the cache
    apt-get -qq update && \
    apt-get -qq dist-upgrade && \
    #
    # Install some base dependencies
    apt-get -qq install --no-install-recommends -y \
        # Some basic requirements
        wget git sudo curl \
        # Preferred build tools
        clang clang-format-10 clang-tidy clang-tools \
        ccache && \
    #
    # Download MoveIt source, so that we can fetch all necessary dependencies
    mkdir src && \
    git clone https://github.com/ros-planning/moveit2.git src/moveit2 -b main && \
    vcs import src < src/moveit2/moveit2.repos && \
    #
    # Download all dependencies of MoveIt
    rosdep update && \
    DEBIAN_FRONTEND=noninteractive \
    rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} --as-root=apt:false && \
    # Remove the source code from this container
    rm -rf src && \
    #
    # Clear apt-cache to reduce image size
    rm -rf /var/lib/apt/lists/*
