# moveit/moveit2:foxy-ci
# ROS base image augmented with all MoveIt dependencies to use for CI on Travis

ARG ROS_DISTRO=foxy
FROM ros:${ROS_DISTRO}-ros-base-focal

MAINTAINER Robert Haschke rhaschke@techfak.uni-bielefeld.de
MAINTAINER Dave Coleman dave@picknik.ai

ENV TERM xterm

# Setup catkin workspace
ENV ROS_WS=/opt/ws_moveit

WORKDIR $ROS_WS

# Update apt package list as previous containers clear the cache
RUN apt-get -qq update && \
    apt-get -qq dist-upgrade -y && \
    #
    # Install some base dependencies
    apt-get -qq install -y \
        # Required by moveit_ci
        wget curl sudo xvfb mesa-utils ccache ssh curl gnupg2 lsb-release \
        clang clang-format-10 clang-tidy clang-tools \
        # Some python dependencies for working with ROS2
        python3-colcon-common-extensions \
        python3-pip \
        python3-rosdep \
        python3-wstool \
        python3-vcstool \
        python3-rospkg-modules \
        python3-rosdistro-modules \
        && \
    # Clear apt-cache to reduce image size
    rm -rf /var/lib/apt/lists/*

# Download moveit source and fetch all its dependencies
RUN mkdir src && \
    cd $ROS_WS/src && \
    wget -q https://raw.githubusercontent.com/ros-planning/moveit2/main/moveit2.repos && \
    vcs import < moveit2.repos && \
    # Remove folders declared as COLCON_IGNORE
    find -L . -name COLCON_IGNORE -printf "%h\0" | xargs -0 rm -rf && \
    apt-get -qq update && \
    rosdep update -q && \
    rosdep install -q -y --from-paths . --ignore-src --rosdistro ${ROS_DISTRO} --as-root=apt:false && \
    # Clear apt-cache to reduce image size
    rm -rf /var/lib/apt/lists/* && \
    #
    # Remove the source code from this container
    cd $ROS_WS && \
    rm -rf src/

# Continous Integration Setting
ENV IN_DOCKER 1
