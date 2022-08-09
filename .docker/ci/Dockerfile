# ghcr.io/ros-planning/moveit2:${ROS_DISTRO}-ci
# ROS base image augmented with all MoveIt dependencies to use for CI

ARG ROS_DISTRO=rolling
FROM ros:${ROS_DISTRO}-ros-base
LABEL maintainer Robert Haschke rhaschke@techfak.uni-bielefeld.de

ENV TERM xterm

# Setup (temporary) ROS workspace
WORKDIR /root/ws_moveit

# Copy MoveIt sources from docker context
COPY . src/moveit2

# Commands are combined in single RUN statement with "apt/lists" folder removal to reduce image size
# https://docs.docker.com/develop/develop-images/dockerfile_best-practices/#minimize-the-number-of-layers
RUN \
    # Update apt package list as previous containers clear the cache
    apt-get -q update && \
    apt-get -q -y upgrade && \
    #
    # Install some base dependencies
    apt-get -q install --no-install-recommends -y \
        # Some basic requirements
        wget git sudo curl \
        # Preferred build tools
        clang clang-format-12 clang-tidy clang-tools \
        ccache && \
    #
    # Globally disable git security
    # https://github.blog/2022-04-12-git-security-vulnerability-announced
    git config --global --add safe.directory "*" && \
    #
    # Fetch all dependencies from moveit2.repos
    vcs import src < src/moveit2/moveit2.repos && \
    if [ -r src/moveit2/moveit2_${ROS_DISTRO}.repos ] ; then vcs import src < src/moveit2/moveit2_${ROS_DISTRO}.repos ; fi && \
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
