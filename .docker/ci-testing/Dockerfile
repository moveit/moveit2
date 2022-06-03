# ghcr.io/ros-planning/moveit2:${ROS_DISTRO}-ci-testing
# CI image using the ROS testing repository

ARG ROS_DISTRO=rolling
FROM ghcr.io/ros-planning/moveit2:${ROS_DISTRO}-ci
LABEL maintainer Robert Haschke rhaschke@techfak.uni-bielefeld.de

# Switch to ros-testing
RUN echo "deb http://packages.ros.org/ros2-testing/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2-latest.list

# Upgrade packages to ros-shadow-fixed and clean apt-cache within one RUN command
RUN apt-get -qq update && \
    apt-get -qq upgrade && \
    #
    # Clear apt-cache to reduce image size
    rm -rf /var/lib/apt/lists/*
