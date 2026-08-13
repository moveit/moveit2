# ghcr.io/moveit/moveit2:${ROS_DISTRO}-ci-testing
# CI image using the ROS testing repository

ARG ROS_DISTRO=rolling
FROM moveit/moveit2:${ROS_DISTRO}-ci
LABEL maintainer="Robert Haschke rhaschke@techfak.uni-bielefeld.de"

# Switch to ros-testing
RUN apt-get update && \
    apt-get install -y ros2-testing-apt-source && \
    # Upgrade packages to ros-testing and clean apt-cache within one RUN command
    apt-get update && \
    apt-get -qq -y dist-upgrade && \
    #
    # Clear apt-cache to reduce image size
    rm -rf /var/lib/apt/lists/*
