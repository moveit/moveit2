# moveit/moveit2:foxy-ci-testing
# CI image using the ROS testing repository

ARG ROS_DISTRO=foxy
FROM moveit/moveit2:${ROS_DISTRO}-ci

MAINTAINER Dave Coleman dave@picknik.ai
MAINTAINER Robert Haschke rhaschke@techfak.uni-bielefeld.de

# Switch to ros-testing
RUN echo "deb http://packages.ros.org/ros2-testing/ubuntu `lsb_release -cs` main" | tee /etc/apt/sources.list.d/ros2-latest.list

# Upgrade packages to ros-shadow-fixed and clean apt-cache within one RUN command
RUN apt-get -qq update && \
    apt-get -qq dist-upgrade && \
    #
    # Clear apt-cache to reduce image size
    rm -rf /var/lib/apt/lists/*
