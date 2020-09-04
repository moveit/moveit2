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

# Download moveit source and fetch all its dependencies
# TODO: remove all this when the migration is finished and all dependencies are install from ci base image
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
