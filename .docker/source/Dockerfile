# moveit/moveit2:crystal-source
# Downloads the moveit source code and install remaining debian dependencies

FROM moveit/moveit2:crystal-ci
LABEL maintainer="mike@picknik.ai"

# Commands are combined in single RUN statement with "apt/lists" folder removal to reduce image size
# https://docs.docker.com/develop/develop-images/dockerfile_best-practices/#minimize-the-number-of-layers
RUN mkdir src && \
    cd src && \
    #
    # Download moveit source so that we can get necessary dependencies
    wstool init . https://raw.githubusercontent.com/ros-planning/moveit2/master/moveit.rosinstall

# Update apt package list as cache is cleared in previous container
# Usually upgrading involves a few packages only (if container builds became out-of-sync)
RUN apt-get -qq update && \
    apt-get -qq dist-upgrade && \
    rosdep update && \
    rosdep install -y --from-paths . --ignore-src --rosdistro ${ROS_DISTRO} --as-root=apt:false && \
    rm -rf /var/lib/apt/lists/*

    # Build the workspace
RUN colcon build --symlink-install --event-handlers console_direct+
