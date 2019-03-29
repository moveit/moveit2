# moveit/moveit2:crystal-ci
# Sets up a base image to use for running Continuous Integration on Travis
FROM ros:crystal-ros-base-bionic

LABEL maintainer="mike@picknik.ai"

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
        wget \
        # Required for rosdep command
        sudo \
        clang clang-format-3.9 clang-tidy clang-tools \
        ccache \
        curl gnupg2 lsb-release \
        # Some python dependencies for working with ROS2
        python3-colcon-common-extensions \
        python3-pip \
        python-rosdep \
        python3-wstool \
        python3-rospkg-modules \
        python3-rosdistro-modules \
        && \
    # Clear apt-cache to reduce image size
    rm -rf /var/lib/apt/lists/*

# Upgrade Pip and install some packages needed for testing
# NOTE(mlautman): These pip installs are from the ros2 source install instructions. Seeing as not all of them
#        are installed in the base image I added them here. I have not been able to verify that they are needed
#        and if they aren't necessary they should be removed.
RUN python3 -m pip install -q --upgrade pip && \
    python3 -m pip install -q -U \
        argcomplete \
        flake8 \
        flake8-blind-except \
        flake8-builtins \
        flake8-class-newline \
        flake8-comprehensions \
        flake8-deprecated \
        flake8-docstrings \
        flake8-import-order \
        flake8-quotes \
        pytest-repeat \
        pytest-rerunfailures \
        pytest \
        pytest-cov \
        pytest-runner \
        setuptools

# Install Fast-RTPS dependencies
RUN apt-get -qq update && \
    apt-get install -qq --no-install-recommends -y \
        libasio-dev \
        libtinyxml2-dev \
        && \
    # Clear apt-cache to reduce image size
    rm -rf /var/lib/apt/lists/*

# Download moveit source and fetch all necessary dependencies
RUN mkdir src && \
    cd $ROS_WS/src && \
    # TODO(mlautman): This should be changed to use vcs in the future
    wstool init --shallow . https://raw.githubusercontent.com/ros-planning/moveit2/master/moveit.rosinstall && \
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
