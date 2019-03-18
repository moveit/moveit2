# moveit/moveit2:crystal-ci
# Sets up a base image to use for running Continuous Integration on Travis
FROM ros:crystal-ros-base-bionic

LABEL maintainer="mike@picknik.ai"

ENV TERM xterm

# Setup catkin workspace
ENV ROS_WS=/root/ws_moveit
ENV ROS_DISTRO crystal

WORKDIR $ROS_WS

# Commands are combined in single RUN statement with "apt/lists" folder removal to reduce image size
# https://docs.docker.com/develop/develop-images/dockerfile_best-practices/#minimize-the-number-of-layers

RUN mkdir src && \
    cd $ROS_WS/src && \
    #
    # Download moveit source, so that we can get necessary dependencies
    wstool init --shallow . https://raw.githubusercontent.com/ros-planning/moveit2/moveit2-ci/moveit.rosinstall && \
    #
    # Update apt package list as previous containers clear the cache
    apt-get -qq update && \
    apt-get -qq dist-upgrade -y && \
    #
    # Install some base dependencies
    apt-get -qq install -y \
        # Some source builds require a package.xml be downloaded via wget from an external location
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
        python3-vcstool \
        python3-wstool \
        python3-rospkg-modules \
        python3-rospkg-modules \
        python3-rosdistro-modules \
        && \
    #
    # Upgrade Pip
    python3 -m pip install --upgrade pip && \
    #
    # install some pip packages needed for testing
    python3 -m pip install -U \
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
        # git+https://github.com/lark-parser/lark.git@0.7d \
        pytest-repeat \
        pytest-rerunfailures \
        pytest \
        pytest-cov \
        pytest-runner \
        setuptools \
        && \
    #
    # install Fast-RTPS dependencies
    apt-get install -qq --no-install-recommends -y \
        libasio-dev \
        libtinyxml2-dev \
        && \
    #
    # Download all dependencies of MoveIt!
    rosdep update && \
    # Some packages are skipped. See ROS2 source installation
    rosdep install -y --from-paths . --ignore-src --rosdistro ${ROS_DISTRO} --as-root=apt:false && \
    #
    # Remove the source code from this container
    # TODO: in the future we may want to keep this here for further optimization of later containers
    cd $ROS_WS && \
    rm -rf src/ && \
    # Clear apt-cache to reduce image size
    rm -rf /var/lib/apt/lists/*

# Continous Integration Setting
ENV IN_DOCKER 1
