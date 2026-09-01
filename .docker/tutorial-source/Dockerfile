# syntax = docker/dockerfile:1.3

# ghcr.io/moveit/moveit2:main-${ROS_DISTRO}-tutorial-source
# Source build of the repos file from the tutorial site

ARG ROS_DISTRO=rolling

FROM moveit/moveit2:${ROS_DISTRO}-source
LABEL maintainer="Robert Haschke rhaschke@techfak.uni-bielefeld.de"

# Commands are combined in single RUN statement with "apt/lists" folder removal to reduce image size
# https://docs.docker.com/develop/develop-images/dockerfile_best-practices/#minimize-the-number-of-layers
RUN --mount=type=cache,target=/root/.ccache/,sharing=locked \
        # Install Gazebo, which is needed by some dependencies.
        apt-get -q update && \
        apt-get install -y "ros-${ROS_DISTRO}-ros-gz" --no-install-recommends && \
        # Enable ccache
        PATH=/usr/lib/ccache:$PATH && \
        # Checkout the tutorial repo
        git clone https://github.com/moveit/moveit2_tutorials src/moveit2_tutorials && \
        # Fetch required upstream sources for building
        vcs import --skip-existing src < src/moveit2_tutorials/.github/upstream.repos && \
        # Source ROS install
        . "/opt/ros/${ROS_DISTRO}/setup.sh" &&\
        # Install dependencies from rosdep
        rosdep update && \
        DEBIAN_FRONTEND=noninteractive \
            rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} --as-root=apt:false && \
        rm -rf /var/lib/apt/lists/* && \
        # Build the workspace
        colcon build \
        --cmake-args "-DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --no-warn-unused-cli" \
        --ament-cmake-args -DCMAKE_BUILD_TYPE=Release \
        --event-handlers desktop_notification- status- && \
        ccache -s
