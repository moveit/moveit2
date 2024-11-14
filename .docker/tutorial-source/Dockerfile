# syntax = docker/dockerfile:1.3

# ghcr.io/moveit/moveit2:main-${ROS_DISTRO}-tutorial-source
# Source build of the repos file from the tutorial site

ARG ROS_DISTRO=humble

FROM moveit/moveit2:${ROS_DISTRO}-ci
LABEL maintainer Tyler Weaver tyler@picknik.ai

# Export ROS_UNDERLAY for downstream docker containers
ENV ROS_UNDERLAY /root/ws_moveit/install
WORKDIR $ROS_UNDERLAY/..

# Copy MoveIt sources from docker context
COPY . src/moveit2

# Commands are combined in single RUN statement with "apt/lists" folder removal to reduce image size
# https://docs.docker.com/develop/develop-images/dockerfile_best-practices/#minimize-the-number-of-layers
RUN --mount=type=cache,target=/root/.ccache/,sharing=locked \
        # Enable ccache
        PATH=/usr/lib/ccache:$PATH && \
        # Checkout the tutorial repo
        git clone -b ${ROS_DISTRO} https://github.com/moveit/moveit2_tutorials src/moveit2_tutorials && \
        # Fetch required upstream sources for building
        vcs import --skip-existing src < src/moveit2_tutorials/moveit2_tutorials.repos && \
        # Source ROS install
        . "/opt/ros/${ROS_DISTRO}/setup.sh" &&\
        # Install dependencies from rosdep
        apt-get -q update && \
        rosdep update && \
        DEBIAN_FRONTEND=noninteractive \
        rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} --as-root=apt:false && \
        rm -rf /var/lib/apt/lists/* && \
        # Build the workspace
        colcon build \
        --cmake-args "-DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --no-warn-unused-cli" \
        --ament-cmake-args -DCMAKE_BUILD_TYPE=Release \
        --event-handlers desktop_notification- status- && \
        ccache -s && \
        #
        # Update /ros_entrypoint.sh to source our new workspace
        sed -i "s#/opt/ros/\$ROS_DISTRO/setup.bash#$ROS_UNDERLAY/setup.sh#g" /ros_entrypoint.sh
