# syntax = docker/dockerfile:1.3

# ghcr.io/ros-planning/moveit2:${ROS_DISTRO}-source
# Downloads the moveit source code and install remaining debian dependencies

ARG ROS_DISTRO=rolling
FROM moveit/moveit2:${ROS_DISTRO}-ci-testing
LABEL maintainer Robert Haschke rhaschke@techfak.uni-bielefeld.de

# Export ROS_UNDERLAY for downstream docker containers
ENV ROS_UNDERLAY /root/ws_moveit/install
WORKDIR $ROS_UNDERLAY/..

# Copy MoveIt sources from docker context
COPY . src/moveit2

# Commands are combined in single RUN statement with "apt/lists" folder removal to reduce image size
# https://docs.docker.com/develop/develop-images/dockerfile_best-practices/#minimize-the-number-of-layers
RUN --mount=type=cache,target=/root/.ccache/ \
    # Enable ccache
    PATH=/usr/lib/ccache:$PATH && \
    # Fetch required upstream sources for building
    vcs import src < src/moveit2/moveit2.repos && \
    if [ -r "src/moveit2/moveit2_${ROS_DISTRO}.repos" ] ; then vcs import src < "src/moveit2/moveit2_${ROS_DISTRO}.repos" ; fi && \
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
            --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
            --ament-cmake-args -DCMAKE_BUILD_TYPE=Release \
            --event-handlers desktop_notification- status- && \
    ccache -s && \
    #
    # Update /ros_entrypoint.sh to source our new workspace
    sed -i "s#/opt/ros/\$ROS_DISTRO/setup.bash#$ROS_UNDERLAY/setup.sh#g" /ros_entrypoint.sh
