FROM ros:crystal-ros-base-bionic

MAINTAINER Lander Usategui  <lander@erlerobotics.com>

ENV ROS_DISTRO crystal
WORKDIR /ros2_ws

RUN \
    apt-get update && apt-get install -y \
        python3-colcon-common-extensions \
        libboost-all-dev \
        libglew-dev \
        freeglut3-dev \
        pkg-config \
        libfcl-dev \
        libassimp-dev \
        libqhull-dev \
        ros-$ROS_DISTRO-action-msgs \
        ros-$ROS_DISTRO-resource-retriever \
        ros-$ROS_DISTRO-image-transport \
        python3-vcstool \
        libopencv-dev \
        wget \
        && rm -rf /var/lib/apt/lists/* \
    # Setup ROS2 workspace
    && mkdir -p /ros2_ws/src && cd /ros2_ws \
    && wget https://raw.githubusercontent.com/AcutronicRobotics/moveit2/master/external-repos.repos \
    && vcs import src < external-repos.repos
COPY ./docker-entrypoint.sh /
ENTRYPOINT ["/docker-entrypoint.sh"]
