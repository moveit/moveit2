FROM ubuntu:bionic

LABEL maintainer="Dave Coleman dave@picknik.ai"

ENV TERM xterm

ENV ROS_WS /root/ws_moveit
ENV ROS_DISTRO crystal

WORKDIR $ROS_WS

ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

RUN \
     echo 'Etc/UTC' > /etc/timezone \
      && ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime \
      && apt-get update -qq && apt-get install -qq -y tzdata dirmngr gnupg2 lsb-release curl \
      # setup ros2 keys
      && curl http://repo.ros2.org/repos.key | apt-key add - \
      # setup sources.list
      && echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list \
      && apt-get update -qq && apt-get install -qq -y \
         build-essential \
         cmake \
         git \
         python3-colcon-common-extensions \
         python3-pip \
         python3-vcstool \
         wget \
      	 libboost-all-dev \
         libglew-dev \
         freeglut3-dev \
         pkg-config \
         libfcl-dev \
         libassimp-dev \
         libqhull-dev \
         libopencv-dev \
         clang clang-format-3.9 clang-tidy clang-tools \
         ros-$ROS_DISTRO-ros-base \
         ros-$ROS_DISTRO-action-msgs \
         ros-$ROS_DISTRO-message-filters \
         ros-$ROS_DISTRO-rclcpp-action \
         ros-$ROS_DISTRO-resource-retriever \
      && rm -rf /var/lib/apt/lists/* \
      && mkdir -p $ROS_WS/src \
      && wget https://raw.githubusercontent.com/AcutronicRobotics/moveit2/master/moveit2.repos \
      && vcs import src < moveit2.repos
COPY ./docker-entrypoint.sh /
ENV DOCKER 1
ENTRYPOINT ["/docker-entrypoint.sh"]
