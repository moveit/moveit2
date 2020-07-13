# This dockerfile can be configured via --build-arg
# Build context must be the /moveit2 root folder for COPY.
# Example build command:
# export UNDERLAY_MIXINS="debug ccache"
# export OVERLAY_MIXINS="debug ccache coverage"
# docker build -t moveit2:latest \
#   --build-arg UNDERLAY_MIXINS \
#   --build-arg OVERLAY_MIXINS ./
ARG FROM_IMAGE=osrf/ros2:nightly
ARG UNDERLAY_WS=/opt/underlay_ws
ARG OVERLAY_WS=/opt/overlay_ws

# multi-stage for caching
FROM $FROM_IMAGE AS cacher

# clone underlay source
ARG UNDERLAY_WS
WORKDIR $UNDERLAY_WS/src
COPY ./underlay.repos ../
RUN vcs import ./ < ../underlay.repos && \
    find ./ -name ".git" | xargs rm -rf

# copy overlay source
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS/src
COPY ./ ./moveit2

# copy manifests for caching
WORKDIR /opt
RUN mkdir -p /tmp/opt && \
    find ./ -name "package.xml" | \
      xargs cp --parents -t /tmp/opt && \
    find ./ -name "COLCON_IGNORE" | \
      xargs cp --parents -t /tmp/opt || true

# multi-stage for building
FROM $FROM_IMAGE AS builder

# install CI dependencies
RUN apt-get update && apt-get install -q -y \
      ccache \
      lcov \
    && rosdep update \
    && rm -rf /var/lib/apt/lists/*

# install underlay dependencies
ARG UNDERLAY_WS
WORKDIR $UNDERLAY_WS
COPY --from=cacher /tmp/$UNDERLAY_WS ./
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && rosdep install -q -y \
      --from-paths src \
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# build underlay source
COPY --from=cacher $UNDERLAY_WS ./
ARG UNDERLAY_MIXINS="release ccache"
ARG FAIL_ON_BUILD_FAILURE=True
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
      --symlink-install \
      --mixin $UNDERLAY_MIXINS \
      --event-handlers console_direct+ \
    || ([ -z "$FAIL_ON_BUILD_FAILURE" ] || exit 1)

# install overlay dependencies
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS
COPY --from=cacher /tmp/$OVERLAY_WS ./
RUN . $UNDERLAY_WS/install/setup.sh && \
    apt-get update && rosdep install -q -y \
      --from-paths src \
        $UNDERLAY_WS/src \
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# build overlay source
COPY --from=cacher $OVERLAY_WS ./
ARG OVERLAY_MIXINS="release ccache"
RUN . $UNDERLAY_WS/install/setup.sh && \
    colcon build \
      --symlink-install \
      --mixin $OVERLAY_MIXINS \
    || ([ -z "$FAIL_ON_BUILD_FAILURE" ] || exit 1)

# source overlay from entrypoint
ENV UNDERLAY_WS $UNDERLAY_WS
ENV OVERLAY_WS $OVERLAY_WS
RUN sed --in-place \
      's|^source .*|source "$OVERLAY_WS/install/setup.bash"|' \
      /ros_entrypoint.sh

# test overlay build
ARG RUN_TESTS
ARG FAIL_ON_TEST_FAILURE=Ture
RUN if [ -n "$RUN_TESTS" ]; then \
        . $OVERLAY_WS/install/setup.sh && \
        colcon test \
          --mixin $OVERLAY_MIXINS \
        && colcon test-result \
          || ([ -z "$FAIL_ON_TEST_FAILURE" ] || exit 1) \
    fi
