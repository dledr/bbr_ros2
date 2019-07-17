FROM osrf/ros2:nightly

# copy bbr_ros2 package repo
ENV BBR_WS /opt/bbr_ws
RUN mkdir -p $BBR_WS/src
WORKDIR $BBR_WS/src
COPY ./ bbr_ros2/

# install bbr_ros2 package dependencies
WORKDIR $BBR_WS
RUN apt-get update && \
    rosdep install -q -y \
      --from-paths \
        src \
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# build bbr_ros2 package source
# ARG CMAKE_BUILD_TYPE=Release
# RUN rm $BBR_WS/src/bbr_ros2/bbr_system_tests/COLCON_IGNORE
# ARG COVERAGE_ENABLED=False
# RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
#     colcon build \
#       --symlink-install \
#       --cmake-args \
#         --no-warn-unused-cli \
#         -DCMAKE_BUILD_TYPE=$CMAKE_BUILD_TYPE \
#         -DCOVERAGE_ENABLED=$COVERAGE_ENABLED \
#         -DCMAKE_CXX_FLAGS="-Wno-unused-parameter"
#
# # source bbr_ros2 workspace from entrypoint
# RUN sed --in-place \
#       's|^source .*|source "$BBR_WS/install/setup.bash"|' \
#       /ros_entrypoint.sh
