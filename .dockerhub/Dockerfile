FROM ros:dashing

# copy all package.xml
ENV BBR_WS /opt/bbr_ws
RUN mkdir -p $BBR_WS/src
WORKDIR $BBR_WS/src/bbr_ros2
COPY ./bbr_common/package.xml bbr_common/
COPY ./bbr_msgs/package.xml bbr_msgs/
COPY ./bbr_rosbag2_storage_plugin/package.xml bbr_rosbag2_storage_plugin/
COPY ./bbr_sawtooth_bridge/package.xml bbr_sawtooth_bridge/
COPY ./bbr_protobuf/package.xml bbr_protobuf/

# install package dependencies
WORKDIR $BBR_WS
RUN apt-get update && apt-get install -y \
      ros-$ROS_DISTRO-ros2bag && \
    rosdep update && \
    rosdep install -q -y \
      --from-paths \
        src \
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# copy repo packages
COPY ./ src/bbr_ros2/

# build package source
ARG CMAKE_BUILD_TYPE=Release
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
      --symlink-install \
      --cmake-args \
        --no-warn-unused-cli \
        -DCMAKE_BUILD_TYPE=$CMAKE_BUILD_TYPE \
        -DCMAKE_CXX_FLAGS="-Wno-unused-parameter"

# source workspace from entrypoint
RUN sed --in-place --expression \
      '$isource "$BBR_WS/install/setup.bash"' \
      /ros_entrypoint.sh
