FROM ros:crystal

# install build dependencies
RUN apt-get update && apt-get install -q -y \
      build-essential \
      cmake \
      git \
      python3-colcon-common-extensions \
      python3-vcstool \
      wget \
    && rm -rf /var/lib/apt/lists/*

# copy all package.xml
ENV ROS_WS /opt/ros_ws
RUN mkdir -p $ROS_WS/src
WORKDIR $ROS_WS
COPY ./bbr_msgs/package.xml src/bbr_ros2/bbr_msgs/
COPY ./bbr_rosbag2_storage_plugin/package.xml src/bbr_ros2/bbr_rosbag2_storage_plugin/
COPY ./bbr_sawtooth_bridge/package.xml src/bbr_ros2/bbr_sawtooth_bridge/

# install package dependencies
RUN apt-get update && apt-get install -y \
      ros-$ROS_DISTRO-ros2bag && \
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
        -DCMAKE_BUILD_TYPE=$CMAKE_BUILD_TYPE

# source workspace from entrypoint
RUN sed --in-place --expression \
      '$isource "$ROS_WS/install/setup.bash"' \
      /ros_entrypoint.sh
