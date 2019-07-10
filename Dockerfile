FROM ros:dashing

# install build dependencies
RUN apt-get update && apt-get install -q -y \
      build-essential \
      cmake \
      git \
      python3-colcon-common-extensions \
      python3-vcstool \
      wget \
    && rm -rf /var/lib/apt/lists/*

# # setup sawtooth keys
# RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 44FC67F19B2466EA
#
# # setup sources.list
# RUN echo "deb [arch=amd64] http://repo.sawtooth.me/ubuntu/nightly `lsb_release -sc` universe" > /etc/apt/sources.list.d/sawtooth-nightly.list
# # http://repo.sawtooth.me/ubuntu/1.0/stable
#
# # install sawtooth dependencies
# RUN apt-get update && apt-get install -q -y \
#       python3-sawtooth-sdk \
#       python3-sawtooth-signing \
#     && rm -rf /var/lib/apt/lists/*

# copy all package.xml
ENV ROS_WS /opt/ros_ws
RUN mkdir -p $ROS_WS/src
WORKDIR $ROS_WS/src/bbr_ros2
COPY ./bbr_common/package.xml bbr_common/
COPY ./bbr_msgs/package.xml bbr_msgs/
COPY ./bbr_rosbag2_storage_plugin/package.xml bbr_rosbag2_storage_plugin/
COPY ./bbr_sawtooth_bridge/package.xml bbr_sawtooth_bridge/
COPY ./bbr_protobuf/package.xml bbr_protobuf/

WORKDIR $ROS_WS
# install package dependencies
RUN apt-get update && apt-get install -y \
      ros-$ROS_DISTRO-ros2bag \
      libsecp256k1-dev && \
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
      '$isource "$ROS_WS/install/setup.bash"' \
      /ros_entrypoint.sh
