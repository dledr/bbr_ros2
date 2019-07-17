# bbr_ros2
Black Block Recorder with ROS2 | Immutable Logging via Blockchain

* [![CircleCI](https://img.shields.io/circleci/build/github/dledr/bbr_ros2.svg?label=CircleCI)](https://circleci.com/gh/dledr/bbr_ros2/tree/master)
* [![DockerHub](https://img.shields.io/docker/cloud/build/dledr/bbr_ros2.svg?label=DockerHub)](https://hub.docker.com/r/dledr/bbr_ros2)
* [![Codecov](https://img.shields.io/codecov/c/github/dledr/bbr_ros2.svg?label=Codecov)](https://codecov.io/gh/dledr/bbr_ros2)
* [![License](https://img.shields.io/github/license/dledr/bbr_ros2.svg?label=License)](https://github.com/dledr/bbr_ros2/blob/master/LICENSE)
* [![Release](https://img.shields.io/github/release/dledr/bbr_ros2.svg?label=Release)](https://github.com/dledr/bbr_ros2/releases)

> Start the bbr_sawtooth_bridge, needed by rosbag2 when using BBR

```
ros2 run bbr_sawtooth_bridge bridge_cpp
```

> Start recording with rosbag2 using BBR storage plugin

```
ros2 bag record -o foo -s bbr /chatter
```

> Publish message data to recorded topic

```
ros2 run demo_nodes_cpp talker
```

> Examine bag file using an sqlite browser

```
sqlitebrowser foo/foo.db3
```
