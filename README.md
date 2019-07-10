# bbr_ros2
Black Block Recorder with ROS2 | Immutable Logging via Blockchain

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
