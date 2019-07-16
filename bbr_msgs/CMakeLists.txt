cmake_minimum_required(VERSION 3.5)
project(bbr_msgs)

find_package(ament_cmake REQUIRED)
find_package(bbr_common REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

bbr_package()

set(msg_files
  "msg/Checkpoint.msg"
  "msg/CheckpointArray.msg"
  "msg/Record.msg"
  "msg/RecordArray.msg"
)
set(srv_files
  "srv/CreateRecords.srv"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
