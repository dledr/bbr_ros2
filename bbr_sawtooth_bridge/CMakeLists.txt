cmake_minimum_required(VERSION 3.5)
project(bbr_sawtooth_bridge)

find_package(ament_cmake REQUIRED)
find_package(bbr_common REQUIRED)
find_package(bbr_msgs REQUIRED)
find_package(bbr_sawtooth_protobuf REQUIRED)
find_package(rclcpp REQUIRED)

bbr_package()

file(GLOB_RECURSE ALL_LIBRARY_HDRS "include/*.hpp")
file(GLOB_RECURSE ALL_LIBRARY_SRCS "src/*.cpp")
file(GLOB_RECURSE ALL_EXECUTABLES "src/*_main.cpp")
list(REMOVE_ITEM ALL_LIBRARY_SRCS ${ALL_EXECUTABLES})

list(APPEND ALL_LIBRARY_HDRS ${ALL_PROTO_HDRS})
list(APPEND ALL_LIBRARY_SRCS ${ALL_PROTO_SRCS})

set(dependencies
  bbr_msgs
  bbr_sawtooth_protobuf
  rclcpp
)

include_directories(
        include
)

set(executable_name sawtooth_bridge)
set(library_name ${executable_name}_core)

add_library(${library_name} SHARED
  ${ALL_LIBRARY_HDRS}
  ${ALL_LIBRARY_SRCS}
)
ament_target_dependencies(${library_name}
  ${dependencies}
)

add_executable(${executable_name}
  src/bbr_sawtooth_bridge/bridge_main.cpp
)
ament_target_dependencies(${executable_name}
  ${dependencies}
)
target_link_libraries(${executable_name}
  ${library_name}
)

install(TARGETS ${library_name} ${executable_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

ament_package()
