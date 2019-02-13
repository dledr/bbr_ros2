cmake_minimum_required(VERSION 3.5)
project(bbr_sawtooth_bridge)

find_package(ament_cmake REQUIRED)
find_package(bbr_common REQUIRED)
find_package(bbr_msgs REQUIRED)
find_package(bbr_sawtooth_protobuf REQUIRED)
find_package(Poco COMPONENTS Crypto)
find_package(poco_vendor REQUIRED)
find_package(rclcpp REQUIRED)
find_library(SECP256k1_LIBRARY libsecp256k1.so)


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
  Poco
  poco_vendor
  rclcpp
)

include_directories(
        include
)

set(library_name ${PROJECT_NAME})
add_library(${library_name} SHARED
  ${ALL_LIBRARY_HDRS}
  ${ALL_LIBRARY_SRCS}
)
ament_target_dependencies(${library_name}
  ${dependencies}
  ${SECP256k1_LIBRARY}
)

set(executable_name bridge_cpp)
add_executable(${executable_name}
  src/bbr_sawtooth_bridge/bridge_main.cpp
)
ament_target_dependencies(${executable_name}
  ${dependencies}
)
target_link_libraries(${executable_name}
  ${library_name}
  ${SECP256k1_LIBRARY}
)

add_executable(demo_cpp
  src/bbr_sawtooth_bridge/demo_main.cpp
)
ament_target_dependencies(demo_cpp
  ${dependencies}
)
target_link_libraries(demo_cpp
  ${library_name}
  ${SECP256k1_LIBRARY}
)

install(TARGETS ${library_name} ${executable_name} demo_cpp
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
