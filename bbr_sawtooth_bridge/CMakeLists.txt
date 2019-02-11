cmake_minimum_required(VERSION 3.5)
project(bbr_sawtooth_bridge)

find_package(ament_cmake REQUIRED)
find_package(bbr_common REQUIRED)
find_package(bbr_msgs REQUIRED)
find_package(Poco COMPONENTS Crypto)
find_package(poco_vendor REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)

bbr_package()

add_library(${PROJECT_NAME} SHARED
  src/bbr_sawtooth_bridge/main.cpp)

add_executable(bridge_cpp
  src/bbr_sawtooth_bridge/main.cpp)

ament_target_dependencies(${PROJECT_NAME}
  bbr_msgs
  Poco
  rclcpp
  rcutils
  std_msgs)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(
  DIRECTORY include/
  DESTINATION include)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS bridge_cpp
  DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  bbr_msgs
  Poco
  poco_vendor
  rclcpp
  rcutils
  std_msgs)

ament_target_dependencies(bridge_cpp
  bbr_msgs
  Poco
  poco_vendor
  rclcpp
  rcutils
  std_msgs)

ament_package()
