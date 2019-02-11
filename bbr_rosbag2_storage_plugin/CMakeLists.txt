cmake_minimum_required(VERSION 3.5)
project(bbr_rosbag2_storage_plugin)

find_package(ament_cmake REQUIRED)
find_package(bbr_common REQUIRED)
find_package(bbr_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Poco COMPONENTS Crypto)
find_package(poco_vendor REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosbag2_storage_default_plugins REQUIRED)
find_package(SQLite3 REQUIRED)  # provided by sqlite3_vendor
find_package(sqlite3_vendor REQUIRED)
find_package(std_msgs REQUIRED)

bbr_package()

add_library(${PROJECT_NAME} SHARED
  src/bbr_rosbag2_storage_plugin/bbr/bbr_helper.cpp
  src/bbr_rosbag2_storage_plugin/bbr/bbr_node.cpp
  src/bbr_rosbag2_storage_plugin/bbr/bbr_storage.cpp)

ament_target_dependencies(${PROJECT_NAME}
  bbr_msgs
  pluginlib
  Poco
  rclcpp
  rcutils
  rosbag2_storage
  rosbag2_storage_default_plugins
  SQLite3
  std_msgs
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE
ROSBAG2_STORAGE_DEFAULT_PLUGINS_BUILDING_DLL)

pluginlib_export_plugin_description_file(rosbag2_storage plugin_description.xml)

install(
  DIRECTORY include/
  DESTINATION include)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  bbr_msgs
  Poco
  poco_vendor
  rclcpp
  rcutils
  rosbag2_storage
  rosbag2_storage_default_plugins
  SQLite3
  sqlite3_vendor
  std_msgs
)

ament_package()
