cmake_minimum_required(VERSION 3.5)
project(rosbag2_storage_checkpoint_plugin)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosbag2_storage_default_plugins REQUIRED)
find_package(sqlite3_vendor REQUIRED)
find_package(SQLite3 REQUIRED)  # provided by sqlite3_vendor
find_package(std_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/rosbag2_storage_checkpoint_plugin/checkpoint/checkpoint_node.cpp
  src/rosbag2_storage_checkpoint_plugin/checkpoint/checkpoint_storage.cpp)

ament_target_dependencies(${PROJECT_NAME}
  rosbag2_storage
  rosbag2_storage_default_plugins
  rclcpp
  rcutils
  SQLite3
  pluginlib
  std_msgs)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE
ROSBAG2_STORAGE_CHECKPOINT_PLUGINS_BUILDING_DLL)

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
ament_export_dependencies(rosbag2_storage rcutils sqlite3_vendor SQLite3)

ament_package()
