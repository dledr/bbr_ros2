cmake_minimum_required(VERSION 3.5)
project(bbr_protobuf)

find_package(ament_cmake REQUIRED)
find_package(bbr_common REQUIRED)
find_package(Protobuf 3.0.0 REQUIRED)

bbr_package()

file(GLOB_RECURSE ALL_PROTOS "*.proto")
set(ALL_PROTO_SRCS)
set(ALL_PROTO_HDRS)
foreach(ABS_FIL ${ALL_PROTOS})
  file(RELATIVE_PATH
      REL_FIL
      ${PROJECT_SOURCE_DIR}
      ${ABS_FIL})
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  get_filename_component(FIL_WE ${REL_FIL} NAME_WE)

  list(APPEND ALL_PROTO_SRCS "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.cc")
  list(APPEND ALL_PROTO_HDRS "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h")
  file(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/${DIR})

  add_custom_command(
    OUTPUT "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.cc"
          "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h"
    COMMAND "${PROTOBUF_PROTOC_EXECUTABLE}"
            ARGS
            --cpp_out
            ${PROJECT_BINARY_DIR}/${DIR}
            --proto_path
            ${PROJECT_SOURCE_DIR}/${DIR}
            ${ABS_FIL}
    DEPENDS ${ABS_FIL}
    COMMENT "Running C++ protocol buffer compiler on ${ABS_FIL}"
    VERBATIM)
endforeach()

set_source_files_properties(${ALL_PROTO_SRCS}
                            ${ALL_PROTO_HDRS}
                            PROPERTIES
                            GENERATED
                            TRUE)

set(dependencies Protobuf)

set(library_name ${PROJECT_NAME})

include_directories(${PROJECT_BINARY_DIR})
add_library(${library_name} SHARED ${ALL_PROTO_HDRS} ${ALL_PROTO_SRCS})
ament_target_dependencies(${library_name} ${dependencies})

install(TARGETS ${library_name}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME})

foreach(HDR ${ALL_PROTO_HDRS})
  file(RELATIVE_PATH
      REL_FIL
      ${PROJECT_BINARY_DIR}
      ${HDR})
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  install(FILES ${HDR} DESTINATION include/${PROJECT_NAME}/${DIR})
endforeach()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

ament_package()
