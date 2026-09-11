cmake_minimum_required(VERSION 3.20)

add_library(agi3sdk SHARED
  src/aimrt_transport.cpp
  src/c_api.cpp
  src/robot_io.cpp
  src/state_assembler.cpp
)
add_library(agi3sdk::agi3sdk ALIAS agi3sdk)
target_compile_features(agi3sdk PUBLIC cxx_std_20)
target_include_directories(agi3sdk PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

option(AGI3SDK_WITH_AIMRT "Build the A3 AimRT/Iceoryx transport" OFF)
if(AGI3SDK_WITH_AIMRT)
  if(NOT AGI3_AIMRT_SOURCE_DIR OR NOT EXISTS "${AGI3_AIMRT_SOURCE_DIR}/CMakeLists.txt")
    message(FATAL_ERROR "AGI3SDK_WITH_AIMRT requires AGI3_AIMRT_SOURCE_DIR")
  endif()

  set(AIMRT_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(AIMRT_BUILD_DOCUMENT OFF CACHE BOOL "" FORCE)
  set(AIMRT_BUILD_RUNTIME ON CACHE BOOL "" FORCE)
  set(AIMRT_BUILD_CLI_TOOLS OFF CACHE BOOL "" FORCE)
  set(AIMRT_BUILD_WITH_PROTOBUF OFF CACHE BOOL "" FORCE)
  set(AIMRT_BUILD_WITH_ROS2 ON CACHE BOOL "" FORCE)
  set(AIMRT_BUILD_PROTOCOLS OFF CACHE BOOL "" FORCE)
  set(AIMRT_BUILD_WITH_WERROR OFF CACHE BOOL "" FORCE)
  set(AIMRT_INSTALL OFF CACHE BOOL "" FORCE)
  set(AIMRT_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(AIMRT_BUILD_ICEORYX_PLUGIN OFF CACHE BOOL "" FORCE)
  set(AIMRT_BUILD_ROS2_PLUGIN OFF CACHE BOOL "" FORCE)
  add_subdirectory("${AGI3_AIMRT_SOURCE_DIR}" "${CMAKE_BINARY_DIR}/aimrt" EXCLUDE_FROM_ALL)

  find_package(joint_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  target_compile_definitions(agi3sdk PRIVATE AGI3SDK_WITH_AIMRT=1)
  target_link_libraries(agi3sdk PRIVATE
    aimrt::interface::aimrt_module_cpp_interface
    aimrt::interface::aimrt_module_ros2_interface
    aimrt::runtime::core
    joint_msgs::joint_msgs__rosidl_generator_cpp
    joint_msgs::joint_msgs__rosidl_typesupport_cpp
    joint_msgs::joint_msgs__rosidl_typesupport_fastrtps_cpp
    sensor_msgs::sensor_msgs__rosidl_generator_cpp
    sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
    sensor_msgs::sensor_msgs__rosidl_typesupport_fastrtps_cpp
  )
  set_target_properties(agi3sdk PROPERTIES
    BUILD_RPATH "/opt/ros/jazzy/lib;/opt/agibot/share/ros2_package/aima_protocol_ros2_package/lib"
    INSTALL_RPATH "/opt/ros/jazzy/lib;/opt/agibot/share/ros2_package/aima_protocol_ros2_package/lib"
  )
endif()
if(BUILD_TESTING)
  add_executable(agi3sdk_test tests/robot_io_test.cpp)
  target_link_libraries(agi3sdk_test PRIVATE agi3sdk::agi3sdk)
  add_test(NAME agi3sdk COMMAND agi3sdk_test)
endif()

set_target_properties(agi3sdk PROPERTIES VERSION 0.1.0 SOVERSION 0)
install(TARGETS agi3sdk EXPORT agi3sdkTargets LIBRARY DESTINATION lib)
install(EXPORT agi3sdkTargets NAMESPACE agi3sdk:: DESTINATION lib/cmake/agi3sdk)
include(CMakePackageConfigHelpers)
configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/agi3sdkConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/agi3sdkConfig.cmake"
  INSTALL_DESTINATION lib/cmake/agi3sdk
)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/agi3sdkConfigVersion.cmake"
  VERSION 0.1.0 COMPATIBILITY SameMajorVersion
)
install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/agi3sdkConfig.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/agi3sdkConfigVersion.cmake"
  DESTINATION lib/cmake/agi3sdk
)
install(DIRECTORY include/ DESTINATION include)
