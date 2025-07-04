cmake_minimum_required(VERSION 3.8)
project(openarm_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(openarm_interfaces REQUIRED)

# Find socketcan_control library using environment variable or default path
if(DEFINED ENV{OPENARM_SOCKETCAN_LIB})
    set(OPENARM_SOCKETCAN_LIB $ENV{OPENARM_SOCKETCAN_LIB})
else()
    # Try common locations if environment variable not set
    set(OPENARM_SOCKETCAN_LIB /work/openarm_socketcan)
    if(NOT EXISTS ${OPENARM_SOCKETCAN_LIB})
        set(OPENARM_SOCKETCAN_LIB /home/tracker/openarm_socketcan)
    endif()
endif()

message(STATUS "Using OPENARM_SOCKETCAN_LIB: ${OPENARM_SOCKETCAN_LIB}")

# Set include directories
set(SOCKETCAN_CONTROL_INCLUDE_DIRS
    ${OPENARM_SOCKETCAN_LIB}/include
    ${OPENARM_SOCKETCAN_LIB}/include/canbus
    ${OPENARM_SOCKETCAN_LIB}/include/damiao_motor
    ${OPENARM_SOCKETCAN_LIB}/include/openarm_control
)

# Find the built openarm library
find_library(SOCKETCAN_CONTROL_LIBRARY
    NAMES openarm
    PATHS
        ${OPENARM_SOCKETCAN_LIB}/build
        ${OPENARM_SOCKETCAN_LIB}/build/lib
        ${OPENARM_SOCKETCAN_LIB}/lib
        ${OPENARM_SOCKETCAN_LIB}
    NO_DEFAULT_PATH
)

if(NOT SOCKETCAN_CONTROL_LIBRARY)
    message(FATAL_ERROR "Could not find openarm library in ${OPENARM_SOCKETCAN_LIB}")
endif()

message(STATUS "Found socketcan_control library: ${SOCKETCAN_CONTROL_LIBRARY}")

# Include directories
include_directories(include)
include_directories(${SOCKETCAN_CONTROL_INCLUDE_DIRS})

# Executable for standalone node
add_executable(openarm_socketcan_node
  src/openarm_node_main.cpp
)

ament_target_dependencies(openarm_socketcan_node
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  std_srvs
  openarm_interfaces
)

target_link_libraries(openarm_socketcan_node
  ${SOCKETCAN_CONTROL_LIBRARY}
  openarm_socketcan_components
)

# Composable node library
add_library(openarm_socketcan_components SHARED
  src/openarm_node.cpp
)

ament_target_dependencies(openarm_socketcan_components
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  std_srvs
  openarm_interfaces
)

target_link_libraries(openarm_socketcan_components
  ${SOCKETCAN_CONTROL_LIBRARY}
)

# Recompile socketcan_control library with -fPIC for shared library usage
set_target_properties(openarm_socketcan_components PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)

# Register component nodes
rclcpp_components_register_nodes(openarm_socketcan_components "openarm_socketcan_ros2::OpenArmNode")

# Install targets
install(TARGETS
  openarm_socketcan_node
  openarm_socketcan_components
  DESTINATION lib/${PROJECT_NAME}
)


# Install shared library
install(TARGETS
  openarm_socketcan_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install directories
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()


ament_package()
