cmake_minimum_required(VERSION 3.8)
project(custom_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################
set(msg_files
  # "msg/Demo.msg"
)

set(srv_files
  "srv/Addition.srv"
)

set(action_files
  # "action/Demo.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES action_msgs std_msgs
  ADD_LINTER_TESTS
)
ament_export_dependencies(rosidl_default_runtime)


ament_package()
