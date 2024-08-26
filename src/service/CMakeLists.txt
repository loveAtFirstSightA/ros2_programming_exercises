cmake_minimum_required(VERSION 3.8)
project(service)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_msgs REQUIRED)

include_directories(include)

set(dependencies
  rclcpp
  custom_msgs
)

add_executable(server_a src/server_a.cpp)
target_include_directories(server_a PRIVATE include)
ament_target_dependencies(server_a ${dependencies})

add_executable(client_a src/client_a.cpp)
target_include_directories(client_a PRIVATE include)
ament_target_dependencies(client_a ${dependencies})

add_executable(client_b src/client_b.cpp)
target_include_directories(client_b PRIVATE include)
ament_target_dependencies(client_b ${dependencies})

add_executable(client_c src/client_c.cpp)
target_include_directories(client_c PRIVATE include)
ament_target_dependencies(client_c ${dependencies})

add_executable(client_d src/client_d.cpp)
target_include_directories(client_d PRIVATE include)
ament_target_dependencies(client_d ${dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS server_a
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS client_a
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS client_b
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS client_c
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS client_d
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
