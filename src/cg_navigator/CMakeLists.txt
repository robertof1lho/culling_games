cmake_minimum_required(VERSION 3.8)
project(cg_navigator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependências do ROS2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cg_interfaces REQUIRED)

include_directories(
  include
)

add_executable(navigator_node
  src/navigator_node.cpp
)

ament_target_dependencies(navigator_node
  rclcpp
  cg_interfaces
)

add_executable(mapper_node
  src/mapper_node.cpp
)

ament_target_dependencies(mapper_node
  rclcpp
  cg_interfaces
)

install(TARGETS
  navigator_node
  mapper_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
