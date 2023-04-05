cmake_minimum_required(VERSION 3.8)
project(environment_builder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(environment_interface REQUIRED)

add_executable(block_services src/block_services.cpp)
ament_target_dependencies(
  block_services
  "environment_interface"
  "moveit_ros_planning_interface"
  "rclcpp"
)

add_executable(block_cleaner src/block_cleaner.cpp)
ament_target_dependencies(
  block_cleaner
  "environment_interface"
  "moveit_ros_planning_interface"
  "rclcpp"
)

add_executable(dummy_client src/dummy_client.cpp)
ament_target_dependencies(
  dummy_client
  "rclcpp"
  "environment_interface"
)

install(TARGETS
block_services
dummy_client
block_cleaner
DESTINATION lib/${PROJECT_NAME})

ament_package()
