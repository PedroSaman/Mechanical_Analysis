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

add_executable(world_builder src/world_builder.cpp)
ament_target_dependencies(
  world_builder 
  "moveit_ros_planning_interface"
  "environment_interface"
  "rclcpp"
)

install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME})

install(TARGETS
block_services
dummy_client
block_cleaner
world_builder
DESTINATION lib/${PROJECT_NAME})

ament_package()
