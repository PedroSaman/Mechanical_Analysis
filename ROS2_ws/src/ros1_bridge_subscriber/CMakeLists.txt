cmake_minimum_required(VERSION 3.8)
project(ros1_bridge_subscriber)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(control_msgs REQUIRED)

add_executable(subscriber src/subscriber.cpp)
ament_target_dependencies(subscriber
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "moveit_ros_planning_interface"  
)
add_executable(commander src/commander.cpp)
ament_target_dependencies(commander 
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "moveit_ros_planning_interface"  
  "control_msgs"
)
add_executable(joint_commands src/joint_commands.cpp)
ament_target_dependencies(joint_commands 
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "moveit_ros_planning_interface"  
  "std_msgs"
)
include_directories(
  include
  ${control_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
)

install(TARGETS
  subscriber
  commander
  joint_commands
  DESTINATION lib/${PROJECT_NAME})

ament_package()

