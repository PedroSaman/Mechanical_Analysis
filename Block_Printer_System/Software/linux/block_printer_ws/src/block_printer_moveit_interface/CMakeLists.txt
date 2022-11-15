cmake_minimum_required(VERSION 2.8.3)
project(block_printer_moveit_interface)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  block_printer_motion_commander
  block_printer_visualizer
  actionlib
  control_msgs
  moveit_commander
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
  moveit_visual_tools
  roscpp
  robot_movement_interface
)

catkin_package(INCLUDE_DIRS include)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(moveit_robot_movement_interface
  src/moveit_robot_movement_interface.cpp
  src/MoveitPlanner.cpp
  src/Waypoint.cpp
  src/ForceConverter.cpp
)

add_dependencies(moveit_robot_movement_interface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(moveit_robot_movement_interface
  ${catkin_LIBRARIES}
)
