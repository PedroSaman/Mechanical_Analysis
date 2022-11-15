cmake_minimum_required(VERSION 2.8.3)
project(block_printer_motion_commander)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  actionlib
  block_printer_model_reader
  block_printer_pick
  block_printer_place
  robot_movement_interface
  roscpp
)

catkin_package(INCLUDE_DIRS include)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/block_printer_motion_commander_node.cpp src/Pose.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

add_executable(interpolator src/motion_interpolator.cpp src/Pose.cpp)

add_dependencies(interpolator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(interpolator
  ${catkin_LIBRARIES}
)
