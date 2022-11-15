cmake_minimum_required(VERSION 2.8.3)
project(block_printer_visualizer)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  moveit_commander
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
  moveit_visual_tools
  shape_msgs
  geometry_msgs
  block_printer_model_reader
  block_printer_pick
  block_printer_place
)

add_message_files(
  FILES
  Object.msg
)

add_service_files(
  FILES
  SpawnObject.srv
  SpawnSceneFile.srv
  SpawnBlock.srv
  AttachBlock.srv
  DetachBlock.srv
  DeleteBlock.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  shape_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /usr/include/eigen3
)

add_executable(object_spawner src/object_spawner.cpp)

add_dependencies(object_spawner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(object_spawner
  ${catkin_LIBRARIES}
)

add_executable(scene_spawner src/scene_spawner.cpp)

add_dependencies(scene_spawner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(scene_spawner
  ${catkin_LIBRARIES}
)

add_executable(block_spawner src/block_spawner.cpp)

add_dependencies(block_spawner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(block_spawner
  ${catkin_LIBRARIES}
)
