cmake_minimum_required(VERSION 2.8.3)
project(denso_bcap_controller)

add_compile_options(-std=c++11)
add_compile_options(-fpermissive)
add_compile_options(-Wwrite-strings)

find_package(catkin REQUIRED COMPONENTS
  urdf
  actionlib
  sensor_msgs
  trajectory_msgs
  control_msgs
  denso_bcap
  robot_movement_interface
  roscpp
  trajectory_reducer
  message_generation
  block_printer_motion_commander
  block_printer_moveit_interface
)

add_service_files(
  FILES
  FollowJointTrajectoryInterface.srv
  GripperCommandInterface.srv
  CommandsActionInterface.srv
)

generate_messages(
  DEPENDENCIES
  trajectory_msgs
  control_msgs
  robot_movement_interface
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
  src/${PROJECT_NAME}_node.cpp
  src/function.cpp
  src/Object.cpp
  src/RcController.cpp
  src/Task.cpp
  src/Int32.cpp
  src/Float32.cpp
  src/RadianJoint.cpp
  src/Manipulator.cpp
  src/Hand.cpp
)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

add_executable(denso_bcap_controller_interface
  src/denso_bcap_controller_interface.cpp
)

add_dependencies(denso_bcap_controller_interface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(denso_bcap_controller_interface ${catkin_LIBRARIES})
