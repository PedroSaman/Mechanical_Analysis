cmake_minimum_required(VERSION 2.8.3)
project(block_printer_place)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  message_runtime
  roscpp
  block_printer_model_reader
)

add_service_files(
  FILES
  GetPlacePose.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
  src/block_printer_place_node.cpp
  src/ColumnVector.cpp
  src/Matrix.cpp
  src/SquareMatrix.cpp
  src/LeastSquarePlane.cpp
  src/WorkPad.cpp
)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
