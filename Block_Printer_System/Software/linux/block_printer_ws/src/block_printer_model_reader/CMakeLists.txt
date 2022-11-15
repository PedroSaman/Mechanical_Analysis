cmake_minimum_required(VERSION 2.8.3)
project(block_printer_model_reader)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  message_runtime
)

add_service_files(
  FILES
  GetComponentBlock.srv
  GetBlockState.srv
)

generate_messages(
  DEPENDENCIES
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
  src/block_printer_model_reader_node.cpp
  src/Point2.cpp
  src/Point3.cpp
  src/Block.cpp
  src/BlockColor.cpp
  src/ComponentBlock.cpp
  src/BlockModel.cpp
)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
