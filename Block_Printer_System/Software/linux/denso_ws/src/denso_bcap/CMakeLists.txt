cmake_minimum_required(VERSION 2.8.3)
project(denso_bcap)

add_compile_options(-std=c++11)
add_compile_options(-fpermissive)
add_compile_options(-Wwrite-strings)

find_package(catkin REQUIRED COMPONENTS roscpp)

find_package(Boost REQUIRED COMPONENTS thread system)

catkin_package(
  DEPENDS boost
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/bcap_base.cpp
  src/bcap_net.cpp
  src/bcap_serial.cpp
  src/crc16.cpp           
)
