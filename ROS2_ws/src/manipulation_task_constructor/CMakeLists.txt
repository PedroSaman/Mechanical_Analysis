cmake_minimum_required(VERSION 3.8)
project(manipulation_task_constructor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(environment_interface REQUIRED)

add_executable(manipulation_task_constructor src/manipulation_task_constructor.cpp)
ament_target_dependencies(
  manipulation_task_constructor 
  "moveit_task_constructor_core"
  "moveit_ros_planning_interface"
  environment_interface
  "rclcpp"
)
target_include_directories(manipulation_task_constructor PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
target_compile_features(manipulation_task_constructor PUBLIC c_std_99 cxx_std_17)  

add_executable(world_builder src/world_builder.cpp)
ament_target_dependencies(
  world_builder 
  "moveit_ros_planning_interface"
  "environment_interface"
  "rclcpp"
)
target_include_directories(world_builder PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
target_compile_features(world_builder PUBLIC c_std_99 cxx_std_17)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(
  DIRECTORY include/manipulation_task_constructor
  DESTINATION include
)

install(TARGETS manipulation_task_constructor world_builder
DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
find_package(ament_lint_auto REQUIRED)
# the following line skips the linter which checks for copyrights
# uncomment the line when a copyright and license is not present in all source files
#set(ament_cmake_copyright_FOUND TRUE)
# the following line skips cpplint (only works in a git repo)
# uncomment the line when this package is not in a git repo
#set(ament_cmake_cpplint_FOUND TRUE)
ament_lint_auto_find_test_dependencies()
endif()

ament_package()
