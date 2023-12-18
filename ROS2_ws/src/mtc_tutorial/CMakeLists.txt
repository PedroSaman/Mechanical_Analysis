cmake_minimum_required(VERSION 3.8)
project(mtc_tutorial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(environment_interface REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)

add_executable(task_creator_action_client src/task_creator_action_client.cpp)
  ament_target_dependencies(task_creator_action_client
  "rclcpp"
  "rclcpp_action"
  "environment_interface"
  "rclcpp_components"
  "moveit_task_constructor_core"
  "moveit_ros_planning_interface"
)

add_executable(task_creator_action_server src/task_creator_action_server.cpp)
ament_target_dependencies(task_creator_action_server
  "rclcpp"
  "rclcpp_action"
  "environment_interface"
  "rclcpp_components"
  "moveit_task_constructor_core"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS 
  task_creator_action_server
  task_creator_action_client
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
