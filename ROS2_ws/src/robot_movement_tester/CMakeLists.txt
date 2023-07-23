cmake_minimum_required(VERSION 3.8)
project(robot_movement_tester)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(joint_states_reproducer src/joint_states_reproducer.cpp)
target_include_directories(joint_states_reproducer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(joint_states_reproducer PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  joint_states_reproducer
  "moveit_ros_planning_interface"
  "rclcpp"
)

add_executable(collision_avoid_pose_commander src/collision_avoid_pose_commander.cpp)
target_include_directories(collision_avoid_pose_commander PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(collision_avoid_pose_commander PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  collision_avoid_pose_commander 
  "moveit_ros_planning_interface"
  "std_msgs"
  "rclcpp"
)

install(TARGETS joint_states_reproducer collision_avoid_pose_commander
  DESTINATION lib/${PROJECT_NAME})

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

ament_package()
