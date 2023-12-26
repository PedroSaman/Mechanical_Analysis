cmake_minimum_required(VERSION 3.22)
project(scara_robot_description)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY urdf meshes launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
