#pragma once

#include <string>

#define PRINT_INFO(args) ROS_INFO_STREAM(block_printer_moveit_interface::logHeader << ": " << args);
#define PRINT_WARN(args) ROS_WARN_STREAM(block_printer_moveit_interface::logHeader << ": " << args);
#define PRINT_ERROR(args) ROS_ERROR_STREAM(block_printer_moveit_interface::logHeader << ": " << args);

namespace block_printer_moveit_interface
{

static const std::string logHeader = "moveit_robot_movement_interface";

} // namespace block_printer_moveit_interface
