#pragma once
#include <string>
#include <ros/console.h>

namespace block_printer_model_reader
{
static const std::string nodeName = "model_reader";
#define PRINT_INFO(args) ROS_INFO_STREAM(nodeName << ": " << args)
#define PRINT_WARN(args) ROS_WARN_STREAM(nodeName << ": " << args)
#define PRINT_ERROR(args) ROS_ERROR_STREAM(nodeName << ": " << args)
} // namespace block_printer_model_reader