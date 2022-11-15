#pragma once

#include <string>
#include <ros/console.h>

namespace denso_bcap_controller
{
static const std::string logHeader = "Denso B-CAP Controller";
}
#define PRINT_INFO(args) ROS_INFO_STREAM(denso_bcap_controller::logHeader << ": " << args)
#define PRINT_WARN(args) ROS_WARN_STREAM(denso_bcap_controller::logHeader << ": " << args)
#define PRINT_ERROR(args) ROS_ERROR_STREAM(denso_bcap_controller::logHeader << ": " << args)
