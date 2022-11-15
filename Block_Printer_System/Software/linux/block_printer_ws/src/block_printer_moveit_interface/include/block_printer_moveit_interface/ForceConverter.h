#pragma once

#include <sensor_msgs/JointState.h>
#include <moveit/move_group_interface/move_group_interface.h>

namespace block_printer_moveit_interface
{

/*
負の数は，その方向の力・モーメントは制限しないことを表しているものとする
example
[x,y,z,rx,ry,rz]
[-1,-1,10,-1,-1,-1]
*/

extern bool getForceLimits(const std::string &forceLimitType,
                           const moveit::planning_interface::MoveGroupInterface &moveGroup,
                           const sensor_msgs::JointState &jointState,
                           const std::vector<double> &constraints,
                           std::vector<double> &forceFactors);
} // namespace block_printer_moveit_interface
