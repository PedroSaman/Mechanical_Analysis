#pragma once

#include <vector>
#include <geometry_msgs/PoseStamped.h>

namespace block_printer_moveit_interface
{
enum InterpolationType
{
    DYNAMIC,
    CARTESIAN,
};

class Waypoint
{
  public:
    InterpolationType interpolation;
    geometry_msgs::PoseStamped poseStamped;
    Waypoint();
    Waypoint(InterpolationType type, const geometry_msgs::PoseStamped &poseStamped);
    Waypoint(const Waypoint &other);
    Waypoint &operator=(const Waypoint &other);
};

typedef std::vector<Waypoint> WaypointGroup;

std::vector<WaypointGroup> splitWaypointGroupsByInterpolation(const std::vector<Waypoint> &waypoints);

} // namespace block_printer_moveit_interface