#include <block_printer_moveit_interface/Waypoint.h>

using namespace block_printer_moveit_interface;

Waypoint::Waypoint() {}

Waypoint::Waypoint(InterpolationType type, const geometry_msgs::PoseStamped &poseStamped) : interpolation(type),
                                                                                            poseStamped(poseStamped) {}

Waypoint::Waypoint(const Waypoint &other) : interpolation(other.interpolation),
                                            poseStamped(other.poseStamped) {}

Waypoint &Waypoint::operator=(const Waypoint &other)
{
    this->interpolation = other.interpolation;
    this->poseStamped = other.poseStamped;
    return *this;
}

std::vector<WaypointGroup> block_printer_moveit_interface::splitWaypointGroupsByInterpolation(const std::vector<Waypoint> &waypoints)
{
    std::vector<WaypointGroup> value;
    WaypointGroup temp;
    if (waypoints.empty())
    {
        return value;
    }
    for (auto waypoint : waypoints)
    {
        auto interpolation = waypoint.interpolation;
        if (interpolation == InterpolationType::DYNAMIC)
        {
            WaypointGroup group(temp);
            value.push_back(group);
            temp.clear();
            temp.push_back(waypoint);
        }
        else
        {
            temp.push_back(waypoint);
        }
    }
    if (!temp.empty())
    {
        WaypointGroup group(temp);
        value.push_back(group);
    }
    return value;
}
