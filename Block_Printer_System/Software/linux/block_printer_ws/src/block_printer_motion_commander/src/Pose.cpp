#include <block_printer_motion_commander/Pose.h>

using namespace block_printer_motion_commander;

Pose::Pose() {}

Pose::Pose(const std::vector<float> &v)
{
    position.x = v.at(0);
    position.y = v.at(1);
    position.z = v.at(2);
    for (int i = 3; i < v.size(); i++)
    {
        orientation.push_back(v[i]);
    }
}

Pose::Pose(const std::vector<double> &_position, const std::vector<double> &orientation) : orientation(orientation)
{
    position.x = _position.at(0);
    position.y = _position.at(1);
    position.z = _position.at(2);
}

Pose::Pose(const geometry_msgs::Point &position, const std::vector<double> &orientation) : position(position), orientation(orientation) {}

Pose::Pose(const block_printer_pick::GetPickPoseResponse &response) : position(response.position), orientation(response.orientation) {}

Pose::Pose(const block_printer_place::GetPlacePoseResponse &response) : position(response.position), orientation(response.orientation) {}

Pose::Pose(const Pose &other) : position(other.position), orientation(other.orientation) {}

Pose &Pose::operator=(const Pose &other)
{
    this->position = other.position;
    this->orientation = other.orientation;
    return *this;
}