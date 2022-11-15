#pragma once
#include <vector>
#include <geometry_msgs/Point.h>
#include <block_printer_pick/GetPickPoseResponse.h>
#include <block_printer_place/GetPlacePoseResponse.h>

namespace block_printer_motion_commander
{
class Pose
{
public:
  geometry_msgs::Point position;
  std::vector<double> orientation;
  Pose();
  Pose(const std::vector<float> &v);
  Pose(const std::vector<double> &position, const std::vector<double> &orientation);
  Pose(const geometry_msgs::Point &position, const std::vector<double> &orientation);
  Pose(const block_printer_pick::GetPickPoseResponse &response);
  Pose(const block_printer_place::GetPlacePoseResponse &response);
  Pose(const Pose &other);
  template <typename T>
  void toVector(std::vector<T> &v) const
  {
    v.reserve(3 + this->orientation.size());
    v.push_back((T)this->position.x);
    v.push_back((T)this->position.y);
    v.push_back((T)this->position.z);
    for (auto e : this->orientation)
    {
      v.push_back((T)e);
    }
  }
  Pose &operator=(const Pose &other);
};
} // namespace block_printer_motion_commander