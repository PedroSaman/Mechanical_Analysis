#pragma once

#include <ros/duration.h>
#include <ros/node_handle.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <block_printer_moveit_interface/Waypoint.h>

namespace block_printer_moveit_interface
{

class MoveitPlanner
{
public:
  MoveitPlanner(ros::NodeHandle &node,
                const std::string &moveGroupName,
                const std::string &referenceFrame = "",
                const std::string &endEffectorLink = "");
  void addWaypoint(InterpolationType interpolationType, const geometry_msgs::Pose &pose);
  void clearWaypoints();
  const moveit::planning_interface::MoveGroupInterface &getMoveGroup() const;
  geometry_msgs::PoseStamped getCurrentPose();
  moveit::planning_interface::MoveItErrorCode move(double maxVelocityFactor,
                                                   double maxAccelerationFactor,
                                                   double cartesianMovePointStep,
                                                   double cartesianMoveJumpThreshold,
                                                   ros::Duration &planDuration,
                                                   ros::Duration &executeDuration);

private:
  moveit::planning_interface::MoveGroupInterface moveGroup;
  ros::ServiceClient fkServiceClient;
  ros::ServiceClient ikServiceClient;
  std::vector<Waypoint> waypoints;
  bool toManipulatorJoint(const geometry_msgs::PoseStamped &poseStamped, sensor_msgs::JointState &jointState);
  bool toPose(const std::vector<double> &joints, geometry_msgs::PoseStamped &pose);
  moveit::planning_interface::MoveItErrorCode planDynamicPath(const Waypoint &waypoint,
                                                              const moveit_msgs::RobotState &startState,
                                                              double maxVelocityFactor,
                                                              double maxAccelerationFactor,
                                                              moveit::planning_interface::MoveGroupInterface::Plan &plan);
  moveit::planning_interface::MoveItErrorCode planCartesianPath(const WaypointGroup &group,
                                                                const moveit_msgs::RobotState &startState,
                                                                double maxVelocityFactor,
                                                                double maxAccelerationFactor,
                                                                double pointStep,
                                                                double jumpThreshold,
                                                                moveit::planning_interface::MoveGroupInterface::Plan &plan);
  void appendPlan(moveit::planning_interface::MoveGroupInterface::Plan &previous,
                  const moveit::planning_interface::MoveGroupInterface::Plan &next,
                  moveit_msgs::RobotState &nextStartState) const;
  void applyVelocityAndAccelerationFactors(trajectory_msgs::JointTrajectory &trajectory, double maxVelocityFactor, double maxAccelerationFactor);
};
} // namespace block_printer_moveit_interface
