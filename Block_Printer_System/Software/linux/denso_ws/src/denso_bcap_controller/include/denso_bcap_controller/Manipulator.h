#pragma once
#include <ros/publisher.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_reducer/ReduceTrajectory.h>
#include <denso_bcap_controller/Int32.h>
#include <denso_bcap_controller/Float32.h>
#include <denso_bcap_controller/RadianJoint.h>
#include <denso_bcap_controller/Task.h>

namespace denso_bcap_controller
{

class Manipulator : public Object
{
public:
  Manipulator(RcController &controller, const std::string &robotDescriptionName);
  virtual ~Manipulator();
  BCAP_HRESULT setSlaveMode(bool slave);
  std::pair<sensor_msgs::JointState, BCAP_HRESULT> getJointState();
  BCAP_HRESULT setPreciseGoal(bool enablePreciseGoal);
  BCAP_HRESULT setVelocityFactor(double factor);
  BCAP_HRESULT setForceLimitPercents(double x, double y, double z, double rx, double ry, double rz);
  bool executeAction(const trajectory_msgs::JointTrajectory &trajectory, const ros::Publisher &jointStatePublisher, bool enableTrajectoryFilter);

private:
  std::string rootLinkName;
  //
  BCapHandle robotHandle;
  BCapHandle angleHandle;
  bool isSlaveMode;
  bool isForceLimited;
  Int32 curveNumVariable;
  Int32 preciseGoalVariable;
  Float32 speedVariable;
  std::vector<Float32> forceLimitVariables;
  std::vector<RadianJoint> jointTrajectoryVariables;
  Task jointTrajectoryTask;
  Task slaveTask;
  bool executeActionSlave(const trajectory_msgs::JointTrajectory &trajectory);
  bool executeActionNormal(const trajectory_msgs::JointTrajectory &trajectory, const ros::Publisher &jointStatePublisher, bool enableTrajectoryFilter);
  bool getTrajectoryGroup(const trajectory_msgs::JointTrajectory &trajectory, std::vector<trajectory_msgs::JointTrajectory> &trajectoryGroup, bool enableTrajectoryFilter);
};
} // namespace denso_bcap_controller
