#pragma once
#include <control_msgs/GripperCommandAction.h>
#include <actionlib/server/simple_action_server.h>
#include <denso_bcap_controller/Float32.h>
#include <denso_bcap_controller/Task.h>

namespace denso_bcap_controller
{
class Hand : public Object
{
public:
  Hand(RcController &controller, bool useHand);
  virtual ~Hand();
  bool executeCommand(const control_msgs::GripperCommand &command);

private:
  Float32 targetPositionVariable;
  Task initTask;
  Task moveTask;
};
} // namespace denso_bcap_controller