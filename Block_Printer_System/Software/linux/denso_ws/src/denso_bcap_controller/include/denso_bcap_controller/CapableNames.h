#pragma once
#include <string>

namespace denso_bcap_controller
{
const std::string INTERNAL_JOINT_STATE = "/vp6242/joint_states";
const std::string JOINT_TRAJECTORY_SERVICE = "/denso_bcap_controller/execute_joint_trajectory";
const std::string GRIPPER_SERVICE = "/denso_bcap_controller/execute_gripper_command";
const std::string COMMANDS_SERVICE = "/denso_bcap_controller/execute_commands_action";
} // namespace denso_bcap_controller
