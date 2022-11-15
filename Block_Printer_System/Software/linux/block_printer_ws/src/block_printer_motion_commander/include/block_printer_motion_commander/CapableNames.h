#pragma once
#include <string>

namespace block_printer_motion_commander
{
static const std::string MOTION_COMMAND_ACTION = "/block_printer/motion_commander/commands";
static const std::string MOTION_COMMAND_ACTION_INTERPOLATED = "/block_printer/motion_commander/commands_interpolated";
static const std::string MOVEIT_INTERPOLATION_COMMAND = "moveit_interpolation";

typedef std::string ProcessIndex;

static const ProcessIndex TO_INITIAL_POSE = "TO_INITIAL_POSE";
static const ProcessIndex TO_PICK_APPROACH = "TO_PICK_APPROACH";
static const ProcessIndex PICK_ADJUST_GRIPPER = "PICK_ADJUST_GRIPPER";
static const ProcessIndex TO_PICK = "TO_PICK";
static const ProcessIndex PICK_MOVE_GRIPPER = "PICK_MOVE_GRIPPER";
static const ProcessIndex TO_PLACE_APPROACH = "TO_PLACE_APPROACH";
static const ProcessIndex PLACE_START_FORCE_LIMIT = "PLACE_START_FORCE_LIMIT";
static const ProcessIndex TO_PLACE = "TO_PLACE";
static const ProcessIndex PLACE_MOVE_GRIPPER = "PLACE_MOVE_GRIPPER";
static const ProcessIndex PICK_RECOVER_GRIPPER = "PICK_RECOVER_GRIPPER";
static const ProcessIndex PICK_RECOVER_DISAPPROACH = "PICK_RECOVER_DISAPPROACH";
static const ProcessIndex TO_FINISH_POSE = "TO_FINISH_POSE";

} // namespace block_printer_motion_commander