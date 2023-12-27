#include "environment_interface/action/task_creator.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include "environment_interface/msg/block.hpp"
#include "environment_interface/srv/block_create.hpp"
#include "environment_interface/srv/block_remove.hpp"
#include "environment_interface/srv/get_block_color.hpp"
#include "environment_information.h"
#include <chrono>
#include <string>
#include <fstream>
#include <iostream>
#include <functional>
#include <memory>
#include <thread>

using namespace std::chrono_literals;
namespace mtc = moveit::task_constructor;


class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
  std::string doTask(environment_interface::msg::Block block, size_t operation_id, std::string robot_name, std::string planner_id);
  void shutdownNode(const std::string& reason = "User-requested shutdown");

private:
  // Compose an MTC task from a series of stages.
  std::string position_to_pickPlan(environment_interface::msg::Block block, std::string robot_name, std::string planner_id);
  std::string descend_to_pickPlan(std::string robot_name, std::string planner_id);
  std::string move_gripperPlan(environment_interface::msg::Block block, std::string command, std::string robot_name);
  std::string ascend_with_blockPlan(environment_interface::msg::Block block, std::string robot_name, std::string planner_id);
  std::string position_to_placePlan(environment_interface::msg::Block block, std::string robot_name, std::string planner_id);
  std::string descend_to_placePlan(std::string robot_name, std::string planner_id);
  std::string retreat_from_structurePlan(environment_interface::msg::Block block, std::string robot_name, std::string planner_id);
  mtc::Task go_homeTask(std::string robot_name);
  rclcpp::Node::SharedPtr node_;
};
MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options):node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
{
}
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

class TaskCreatorActionServer : public rclcpp::Node
{
public:
  std::shared_ptr<MTCTaskNode> mtc_node_;
  using TaskCreator = environment_interface::action::TaskCreator;
  using GoalHandleTaskCreator = rclcpp_action::ServerGoalHandle<TaskCreator>;
  std::unique_ptr<std::thread> spin_thread_;

  explicit TaskCreatorActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("task_creator_action_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<TaskCreator>(
      this,
      "task_creator",
      std::bind(&TaskCreatorActionServer::handle_goal, this, _1, _2),
      std::bind(&TaskCreatorActionServer::handle_cancel, this, _1),
      std::bind(&TaskCreatorActionServer::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<TaskCreator>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const TaskCreator::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->operation_id);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleTaskCreator> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleTaskCreator> goal_handle)
  {
    using namespace std::placeholders;
    RCLCPP_INFO(this->get_logger(), "handle accepted");
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&TaskCreatorActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleTaskCreator> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<TaskCreator::Result>();

    environment_interface::msg::Block block = goal->block;;
    size_t operation_id = goal->operation_id;
    std::string robot_name = goal->robot_name;
    std::string planner_id = goal->planner_id;

    result->task_resposnse = mtc_node_->doTask(block, operation_id, robot_name, planner_id);

    // Check if goal is done
    if (rclcpp::ok()) {
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }
};  // class TaskCreatorActionServer

std::string MTCTaskNode::doTask(environment_interface::msg::Block block, size_t operation_id, std::string robot_name, std::string planner_id)
{ 
  std::string operation_id_string;
  bool is_a_mtc_task = false;
  std::string plan_outout;
  mtc::Task task;
  
  switch (operation_id)
  {
  case POSITION_TO_PICK:
    operation_id_string = "POSITION_TO_PICK";
    plan_outout = position_to_pickPlan(block, robot_name, planner_id);
    break;
    
  case DESCEND_TO_PICK:
    plan_outout = descend_to_pickPlan(robot_name, planner_id);
    operation_id_string = "DESCEND_TO_PICK";
    break;
    
  case CLOSE_GRIPPER:
    plan_outout = move_gripperPlan(block, "CLOSE", robot_name);
    operation_id_string = "CLOSE_GRIPPER";
    break;
    
  case ASCEND_WITH_BLOCK:
    plan_outout = ascend_with_blockPlan(block, robot_name, planner_id);
    operation_id_string = "ASCEND_WITH_BLOCK";
    break;
    
  case POSITION_TO_PLACE:
    plan_outout = position_to_placePlan(block, robot_name, planner_id);
    operation_id_string = "POSITION_TO_PLACE";
    break;
    
  case DESCEND_TO_PLACE:
    plan_outout = descend_to_placePlan(robot_name, planner_id);
    operation_id_string = "DESCEND_TO_PLACE";
    break;
    
  case OPEN_GRIPPER:
    plan_outout = move_gripperPlan(block, "OPEN", robot_name);
    operation_id_string = "OPEN_GRIPPER";
    break;
    
  case RETREAT_FROM_STRUCTURE:
    plan_outout = retreat_from_structurePlan(block, robot_name, planner_id);
    operation_id_string = "RETREAT_FROM_STRUCTURE";
    break;
    
  case GO_HOME:
    task = go_homeTask(robot_name);
    is_a_mtc_task = true;
    operation_id_string = "GO_HOME";
    break;

  default:
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Opperation not defined.");
    operation_id_string = "OPERATION_UNKNOWN";
    plan_outout = "Operation is not implemented.";
    break;
  }
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "operation_id: %s",operation_id_string.c_str());

  if(is_a_mtc_task){ //If the plan was done using the MTC task constructor
    try
    {
      task.init();
    }
    catch (mtc::InitStageException& e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "task initialization error.");
      return "task initialization error.";
    }

    if (!task.plan())
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "task planning failed.");
      return "task planning failed.";
    }
    //task.introspection().publishSolution(*task.solutions().front());
    auto result = task.execute(*task.solutions().front());
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "task execution failed.");
      return "task execution failed.";
    }
  }else{ //If it is a simple move group plan
    if(plan_outout != "SUCCESS"){
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "plan error code: %s", plan_outout.c_str());
      return "Planning failed.";
    }
  }
  return "Everything should have went right, yahooo";
}

std::string MTCTaskNode::position_to_pickPlan(environment_interface::msg::Block block, std::string robot_name, std::string planner_id)
{
  const auto& arm_group_name = robot_name + "_arm";
  const auto& hand_frame = "gripper_tip";
  std::string object_name = "block_" + std::to_string(block.number);
  std::vector<std::string> object_vector;
  object_vector.insert(object_vector.end(),object_name);

  moveit::planning_interface::PlanningSceneInterface psi;
  std::map<std::string, geometry_msgs::msg::Pose> object_name_pose;
  object_name_pose = psi.getObjectPoses(object_vector);
  geometry_msgs::msg::Pose retrieved_pose = object_name_pose[object_name];
  //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Retrieved pose position x: %.2f y: %.2f z: %.2f\n",retrieved_pose.position.x,retrieved_pose.position.y,retrieved_pose.position.z);
  //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Retrieved pose orientation w: %.2f x: %.2f y: %.2f z: %.2f",retrieved_pose.orientation.w,retrieved_pose.orientation.x,retrieved_pose.orientation.y,retrieved_pose.orientation.z);

  auto move_group_interface = moveit::planning_interface::MoveGroupInterface(node_, arm_group_name);
  retrieved_pose.position.z += POSITION_TO_PICK_DISTANCE; //Stop at POSITION_TO_PICK_DISTANCE cm above the block
  retrieved_pose.position.x -= GRIPPER_POSITION_TO_PICK_CORRECTION; //Stop at POSITION_TO_PICK_DISTANCE cm above the block
  retrieved_pose.orientation.w = 0; //Correcting the orientation to pick
  retrieved_pose.orientation.y = 1;
  move_group_interface.setPoseTarget(retrieved_pose,hand_frame);
  move_group_interface.setPlannerId(planner_id);
  move_group_interface.setMaxVelocityScalingFactor(1.0);
  move_group_interface.setMaxAccelerationScalingFactor(1.0);
  // Create a plan to that target pose
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto const plan_output = static_cast<bool>(move_group_interface.plan(plan));

  // Execute the plan
  if(plan_output) {
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Planing failed!");
  }

  return moveit::core::errorCodeToString(plan_output);
}

std::string MTCTaskNode::descend_to_pickPlan(std::string robot_name, std::string planner_id)
{
  const auto& arm_group_name = robot_name + "_arm";
  const auto& hand_frame = "gripper_tip";

  auto move_group_interface = moveit::planning_interface::MoveGroupInterface(node_, arm_group_name);
  geometry_msgs::msg::PoseStamped gripper_tip_pose = move_group_interface.getCurrentPose(hand_frame);
  //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Retrieved pose position x: %.2f y: %.2f z: %.2f",gripper_tip_pose.pose.position.x,gripper_tip_pose.pose.position.y,gripper_tip_pose.pose.position.z);
  //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Retrieved pose orientation w: %.2f x: %.2f y: %.2f z: %.2f",gripper_tip_pose.pose.orientation.w,gripper_tip_pose.pose.orientation.x,gripper_tip_pose.pose.orientation.y,gripper_tip_pose.pose.orientation.z);
  gripper_tip_pose.pose.position.z -= (POSITION_TO_PICK_DISTANCE - block_size_z/2 - minimum_resolution); //Stop at POSITION_TO_PICK_DISTANCE cm above the block 22.12;//
  //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "New z position: %.2f",gripper_tip_pose.pose.position.z);
  move_group_interface.setPoseTarget(gripper_tip_pose.pose,hand_frame);
  move_group_interface.setPlannerId(planner_id);
  move_group_interface.setMaxVelocityScalingFactor(1.0);
  move_group_interface.setMaxAccelerationScalingFactor(1.0);
  // Create a plan to that target pose
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto const plan_output = static_cast<bool>(move_group_interface.plan(plan));

  // Execute the plan
  if(plan_output) {
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Planing failed!");
  }
  return moveit::core::errorCodeToString(plan_output);
}

std::string MTCTaskNode::move_gripperPlan(environment_interface::msg::Block block, std::string command, std::string robot_name)
{
  // Gripper full closed is 1.882 cm 
  const auto& hand_group_name = robot_name + "_hand";
  float gripper_joint_position;
  if(command == "CLOSE")
  {
    gripper_joint_position = gripper_max_opening - (block_size_x*block.x_size)/2 + KNOB_DISTANCE_TO_GRASP;
  }else
  {
    gripper_joint_position = 0.0;
  }

  std::vector<double> group_variable_values;
  group_variable_values.insert(group_variable_values.end(),-gripper_joint_position);
  auto move_group_interface = moveit::planning_interface::MoveGroupInterface(node_, hand_group_name);
  std::map<std::string, double> joint_values;
  joint_values["joint_gripper"] = gripper_joint_position;
  
  move_group_interface.setJointValueTarget(joint_values);
  move_group_interface.setMaxVelocityScalingFactor(0.01);
  move_group_interface.setMaxAccelerationScalingFactor(0.01);
  // Create a plan to that target pose
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto const plan_output = static_cast<bool>(move_group_interface.plan(plan));

  // Execute the plan
  if(plan_output) {
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Planing failed!");
  }

  return moveit::core::errorCodeToString(plan_output);
}

std::string MTCTaskNode::ascend_with_blockPlan(environment_interface::msg::Block block, std::string robot_name, std::string planner_id)
{
  const auto& arm_group_name = robot_name + "_arm";
  const auto& hand_frame = "gripper_tip";
  std::string object_name = "block_" + std::to_string(block.number);
  std::vector<std::string> object_vector;
  object_vector.insert(object_vector.end(),object_name);

  auto move_group_interface = moveit::planning_interface::MoveGroupInterface(node_, arm_group_name);
  move_group_interface.attachObject(object_name, hand_frame);
  geometry_msgs::msg::PoseStamped gripper_tip_pose = move_group_interface.getCurrentPose(hand_frame);
  //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Retrieved pose position x: %.2f y: %.2f z: %.2f",gripper_tip_pose.pose.position.x,gripper_tip_pose.pose.position.y,gripper_tip_pose.pose.position.z);
  //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Retrieved pose orientation w: %.2f x: %.2f y: %.2f z: %.2f",gripper_tip_pose.pose.orientation.w,gripper_tip_pose.pose.orientation.x,gripper_tip_pose.pose.orientation.y,gripper_tip_pose.pose.orientation.z);
  gripper_tip_pose.pose.position.z += POSITION_TO_PICK_DISTANCE/4;
  //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "New z position: %.2f",gripper_tip_pose.pose.position.z);
  move_group_interface.setPoseTarget(gripper_tip_pose.pose,hand_frame);
  move_group_interface.setPlannerId(planner_id);
  move_group_interface.setMaxVelocityScalingFactor(1.0);
  move_group_interface.setMaxAccelerationScalingFactor(1.0);
  // Create a plan to that target pose
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto const plan_output = static_cast<bool>(move_group_interface.plan(plan));

  // Execute the plan
  if(plan_output) {
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Planing failed!");
  }

  return moveit::core::errorCodeToString(plan_output);
}

std::string MTCTaskNode::position_to_placePlan(environment_interface::msg::Block block, std::string robot_name, std::string planner_id)
{
  const auto& arm_group_name = robot_name + "_arm";
  std::string object_name = "block_" + std::to_string(block.number);

  geometry_msgs::msg::PoseStamped target_pose_msg;
  target_pose_msg.header.frame_id = "base";
  target_pose_msg.pose.position.x = -base_x_size/2 + block_size_x*((block.x) + block.x_size/2);
  target_pose_msg.pose.position.y = -base_y_size/2 + block_size_x*((block.y) + block.y_size/2);
  target_pose_msg.pose.position.z = base_z_size/2 + block_size_z/2 + block_size_z*(block.z + BASE_CORRECTION_VALUE) + INSERT_DISTANCE;
  
  if((block.x_size == 2 && block.y_size == 1) || (block.x_size == 1 && block.y_size == 2)) // 2x1 blocks are in a diferent orientation in the parts feeder, so this is necessary to maintain the logic for other block types
  {
    int swap_aux = block.x_size;
    block.x_size = block.y_size;
    block.y_size = swap_aux;
  }
  if(block.x_size > block.y_size) // If the assembly orientation is the same as the parts feeder, correct the tool orientation
  {
    double cr = cos(0);
    double sr = sin(0);
    double cp = cos(0);
    double sp = sin(0);
    double cy = cos(M_PI/4); // Yaw turn of 90 degrees
    double sy = sin(M_PI/4); 
    target_pose_msg.pose.orientation.w = cr * cp * cy + sr * sp * sy;
    target_pose_msg.pose.orientation.x = sr * cp * cy - cr * sp * sy;
    target_pose_msg.pose.orientation.y = cr * sp * cy + sr * cp * sy;
    target_pose_msg.pose.orientation.z = cr * cp * sy - sr * sp * cy;
  }

  moveit::planning_interface::PlanningSceneInterface psi;
  auto move_group_interface = moveit::planning_interface::MoveGroupInterface(node_, arm_group_name);
  move_group_interface.setPoseTarget(target_pose_msg,object_name);
  move_group_interface.setPlannerId(planner_id);
  move_group_interface.setMaxVelocityScalingFactor(1.0);
  move_group_interface.setMaxAccelerationScalingFactor(1.0);
  // Create a plan to that target pose
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto const plan_output = static_cast<bool>(move_group_interface.plan(plan));

  // Execute the plan
  if(plan_output) {
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Planing failed!");
  }

  return moveit::core::errorCodeToString(plan_output);
}

std::string MTCTaskNode::descend_to_placePlan(std::string robot_name, std::string planner_id)
{
  const auto& arm_group_name = robot_name + "_arm";
  const auto& hand_frame = "gripper_tip";

  auto move_group_interface = moveit::planning_interface::MoveGroupInterface(node_, arm_group_name);
  geometry_msgs::msg::PoseStamped gripper_tip_pose = move_group_interface.getCurrentPose(hand_frame);
  //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Retrieved pose position x: %.2f y: %.2f z: %.2f",gripper_tip_pose.pose.position.x,gripper_tip_pose.pose.position.y,gripper_tip_pose.pose.position.z);
  //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Retrieved pose orientation w: %.2f x: %.2f y: %.2f z: %.2f",gripper_tip_pose.pose.orientation.w,gripper_tip_pose.pose.orientation.x,gripper_tip_pose.pose.orientation.y,gripper_tip_pose.pose.orientation.z);
  gripper_tip_pose.pose.position.z -= INSERT_DISTANCE; //Stop at POSITION_TO_PICK_DISTANCE cm above the block 22.12;//
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "New z position: %.2f",gripper_tip_pose.pose.position.z);
  move_group_interface.setPoseTarget(gripper_tip_pose.pose,hand_frame);
  move_group_interface.setPlannerId(planner_id);
  move_group_interface.setMaxVelocityScalingFactor(1.0);
  move_group_interface.setMaxAccelerationScalingFactor(1.0);
  // Create a plan to that target pose
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto const plan_output = static_cast<bool>(move_group_interface.plan(plan));

  // Execute the plan
  if(plan_output) {
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Planing failed!");
  }
  return moveit::core::errorCodeToString(plan_output);
}

std::string MTCTaskNode::retreat_from_structurePlan(environment_interface::msg::Block block, std::string robot_name, std::string planner_id)
{
  const auto& arm_group_name = robot_name + "_arm";
  const auto& hand_frame = "gripper_tip";
  std::string object_name = "block_" + std::to_string(block.number);
  std::vector<std::string> object_vector;
  object_vector.insert(object_vector.end(),object_name);

  auto move_group_interface = moveit::planning_interface::MoveGroupInterface(node_, arm_group_name);
  move_group_interface.detachObject(object_name);
  geometry_msgs::msg::PoseStamped gripper_tip_pose = move_group_interface.getCurrentPose(hand_frame);
  //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Retrieved pose position x: %.2f y: %.2f z: %.2f",gripper_tip_pose.pose.position.x,gripper_tip_pose.pose.position.y,gripper_tip_pose.pose.position.z);
  //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Retrieved pose orientation w: %.2f x: %.2f y: %.2f z: %.2f",gripper_tip_pose.pose.orientation.w,gripper_tip_pose.pose.orientation.x,gripper_tip_pose.pose.orientation.y,gripper_tip_pose.pose.orientation.z);
  gripper_tip_pose.pose.position.z += POSITION_TO_RETREAT;
  //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "New z position: %.2f",gripper_tip_pose.pose.position.z);
  move_group_interface.setPoseTarget(gripper_tip_pose.pose,hand_frame);
  move_group_interface.setPlannerId(planner_id);
  move_group_interface.setMaxVelocityScalingFactor(1.0);
  move_group_interface.setMaxAccelerationScalingFactor(1.0);
  // Create a plan to that target pose
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto const plan_output = static_cast<bool>(move_group_interface.plan(plan));

  // Execute the plan
  if(plan_output) {
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Planing failed!");
  }

  return moveit::core::errorCodeToString(plan_output);
}

mtc::Task MTCTaskNode::go_homeTask(std::string robot_name)
{
  mtc::Task task;
  const auto& arm_group_name = robot_name + "_arm";
  
  // Set task properties
  task.setProperty("group", arm_group_name);
  task.stages()->setName("Return Home");
  task.loadRobotModel(node_);
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  task.add(std::move(stage_state_current));

  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  interpolation_planner->setMaxVelocityScalingFactor(0.9);
  interpolation_planner->setMaxAccelerationScalingFactor(0.9);
  {
    auto stage =
        std::make_unique<mtc::stages::MoveTo>("Return Home", interpolation_planner);
    stage->setGroup(arm_group_name);
    stage->setGoal("Home");
    task.add(std::move(stage));
  }

  return task;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions mtc_options;
  mtc_options.automatically_declare_parameters_from_overrides(true);
  auto mtc_node = std::make_shared<MTCTaskNode>(mtc_options);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_node]() {
    executor.add_node(mtc_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_node->getNodeBaseInterface());
  });
  auto action_server = std::make_shared<TaskCreatorActionServer>();
  action_server->mtc_node_ = mtc_node;
  rclcpp::spin(action_server);
  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}
