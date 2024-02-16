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
#include <moveit/robot_model_loader/robot_model_loader.h>
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
  std::string plan_output;
  mtc::Task task;
  
  switch (operation_id)
  {
  case POSITION_TO_PICK:
    operation_id_string = "POSITION_TO_PICK";
    plan_output = position_to_pickPlan(block, robot_name, planner_id);
    break;
  
  case PRE_OPEN_GRIPPER:
    plan_output = move_gripperPlan(block, "PRE_OPEN", robot_name);
    operation_id_string = "PRE_OPEN";
    break;

  case DESCEND_TO_PICK:
    plan_output = descend_to_pickPlan(robot_name, planner_id);
    operation_id_string = "DESCEND_TO_PICK";
    break;
    
  case CLOSE_GRIPPER:
    plan_output = move_gripperPlan(block, "CLOSE", robot_name);
    operation_id_string = "CLOSE_GRIPPER";
    break;
    
  case ASCEND_WITH_BLOCK:
    plan_output = ascend_with_blockPlan(block, robot_name, planner_id);
    operation_id_string = "ASCEND_WITH_BLOCK";
    break;
    
  case POSITION_TO_PLACE:
    plan_output = position_to_placePlan(block, robot_name, planner_id);
    operation_id_string = "POSITION_TO_PLACE";
    break;
    
  case DESCEND_TO_PLACE:
    plan_output = descend_to_placePlan(robot_name, planner_id);
    operation_id_string = "DESCEND_TO_PLACE";
    break;
    
  case OPEN_GRIPPER:
    plan_output = move_gripperPlan(block, "OPEN", robot_name);
    operation_id_string = "OPEN_GRIPPER";
    break;
    
  case RETREAT_FROM_STRUCTURE:
    plan_output = retreat_from_structurePlan(block, robot_name, planner_id);
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
    plan_output = "Operation is not implemented.";
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
    if(plan_output != "SUCCESS"){
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "plan error code: %s", plan_output.c_str());
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
  
  if(robot_name == "epson_t3")
  {
    move_group_interface.setJointValueTarget(retrieved_pose,hand_frame);
  }
  else
  {
    move_group_interface.setPoseTarget(retrieved_pose,hand_frame);
  }
  move_group_interface.allowReplanning(true);
  move_group_interface.setReplanDelay(100);
  move_group_interface.setPlanningTime(10);
  move_group_interface.setPlannerId(planner_id);
  move_group_interface.setMaxVelocityScalingFactor(1.0);
  move_group_interface.setMaxAccelerationScalingFactor(1.0);
  // Create a plan to that target pose
  auto const plan_output = static_cast<bool>(move_group_interface.move());

  // Execute the plan
  if(plan_output) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Planing success!");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Planing failed!");
  }

  return moveit::core::error_code_to_string(plan_output);
}

std::string MTCTaskNode::descend_to_pickPlan(std::string robot_name, std::string planner_id)
{
  const auto& arm_group_name = robot_name + "_arm";
  const auto& hand_frame = "gripper_tip";

  auto move_group_interface = moveit::planning_interface::MoveGroupInterface(node_, arm_group_name);
  geometry_msgs::msg::PoseStamped gripper_tip_pose = move_group_interface.getCurrentPose(hand_frame);

  if(robot_name == "epson_t3")
  {
    //geometry_msgs::msg::PoseStamped gripper_tip_pose = move_group_interface.getCurrentPose("gripper_base");
    gripper_tip_pose.pose.position.z -= (POSITION_TO_PICK_DISTANCE - block_size_z/2 - minimum_resolution);
    move_group_interface.setJointValueTarget(gripper_tip_pose.pose,hand_frame);
  }
  else
  {
    gripper_tip_pose.pose.position.z -= (POSITION_TO_PICK_DISTANCE - block_size_z/2 - minimum_resolution); //Stop at POSITION_TO_PICK_DISTANCE cm above the block 22.12;//
    move_group_interface.setPoseTarget(gripper_tip_pose.pose,hand_frame);
  }
  move_group_interface.allowReplanning(true);
  move_group_interface.setReplanDelay(100);
  move_group_interface.setPlanningTime(10);
  move_group_interface.setPlannerId(planner_id);
  move_group_interface.setMaxVelocityScalingFactor(1.0);
  move_group_interface.setMaxAccelerationScalingFactor(1.0);
  // Create a plan to that target pose
  auto const plan_output = static_cast<bool>(move_group_interface.move());

  // Execute the plan
  if(plan_output) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Planing success!");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Planing failed!");
  }
  return moveit::core::error_code_to_string(plan_output);
}

std::string MTCTaskNode::move_gripperPlan(environment_interface::msg::Block block, std::string command, std::string robot_name)
{
  // Gripper full closed is 1.882 cm 
  const auto& hand_group_name = robot_name + "_hand";
  float gripper_joint_position;
  auto move_group_interface = moveit::planning_interface::MoveGroupInterface(node_, hand_group_name);
  float block_size, gripper_max;
  if(robot_name == "epson_t3")
  {
   gripper_max = 1.5;
  }
  else
  {
   gripper_max = gripper_max_opening;
  }

  if(command == "OPEN")
  {
    std::vector<double> current_joint_value;
    current_joint_value = move_group_interface.getCurrentJointValues();
    gripper_joint_position = current_joint_value[0] - 0.05; //Open the gripper 0.05 cm 
  }
  else{
    if((block.x_size == 2 && block.y_size == 1) || (block.x_size == 1 && block.y_size == 2)){
      block_size = block_size_x/2;
    }else if(block.x_size > block.y_size){
      block_size = block_size_x*block.x_size/2;
    }else {
      block_size = block_size_x*block.y_size/2;
    }
    if(command == "CLOSE"){
      gripper_joint_position = gripper_max - block_size + KNOB_DISTANCE_TO_GRASP-0.07;
    }else if(command == "PRE_OPEN"){
      gripper_joint_position = gripper_max - block_size + KNOB_DISTANCE_TO_GRASP - 0.1;
    }
  }

  std::map<std::string, double> joint_values;
  joint_values["joint_gripper"] = gripper_joint_position;
  RCLCPP_ERROR(rclcpp::get_logger("rclpp"),"Gripper position: %.4f", gripper_joint_position);
  move_group_interface.setJointValueTarget(joint_values);
  
  move_group_interface.allowReplanning(true);
  move_group_interface.setReplanDelay(100);
  move_group_interface.setPlanningTime(10);
  move_group_interface.setMaxVelocityScalingFactor(1.0);
  move_group_interface.setMaxAccelerationScalingFactor(1.0);
  // Create a plan to that target pose
  auto const plan_output = static_cast<bool>(move_group_interface.move());

  // Execute the plan
  if(plan_output) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Planing success!");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Planing failed!");
  }

  return moveit::core::error_code_to_string(plan_output);
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

  if(robot_name == "denso_cobotta"){
    gripper_tip_pose.pose.position.z += POSITION_TO_PICK_DISTANCE*5;
  }else if(robot_name == "denso_vp6242"){
    gripper_tip_pose.pose.position.z += POSITION_TO_PICK_DISTANCE;
  }else{
    gripper_tip_pose.pose.position.z += POSITION_TO_PICK_DISTANCE;
  }
  //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "New z position: %.2f",gripper_tip_pose.pose.position.z);
  if(robot_name == "epson_t3")
  {
    move_group_interface.setJointValueTarget(gripper_tip_pose.pose,hand_frame);
  }
  else
  {
    move_group_interface.setPoseTarget(gripper_tip_pose.pose,hand_frame);
  }
  
  move_group_interface.allowReplanning(true);
  move_group_interface.setReplanDelay(100);
  move_group_interface.setPlanningTime(10);
  move_group_interface.setPlannerId(planner_id);
  move_group_interface.setMaxVelocityScalingFactor(1.0);
  move_group_interface.setMaxAccelerationScalingFactor(1.0);
  // Create a plan to that target pose
  auto const plan_output = static_cast<bool>(move_group_interface.move());

  // Execute the plan
  if(plan_output) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Planing success!");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Planing failed!");
  }

  return moveit::core::error_code_to_string(plan_output);
}

std::string MTCTaskNode::position_to_placePlan(environment_interface::msg::Block block, std::string robot_name, std::string planner_id)
{
  const auto& arm_group_name = robot_name + "_arm";
  const auto& hand_frame = "gripper_tip";

  geometry_msgs::msg::PoseStamped target_pose_msg;
  target_pose_msg.header.frame_id = "base"; 
  target_pose_msg.pose.position.x = -base_x_size/2 + block_size_x*((block.x) + block.x_size/2);
  target_pose_msg.pose.position.y = -base_y_size/2 + block_size_x*((block.y) + block.y_size/2);
  target_pose_msg.pose.position.z = base_z_size/2 + block_size_z/2 + block_size_z*(block.z + BASE_CORRECTION_VALUE) + INSERT_DISTANCE;
  
  if((block.x_size == 2 && block.y_size == 1) || (block.x_size == 1 && block.y_size == 2)) // 2x1 blocks are in a different orientation in the parts feeder, so this is necessary to maintain the logic for other block types
  {
    int swap_aux = block.x_size;
    block.x_size = block.y_size;
    block.y_size = swap_aux;
  }
  if(block.x_size > block.y_size) // If the assembly orientation is the same as the parts feeder, correct the tool orientation
  {
    target_pose_msg.pose.position.y += - GRIPPER_POSITION_TO_PICK_CORRECTION;
    target_pose_msg.pose.orientation.w = 0;
    target_pose_msg.pose.orientation.x = 0.7071068;
    target_pose_msg.pose.orientation.y = -0.7071068;
    target_pose_msg.pose.orientation.z = 0;
  }else{
    target_pose_msg.pose.position.x += - GRIPPER_POSITION_TO_PICK_CORRECTION;
    target_pose_msg.pose.orientation.w = 0;
    target_pose_msg.pose.orientation.x = 0;
    target_pose_msg.pose.orientation.y = 1;
    target_pose_msg.pose.orientation.z = 0;
  }

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Retrieved pose position x: %.2f y: %.2f z: %.2f\n",target_pose_msg.pose.position.x,target_pose_msg.pose.position.y,target_pose_msg.pose.position.z);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Retrieved pose orientation w: %.2f x: %.2f y: %.2f z: %.2f",target_pose_msg.pose.orientation.w,target_pose_msg.pose.orientation.x,target_pose_msg.pose.orientation.y,target_pose_msg.pose.orientation.z);

  auto move_group_interface = moveit::planning_interface::MoveGroupInterface(node_, arm_group_name);
  if(robot_name == "epson_t3")
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "executing Position to place!!!!");
    std::vector<geometry_msgs::msg::Pose> collision_objects;
    geometry_msgs::msg::Pose object;
    std::map<std::string, geometry_msgs::msg::Pose> object_name_map;
    geometry_msgs::msg::Pose corrected_pose;
    moveit::planning_interface::PlanningSceneInterface psi;
    std::vector<std::string> obj;
    std::map<std::string, geometry_msgs::msg::Pose>::iterator it;
    obj.push_back("base");
    object_name_map = psi.getObjectPoses(obj); 
    it = object_name_map.find("base");
    if (it != object_name_map.end()) 
    {
      corrected_pose = it->second;
    }
    corrected_pose.position.x += target_pose_msg.pose.position.x;
    corrected_pose.position.y += target_pose_msg.pose.position.y;
    corrected_pose.position.z += target_pose_msg.pose.position.z;

    target_pose_msg.header.frame_id = "world";
    target_pose_msg.pose.position.x =corrected_pose.position.x;
    target_pose_msg.pose.position.y = corrected_pose.position.y;
    target_pose_msg.pose.position.z = corrected_pose.position.z;

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Retrieved pose position from Target Pos  x: %.2f y: %.2f z: %.2f\n",target_pose_msg.pose.position.x,target_pose_msg.pose.position.y,target_pose_msg.pose.position.z);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Retrieved pose position from corrected Pos x: %.2f y: %.2f z: %.2f\n",corrected_pose.position.x,corrected_pose.position.y,corrected_pose.position.z);

    move_group_interface.setJointValueTarget(target_pose_msg,hand_frame);
  }
  else
  {
    move_group_interface.setPoseTarget(target_pose_msg,hand_frame);
  }
  
  move_group_interface.allowReplanning(true);
  move_group_interface.setReplanDelay(100);
  move_group_interface.setPlanningTime(10);
  move_group_interface.setPlannerId(planner_id);
  move_group_interface.setMaxVelocityScalingFactor(1.0);
  move_group_interface.setMaxAccelerationScalingFactor(1.0);
  // Create a plan to that target pose
  auto const plan_output = static_cast<bool>(move_group_interface.move());

  // Execute the plan
  if(plan_output) {
    //RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Planing success!");
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Planing success!");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Planing failed!");
  }
  return moveit::core::error_code_to_string(plan_output);
}

std::string MTCTaskNode::descend_to_placePlan(std::string robot_name, std::string planner_id)
{
  const auto& arm_group_name = robot_name + "_arm";
  const auto& hand_frame = "gripper_tip";
  auto move_group_interface = moveit::planning_interface::MoveGroupInterface(node_, arm_group_name);
  geometry_msgs::msg::PoseStamped gripper_tip_pose = move_group_interface.getCurrentPose(hand_frame);
  gripper_tip_pose.pose.position.z -= (INSERT_DISTANCE - block_size_z/2 - minimum_resolution);
  if(robot_name == "epson_t3")
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "New z position: %.2f",gripper_tip_pose.pose.position.z);
    move_group_interface.setJointValueTarget(gripper_tip_pose.pose,hand_frame);
  }
  else
  {
    move_group_interface.setPoseTarget(gripper_tip_pose.pose,hand_frame);
  }
  move_group_interface.allowReplanning(true);
  move_group_interface.setReplanDelay(100);
  move_group_interface.setPlanningTime(15);
  move_group_interface.setPlannerId(planner_id);
  move_group_interface.setMaxVelocityScalingFactor(1.0);
  move_group_interface.setMaxAccelerationScalingFactor(1.0);
  // Create a plan to that target pose
  auto const plan_output = static_cast<bool>(move_group_interface.move());
  // Execute the plan
  if(plan_output) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Planing success!");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Planing failed!");
  }
  return moveit::core::error_code_to_string(plan_output);
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
  gripper_tip_pose.pose.position.z += POSITION_TO_RETREAT;
  //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "New z position: %.2f",gripper_tip_pose.pose.position.z);

  if(robot_name == "epson_t3")
  {
    move_group_interface.setJointValueTarget(gripper_tip_pose.pose,hand_frame);
  }
  else
  {
    move_group_interface.setPoseTarget(gripper_tip_pose.pose,hand_frame);
  }
  move_group_interface.allowReplanning(true);
  move_group_interface.setReplanDelay(100);
  move_group_interface.setPlanningTime(10);
  move_group_interface.setPlannerId(planner_id);
  move_group_interface.setMaxVelocityScalingFactor(1.0);
  move_group_interface.setMaxAccelerationScalingFactor(1.0);
  // Create a plan to that target pose
  auto const plan_output = static_cast<bool>(move_group_interface.move());

  // Execute the plan
  if(plan_output) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Planing success!");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Planing failed!");
  }
  
  return moveit::core::error_code_to_string(plan_output);
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
