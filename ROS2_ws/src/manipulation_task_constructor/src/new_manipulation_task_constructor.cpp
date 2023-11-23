#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include "environment_interface/msg/block.hpp"
#include "environment_interface/srv/block_create.hpp"
#include "environment_interface/srv/block_remove.hpp"
#include "environment_interface/srv/get_block_color.hpp"
#include "environment_information.h"
#include "manipulation_task_constructor/manipulation_task_constructor.h"
#include <chrono>
#include <string>
#include <fstream>
#include <iostream>

using namespace std::chrono_literals;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("manipulation_task_constructor");
namespace mtc = moveit::task_constructor;

class MTCTaskNode
{
public:
  
  MTCTaskNode(const rclcpp::NodeOptions &options);
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  std::string doTask(environment_interface::msg::Block block, size_t OPERATION, std::string robot_name, std::string solver);
  moveit_msgs::msg::CollisionObject request_block(environment_interface::msg::Block& block);
  std::tuple<std::vector<std::vector<int>>, std::string, int> input_arguments_setup();
  void remove_old_block(environment_interface::msg::Block block);
  void refil_block(moveit_msgs::msg::CollisionObject object);
  void append_block(environment_interface::msg::Block block);
  void correct_color(environment_interface::msg::Block block);
private:
  // Compose an MTC task from a series of stages.
  mtc::Task pick_and_placeTask(environment_interface::msg::Block block, std::string robot_name,  std::string solver);
  mtc::Task return_homeTaks(std::string robot_name);
  mtc::Task retreatTask(std::string robot_name);
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
};

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions &options)
    : node_{std::make_shared<rclcpp::Node>("mtc_node", options)}
{
}

std::string MTCTaskNode::doTask(environment_interface::msg::Block block, size_t OPERATION, std::string robot_name, std::string solver)
{
  switch (OPERATION)
  {
  case PICK_AND_PLACE:
    task_ = pick_and_placeTask(block, robot_name, solver);
    break;
  
  case RETREAT:
    task_ = retreatTask(robot_name);
    break;
  
  case RETURN_HOME:
    task_ = return_homeTaks(robot_name);
    break;
  default:
    RCLCPP_ERROR_STREAM(LOGGER, "Opperation not defined.");
    return "OPERATION_UNKNOWN";
    break;
  }
  
  try
  {
    task_.init();
  }
  catch (mtc::InitStageException &e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return "FATAL_ERROR";
  }
  
  if(!task_.plan())
  {
    RCLCPP_ERROR_STREAM(LOGGER, "TASK PLANNING FAILED.");
    sleep(10000);
    return "TASK_PLANNING_FAIL";
  }
  RCLCPP_INFO(LOGGER, "Task planning succeded.");

  //task_.introspection().publishSolution(*task_.solutions().front()); // Holds the solution for inspection if not commented, the next steps will not be perfomed.
  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, result.val);
    sleep(10000);
    return "TASK_EXECUTION_FAIL";
  }
  RCLCPP_INFO(LOGGER, "Task execution succeded.");
  return "SUCCESS";
}

mtc::Task MTCTaskNode::pick_and_placeTask(environment_interface::msg::Block block, std::string robot_name, std::string solver)
{
  mtc::Task task;
  const auto& arm_group_name = robot_name + "_arm";
  const auto& hand_group_name = robot_name + "_hand";
  const auto& end_effector_name = "gripper";
  const auto& hand_frame = "gripper_tip";
  std::string s = std::to_string(block.number);
  std::string object_name = "block_" + s;

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", end_effector_name);
  task.setProperty("ik_frame", hand_frame);
  task.stages()->setName("assembly task" + object_name);
  task.loadRobotModel(node_);

  geometry_msgs::msg::PoseStamped target_pose_msg;
  target_pose_msg.header.frame_id = "base";
  target_pose_msg.pose.position.x = -base_x_size/2 + block_size*((block.x + BASE_CORRECTION_VALUE) + block.x_size/2);
  target_pose_msg.pose.position.y = -base_y_size/2 + block_size*((block.y + BASE_CORRECTION_VALUE) + block.y_size/2);
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
  
  mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_,"ompl");
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  //sampling_planner->setPlannerId("stomp", solver);
  //auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_,"pilz_industrial_motion_planner");
  //sampling_planner->setPlannerId("PTP");

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setStepSize(.01);
  // clang-format off
  auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
      "move to pick",
      mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
  // clang-format on
  stage_move_to_pick->setTimeout(15.0);
  stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_pick));

  // clang-format off
  mtc::Stage* attach_object_stage = nullptr;  // Forward attach_object_stage to place pose generator
  // clang-format on

  // This is an example of SerialContainer usage. It's not strictly needed here.
  // In fact, `task` itself is a SerialContainer by default.
  {
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
    // clang-format off
    grasp->properties().configureInitFrom(mtc::Stage::PARENT,
                                          { "eef", "group", "ik_frame" });
    // clang-format on
    
    {
      // Sample grasp pose
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      stage->setPreGraspPose("Open");
      stage->setObject(object_name);
      stage->setAngleDelta(M_PI / 2);
      stage->setMonitoredStage(current_state_ptr); // Hook into current state

      // This is the transform from the object frame to the end-effector frame
      Eigen::Isometry3d grasp_frame_transform;
      Eigen::Quaterniond q(0,1,0,0); // (w,x,y,z)
      grasp_frame_transform.linear() = q.matrix();
      grasp_frame_transform.translation().z() = GRASP_CLAW_STOP_DISTANCE; //Stop at CLAW_STOP_DISTANCE from object

      // Compute IK
      // clang-format off
      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
      // clang-format on
      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(grasp_frame_transform, hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      grasp->insert(std::move(wrapper));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject(object_name, hand_frame);
      attach_object_stage = stage.get();
      grasp->insert(std::move(stage));
    }

    {
      // clang-format off
      cartesian_planner->setMaxVelocityScalingFactor(0.7);
      cartesian_planner->setMaxAccelerationScalingFactor(0.7);
      auto stage = std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
      // clang-format on
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(GRASP_CLAW_STOP_DISTANCE - minimum_resolution, GRASP_CLAW_STOP_DISTANCE);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "lift_object");

      // Set upward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }
    
    task.add(std::move(grasp));
  }
  
  // clang-format off
  auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
      "move to place",
      mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner },
                                                { hand_group_name, interpolation_planner } });
  // clang-format on
  stage_move_to_place->setTimeout(15.0);
  stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_place));
  
  {
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
    // clang-format off
    place->properties().configureInitFrom(mtc::Stage::PARENT,
                                          { "eef", "group", "ik_frame" });
    // clang-format on

    {
      // Sample place pose
      auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "place_pose");
      stage->setObject(object_name);
      stage->setPose(target_pose_msg);
      stage->setMonitoredStage(attach_object_stage); // Hook into attach_object_stage

      // This is the transform from the object frame to the end-effector frame
      Eigen::Isometry3d grasp_frame_transform;
      Eigen::Quaterniond q(0,1,0,0); // (w,x,y,z)
      grasp_frame_transform.linear() = q.matrix();

      // Compute IK
      // clang-format off
      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
      // clang-format on
      wrapper->setMaxIKSolutions(10);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(grasp_frame_transform, hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      place->insert(std::move(wrapper));
    }

    {
      // clang-format off
      cartesian_planner->setMaxVelocityScalingFactor(0.2);
      cartesian_planner->setMaxAccelerationScalingFactor(0.2);
      auto stage = std::make_unique<mtc::stages::MoveRelative>("insert object", cartesian_planner);
      // clang-format on
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(INSERT_DISTANCE - minimum_resolution, INSERT_DISTANCE);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "insert_object");

      // Set upward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = -1.0;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }

    task.add(std::move(place));
  }
  
  
  // clang-format off
  auto stage =
      std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
  stage->detachObject(object_name, hand_frame);
  stage->allowCollisions(object_name,
                          task.getRobotModel()
                              ->getJointModelGroup(hand_group_name)
                              ->getLinkModelNamesWithCollisionGeometry(),
                          false);
  // clang-format on
  task.add(std::move(stage));
  
  return task;
}

mtc::Task MTCTaskNode::retreatTask(std::string robot_name)
{
  mtc::Task task;
  const auto& arm_group_name = robot_name + "_arm";
  const auto& hand_frame = "gripper_tip";
  // Set task properties
  task.setProperty("group", arm_group_name);
  task.stages()->setName("Retreat");
  task.loadRobotModel(node_);

  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  task.add(std::move(stage_state_current));

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(.01);

  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    stage->setMinMaxDistance(0.2, 0.5);
    stage->setIKFrame(hand_frame);
    stage->properties().set("marker_ns", "retreat");

    // Set retreat direction
    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = "world";
    vec.vector.z = 1;
    stage->setDirection(vec);
    task.add(std::move(stage));
  }

  return task;
}

mtc::Task MTCTaskNode::return_homeTaks(std::string robot_name)
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
  {
    // clang-format off
    auto stage =
        std::make_unique<mtc::stages::MoveTo>("Return Home", interpolation_planner);
    // clang-format on
    stage->setGroup(arm_group_name);
    stage->setGoal("Home");
    task.add(std::move(stage));
  }

  return task;
}

void MTCTaskNode::remove_old_block(environment_interface::msg::Block block)
{
  moveit::planning_interface::PlanningSceneInterface psi;
  std::vector<std::string> object_name = {block.name,"/0"};
  moveit_msgs::msg::CollisionObject removed_block;
  std::map<std::string, moveit_msgs::msg::CollisionObject> removed_block_msg;
  removed_block_msg = psi.getObjects(object_name);
  removed_block = removed_block_msg.begin()->second;
  removed_block.operation = removed_block.REMOVE;
  psi.applyCollisionObject(removed_block);
}

std::tuple<std::vector<std::vector<int>>, std::string, int> MTCTaskNode::input_arguments_setup()
{
  std::string myFilePath = node_->get_parameter("csv_file_path").as_string();
  std::string robot_name = node_->get_parameter("robot_name").as_string();
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "robot_name: %s", robot_name.c_str());
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "parameter name: %s", robot_name.c_str());
  int first_block = node_->get_parameter("first_block").as_int();
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "robot_name: %s", robot_name.c_str());
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "parameter name: %s", robot_name.c_str());
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "first_block: %d", first_block);
  //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "parameter name: %s", robot_name.c_str());
  std::ifstream assemblyfile;
  assemblyfile.open(myFilePath);
  


  std::vector<std::vector<int>> assembly_plan;
  if(assemblyfile.fail())
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Csv file not found.");
    return std::make_tuple(assembly_plan,robot_name,first_block);
  }
  int assembly_size = 0;
  while(assemblyfile.peek()!=EOF)
  {
    std::string records;
    assembly_plan.push_back(std::vector<int>());
    if (getline (assemblyfile, records)) {            /* read line of input into line */
        int itmp;                       /* temporary integer to fill */
        std::stringstream ss (records);    /* create stringstream from line */
        
        while (ss >> itmp) {            /* read integer value from ss */
            std::string stmp {};        /* temporary string to hold delim */
            assembly_plan[assembly_size].push_back(itmp);          /* add to vector */
            getline (ss, stmp, ',');  /* read delimiter */
        }
        assembly_size++;
    }
  }
  assemblyfile.close();
  return std::make_tuple(assembly_plan,robot_name,first_block);
}

moveit_msgs::msg::CollisionObject MTCTaskNode::request_block(environment_interface::msg::Block& block)
{ 
  moveit_msgs::msg::CollisionObject replacement_block;
  moveit::planning_interface::PlanningSceneInterface psi;
  moveit_msgs::msg::CollisionObject object;
  std::vector<std::string> object_name_vector = {block.name,"/0"};
  std::map<std::string, moveit_msgs::msg::CollisionObject> object_name_map;
  
  object_name_map = psi.getObjects(object_name_vector);

  object = object_name_map.begin()->second;

  replacement_block = object;
  remove_old_block(block);
  block.name = "block_" + std::to_string(block.number);

  if(block.is_support)
  {
    moveit_msgs::msg::CollisionObject support_block_object;
    support_block_object.id = block.name;
    support_block_object.header.frame_id = "world";
    support_block_object.primitives.resize(1);
    support_block_object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    support_block_object.primitives[0].dimensions = { block_size_z - minimum_resolution, block_size/2 - minimum_resolution };
    support_block_object.pose = object.pose;
    support_block_object.operation = support_block_object.ADD;
    psi.applyCollisionObject(support_block_object, block.color);

  }else{
    object.id = block.name;
    object.header.frame_id = "world";
    object.operation = object.ADD;
    psi.applyCollisionObject(object, block.color);
  }
  
  return replacement_block;
}

void MTCTaskNode::correct_color(environment_interface::msg::Block block)
{ 
  moveit::planning_interface::PlanningSceneInterface psi;
  std::vector<std::string> object_name = {block.name,"/0"};
  moveit_msgs::msg::CollisionObject replacement_block;
  std::map<std::string, moveit_msgs::msg::CollisionObject> replacement_block_msg;
  replacement_block_msg = psi.getObjects(object_name);
  replacement_block = replacement_block_msg.begin()->second;
  replacement_block.operation = replacement_block.ADD;
  psi.applyCollisionObject(replacement_block, block.color);
}

std_msgs::msg::ColorRGBA getColor(int color_index)
{
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("block_service_client");  
  rclcpp::Client<environment_interface::srv::GetBlockColor>::SharedPtr client =                
    node->create_client<environment_interface::srv::GetBlockColor>("get_block_color_service");          

  std_msgs::msg::ColorRGBA color;
  auto request = std::make_shared<environment_interface::srv::GetBlockColor::Request>();       
  
  request->index = color_index;
  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      color.r = 1;
      color.g = 1;
      color.b = 1;
      color.a = 1;
      return color;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service create_block");    
  }
  return result.get()->color;
}

void MTCTaskNode::refil_block(moveit_msgs::msg::CollisionObject refil_object)
{
  std_msgs::msg::ColorRGBA refil_color = getColor(GRAY);
  refil_object.operation = refil_object.ADD;
  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(refil_object, refil_color);
}

void MTCTaskNode::append_block(environment_interface::msg::Block block)
//does not work as intended.
{ 
  moveit::planning_interface::PlanningSceneInterface psi;
  std::vector<std::string> object_name = {block.name,"/0"};
  moveit_msgs::msg::CollisionObject original_block;
  moveit_msgs::msg::CollisionObject appended_block;
  std::map<std::string, moveit_msgs::msg::CollisionObject> original_block_msg;
  original_block_msg = psi.getObjects(object_name);
  original_block = original_block_msg.begin()->second;
  appended_block = original_block;
  //appended_block.header.frame_id = "base";
  appended_block.operation = appended_block.APPEND;
  original_block.operation = original_block.REMOVE;
  moveit_msgs::msg::AttachedCollisionObject a;
  a.object = original_block;
  a.link_name = "base";
  //psi.applyCollisionObject(appended_block);
  psi.applyAttachedCollisionObject(a);
  //psi.applyCollisionObject(original_block);
}

int main(int argc, char **argv)
{
  auto start = std::chrono::high_resolution_clock::now();
  rclcpp::init(argc, argv); 
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  std::vector<std::vector<int>> assembly_plan;
  std::string robot_name;
  int first_block;
  std::tie(assembly_plan, robot_name, first_block) = mtc_task_node->input_arguments_setup();
  int assembly_size = assembly_plan.size();
  std::string task_command_return;
  std::string solver;
  std::string result;

  //      0      1 2 3   4     5     6       7           8        9      10     11 
  //AssemblyArea,X,Y,Z,SizeX,SizeY,SizeZ,ColorIndex,IsSupport,CanPress,ShiftX,ShiftY
  for (int i = first_block - 1; i < assembly_size; i++)
  {
    environment_interface::msg::Block block;
    environment_interface::msg::Block block_teste;
    moveit_msgs::msg::CollisionObject replacement_block;

    block.x = assembly_plan[i][1] + 1;
    block.y = assembly_plan[i][2] + 1;
    block.z = assembly_plan[i][3];
    if(assembly_plan[i][8] == 1)
    {
      block.color = getColor(SUPPORT);
      block.is_support = 1;
    }else 
    {
      block.color = getColor(assembly_plan[i][7]);
    }
    block.number = i+1;
    
    if((assembly_plan[i][4] == 2 && assembly_plan[i][5] == 1) || (assembly_plan[i][4] == 1 && assembly_plan[i][5] == 2)) //Special case for 2x1 block due its different positioning in the parts feeder
    {
      block.name = "21";
      block.x_size = 2;
      block.y_size = 1;
    }else if(assembly_plan[i][4] < assembly_plan[i][5])
    {
      block.name = std::to_string(assembly_plan[i][4]) + std::to_string(assembly_plan[i][5]);
      block.x_size = assembly_plan[i][4];
      block.y_size = assembly_plan[i][5];
    }else
    {
      block.name = std::to_string(assembly_plan[i][5]) + std::to_string(assembly_plan[i][4]);
      block.x_size = assembly_plan[i][5];
      block.y_size = assembly_plan[i][4];
    }
    /*
    if(assembly_plan[i][4] >= assembly_plan[i][5])
    {
      block.name = std::to_string(assembly_plan[i][4]) + std::to_string(assembly_plan[i][5]);
      block.x_size = assembly_plan[i][4];
      block.y_size = assembly_plan[i][5];
    }else
    {
      block.name = std::to_string(assembly_plan[i][5]) + std::to_string(assembly_plan[i][4]);
      block.x_size = assembly_plan[i][5];
      block.y_size = assembly_plan[i][4];
    }*/

    replacement_block = mtc_task_node->request_block(block); //Refil the block in the feeder
    block.x_size = assembly_plan[i][4]; // Correct the sizes for the assembly
    block.y_size = assembly_plan[i][5];
    //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "x_size: %s", std::to_string(assembly_plan[i][4]).c_str());
    //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "y_size: %s", std::to_string(assembly_plan[i][5]).c_str());
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Block Name: %s", block.name.c_str());
    solver = "STOMP";
    result = mtc_task_node->doTask(block,PICK_AND_PLACE,robot_name,solver);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Block %s was %s.",block.name.c_str(),result.c_str());
    mtc_task_node->correct_color(block);
    mtc_task_node->refil_block(replacement_block);
    mtc_task_node->doTask(block,RETREAT,robot_name,solver);
  }

  environment_interface::msg::Block block;
  mtc_task_node->doTask(block,RETURN_HOME,robot_name,solver);
  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Assembly time was: %.2f minutes", static_cast< float >(duration.count())/60000000);
  
  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}