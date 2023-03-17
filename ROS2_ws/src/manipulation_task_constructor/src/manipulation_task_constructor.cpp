#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include "environment_interface/msg/block.hpp"
#include "environment_interface/srv/create_block.hpp"
#include <chrono>

#define block_size 0.3
#define table_x_size 30.0
#define table_y_size 20.0
#define table_z_size 25.0
#define base_x_size 6.0
#define base_y_size 6.0
#define base_z_size 0.2
#define dispenser_x_size 2.4
#define dispenser_y_size 12.0
#define dispenser_z_size 0.2
#define minimum_resolution 0.01

#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

using namespace std::chrono_literals;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("manipulation_task_constructor");
namespace mtc = moveit::task_constructor;

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions &options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask(environment_interface::msg::Block block);
  void doReturnHome();

private:
  // Compose an MTC task from a series of stages.
  void request_block(environment_interface::msg::Block block);
  mtc::Task createTask(environment_interface::msg::Block block);
  mtc::Task returnHome();
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

void MTCTaskNode::doTask(environment_interface::msg::Block block)
{
  task_ = createTask(block);
  try
  {
    task_.init();
  }
  catch (mtc::InitStageException &e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  if (!task_.plan(20))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    while(1);
    return;
  }
  //task_.introspection().publishSolution(*task_.solutions().front());
  moveit_msgs::msg::MoveItErrorCodes execute_result;
  execute_result = task_.execute(*task_.solutions().front());
  if (execute_result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed and returned: " << execute_result.val);
    while(1);
    return;
  }
  return;
}

void MTCTaskNode::doReturnHome()
{
  task_ = returnHome();
  try
  {
    task_.init();
  }
  catch (mtc::InitStageException &e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  if (!task_.plan(5))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }
  //task_.introspection().publishSolution(*task_.solutions().front());
  moveit_msgs::msg::MoveItErrorCodes execute_result;
  execute_result = task_.execute(*task_.solutions().front());
  if (execute_result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed and returned: " << execute_result.val);
    return;
  }
  return;
}

/*
void request_block(environment_interface::msg::Block block)
{
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("create_block_client");  // CHANGE
  rclcpp::Client<environment_interface::srv::CreateBlock>::SharedPtr client =                // CHANGE
    node->create_client<environment_interface::srv::CreateBlock>("create_block");          // CHANGE

  auto request = std::make_shared<environment_interface::srv::CreateBlock::Request>();       // CHANGE
  request->x = 18;
  request->y = 1;
  request->z = 1;
  request->x_size = block.x_size;
  request->y_size = block.y_size;
  request->block_number = block.block_number;

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Res: %ld", result.get()->res);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service create_block");    // CHANGE
  }
}*/

mtc::Task MTCTaskNode::returnHome()
{
  mtc::Task task;
  task.stages()->setName("Return Home");
  task.loadRobotModel(node_);

  const auto& arm_group_name = "cobotta_arm";
  const auto& hand_group_name = "cobotta_hand";

  // Set task properties
  task.setProperty("group", arm_group_name);

  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  task.add(std::move(stage_state_current));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScaling(1.0);
  cartesian_planner->setMaxAccelerationScaling(1.0);
  cartesian_planner->setStepSize(.01);

  {
    // clang-format off
    auto stage =
        std::make_unique<mtc::stages::MoveTo>("Return Home", interpolation_planner);
    // clang-format on
    stage->setGroup(arm_group_name);
    stage->setGoal("Home");
    task.add(std::move(stage));
  }
  
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
    stage->setGroup(hand_group_name);
    stage->setGoal("close_hand");
    task.add(std::move(stage));
  }

  return task;
}

mtc::Task MTCTaskNode::createTask(environment_interface::msg::Block block)
{
  mtc::Task task;

  std::string s = std::to_string(block.number);
  std::string object_name = "block_" + s;

  geometry_msgs::msg::PoseStamped target_pose_msg;
  target_pose_msg.header.frame_id = "base";
  target_pose_msg.pose.position.x = -base_x_size/2 + block_size*((block.x - 1) + block.x_size*0.5);
  target_pose_msg.pose.position.y = -base_y_size/2 + block_size*((block.y - 1) + block.y_size*0.5);
  target_pose_msg.pose.position.z = base_z_size/2 + block_size*(block.z - 0.5) + 1.5;
  if(block.x_size >= block.y_size)
  {
    target_pose_msg.pose.orientation.z = 0; //Keep the block orientation
  }else{
    target_pose_msg.pose.orientation.z = 1.0; //(Needs to rotate PI/2 degrees.
  }

  task.stages()->setName("assembly task "+object_name);
  task.loadRobotModel(node_);

  const auto& arm_group_name = "cobotta_arm";
  const auto& hand_group_name = "cobotta_hand";
  const auto &end_effector_name = "gripper";
  const auto& hand_frame = "gripper_base";  

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", end_effector_name);
  task.setProperty("ik_frame", hand_frame);
  
  mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScaling(1.0);
  cartesian_planner->setMaxAccelerationScaling(1.0);
  cartesian_planner->setStepSize(.01);

  // clang-format off
  auto stage_open_hand =
      std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  // clang-format on
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal("open_hand");
  task.add(std::move(stage_open_hand));

  // clang-format off
  auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
      "move to pick",
      mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
  // clang-format on
  stage_move_to_pick->setTimeout(5.0);
  stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_pick));

  // clang-format off
  mtc::Stage* attach_object_stage = nullptr;  // Forward attach_object_stage to place pose generator
  mtc::Stage* insert_object_stage = nullptr;
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
      // clang-format off
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
      // clang-format on
      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.3, 0.5);

      // Set hand forward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame;
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }
    
    {
      // Sample grasp pose
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      stage->setPreGraspPose("open_hand");
      stage->setObject(object_name);
      stage->setAngleDelta(M_PI);
      stage->setMonitoredStage(current_state_ptr); // Hook into current state

      // This is the transform from the object frame to the end-effector frame
      Eigen::Isometry3d grasp_frame_transform;
      Eigen::Quaterniond q(0,1,0,0); // Approach from Z axis from top to bottom
      grasp_frame_transform.linear() = q.matrix();
      grasp_frame_transform.translation().z() = 6.15; //Stop at 65cm from object

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
      // clang-format off
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
      stage->allowCollisions(object_name,
                             task.getRobotModel()
                                 ->getJointModelGroup(hand_group_name)
                                 ->getLinkModelNamesWithCollisionGeometry(),
                             true);
      // clang-format on
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand to grab", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("grab");
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject(object_name, hand_frame);
      attach_object_stage = stage.get();
      grasp->insert(std::move(stage));
    }

    task.add(std::move(grasp));
  }

  {
    // clang-format off
    auto stage = std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
    // clang-format on
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setMinMaxDistance(0.3, 0.5);
    stage->setIKFrame(hand_frame);
    stage->properties().set("marker_ns", "lift_object");

    // Set upward direction
    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = "world";
    vec.vector.z = 1.0;
    stage->setDirection(vec);
    task.add(std::move(stage));
  }

  {
    // clang-format off
    auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
        "move to place",
        mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner },
                                                  { hand_group_name, sampling_planner } });
    // clang-format on
    stage_move_to_place->setTimeout(5.0);
    stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage_move_to_place));
  }

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
      Eigen::Quaterniond q(0,1,0,0);
      grasp_frame_transform.linear() = q.matrix();
      //grasp_frame_transform.translation().z() = 6.15; // This value is how far the robot gripper will stop to attach the object to the robot.

      // Compute IK
      // clang-format off
      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
      // clang-format on
      wrapper->setMaxIKSolutions(10);
      wrapper->setMinSolutionDistance(10.0);
      wrapper->setIKFrame(grasp_frame_transform, hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      place->insert(std::move(wrapper));
    }
    insert_object_stage = place.get();
    task.add(std::move(place));
  }

  {
    // clang-format off
    auto stage_move_to_insert = std::make_unique<mtc::stages::Connect>(
        "move to insert",
        mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner },
                                                  { hand_group_name, sampling_planner } });
    // clang-format on
    stage_move_to_insert->setTimeout(5.0);
    stage_move_to_insert->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage_move_to_insert));
  }
  
  {
    auto insert_block = std::make_unique<mtc::SerialContainer>("insert object");
    task.properties().exposeTo(insert_block->properties(), { "eef", "group", "ik_frame" });
    // clang-format off
    insert_block->properties().configureInitFrom(mtc::Stage::PARENT,
                                          { "eef", "group", "ik_frame" });
    // clang-format on

    {
      // Sample place pose
      auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "place_pose");
      stage->setObject(object_name);

      target_pose_msg.pose.position.z = target_pose_msg.pose.position.z - 1.5;
      
      stage->setMonitoredStage(insert_object_stage);
      stage->setPose(target_pose_msg);

      // This is the transform from the object frame to the end-effector frame
      Eigen::Isometry3d grasp_frame_transform;
      Eigen::Quaterniond q(0,0,1,0);
      grasp_frame_transform.linear() = q.matrix();
      //grasp_frame_transform.translation().z() = 6.15; // This value is how far the robot gripper will stop to attach the object to the robot.

      // Compute IK
      // clang-format off
      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("insert pose IK", std::move(stage));
      // clang-format on
      wrapper->setMaxIKSolutions(2);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(grasp_frame_transform, hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      insert_block->insert(std::move(wrapper));
    }

    {
      // clang-format off
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision object");
      stage->allowCollisions(object_name, true);
      // clang-format on
      insert_block->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("open_hand");
      insert_block->insert(std::move(stage));
    }
    
    {
      // clang-format off
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
      stage->allowCollisions(object_name,
                             task.getRobotModel()
                                 ->getJointModelGroup(hand_group_name)
                                 ->getLinkModelNamesWithCollisionGeometry(),
                             false);
      // clang-format on
      insert_block->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject(object_name, hand_frame);
      insert_block->insert(std::move(stage));
    }

    task.add(std::move(insert_block));
  }

  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    stage->setMinMaxDistance(0.3, 0.5);
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

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]()
                                                   {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface()); });

  int assembly_plan[][6] = {{1,1,1,8,2,82},{1,3,1,2,4,42},{4,3,1,1,2,21},{3,5,1,2,2,22},{5,6,1,3,1,31},{8,6,1,1,1,11},{5,4,1,4,1,41},{6,4,2,2,3,32}};
  int assembly_size = sizeof(assembly_plan)/sizeof(assembly_plan[0]);

  for (int i = 0; i < assembly_size; i++)
  {
    environment_interface::msg::Block block;
    block.x = assembly_plan[i][0];
    block.y = assembly_plan[i][1];
    block.z = assembly_plan[i][2];
    block.x_size = assembly_plan[i][3];
    block.y_size = assembly_plan[i][4];
    block.number = assembly_plan[i][5];
    //request_block(block);
    
    mtc_task_node->doTask(block);
  }

  mtc_task_node->doReturnHome();
  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}