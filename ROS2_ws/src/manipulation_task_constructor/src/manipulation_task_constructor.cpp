#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
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

static const rclcpp::Logger LOGGER = rclcpp::get_logger("manipulation_task_constructor");
namespace mtc = moveit::task_constructor;

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions &options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask(size_t block_number);

private:
  // Compose an MTC task from a series of stages.
  mtc::Task createTask(size_t block_number);
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

void MTCTaskNode::doTask(size_t block_number)
{
  for (size_t i = 1; i <= block_number; i++)
  {
    task_ = createTask(i);
    try
    {
      task_.init();
    }
    catch (mtc::InitStageException &e)
    {
      RCLCPP_ERROR_STREAM(LOGGER, e);
      return;
    }

    if (!task_.plan(2))
    {
      RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
      return;
    }
    //task_.introspection().publishSolution(*task_.solutions().front());
    //task_.execute(*task_.solutions().front());
    moveit_msgs::msg::MoveItErrorCodes execute_result;
    execute_result = task_.execute(*task_.solutions().front());
    if (execute_result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed and returned: " << execute_result.val);
      return;
	  }
  }
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

  if (!task_.plan(1))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }
  //task_.introspection().publishSolution(*task_.solutions().front());
  //task_.execute(*task_.solutions().front());
  moveit_msgs::msg::MoveItErrorCodes execute_result;
  execute_result = task_.execute(*task_.solutions().front());
  if (execute_result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed and returned: " << execute_result.val);
    return;
  }
  return;
}

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

mtc::Task MTCTaskNode::createTask(size_t block_number)
{
  mtc::Task task;

  std::string table_name = "table";
  std::string s = std::to_string(block_number);
  std::string object_name = "block_" + s;

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
  mtc::Stage* attach_object_stage =
      nullptr;  // Forward attach_object_stage to place pose generator
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
      stage->setMinMaxDistance(0.01, 0.05);

      // Set hand forward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame;
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    /****************************************************
  ---- *               Generate Grasp Pose                *
     ***************************************************/
    {
      // Sample grasp pose
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      stage->setPreGraspPose("open_hand");
      stage->setObject(object_name);
      stage->setObject(object_name);
      stage->setAngleDelta(M_PI / 2);
      stage->setMonitoredStage(current_state_ptr); // Hook into current state

      // This is the transform from the object frame to the end-effector frame
      Eigen::Isometry3d grasp_frame_transform;
      Eigen::Quaterniond q(0,1,0,0);
      grasp_frame_transform.linear() = q.matrix();
      grasp_frame_transform.translation().z() = 0.066; //

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
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("close_hand");
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
    stage->setMinMaxDistance(0.01, 0.05);
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

    /****************************************************
  ---- *               Generate Place Pose                *
     ***************************************************/
    {
      // Sample place pose
      auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "place_pose");
      stage->setObject(object_name);

      float pos = static_cast< float >(block_number)*1.25/100;
      geometry_msgs::msg::PoseStamped target_pose_msg;
      target_pose_msg.header.frame_id = object_name;
      target_pose_msg.pose.position.x = 0.05 + pos;
      stage->setPose(target_pose_msg);
      stage->setMonitoredStage(attach_object_stage); // Hook into attach_object_stage

      // This is the transform from the object frame to the end-effector frame
      Eigen::Isometry3d grasp_frame_transform;
      Eigen::Quaterniond q(0,1,0,0);
      grasp_frame_transform.linear() = q.matrix();
      //grasp_frame_transform.translation().z() = 0.065; // This value is how far the robot gripper will stop to attach the object to the robot.

      // Compute IK
      // clang-format off
      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
      // clang-format on
      wrapper->setMaxIKSolutions(2);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(grasp_frame_transform, hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      place->insert(std::move(wrapper));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("open_hand");
      place->insert(std::move(stage));
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
      place->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject(object_name, hand_frame);
      place->insert(std::move(stage));
    }

    task.add(std::move(place));
  }

  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    stage->setMinMaxDistance(0.01, 0.05);
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
    executor.remove_node(mtc_task_node->getNodeBaseInterface()); });

  size_t count = 6;
  mtc_task_node->doTask(count);


  spin_thread->join();  
  rclcpp::shutdown();
  return 0;
}