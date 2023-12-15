#include "environment_information.h"
#include "environment_interface/action/task_creator.hpp"
#include "environment_interface/srv/get_block_color.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>

class TaskCreatorActionClient : public rclcpp::Node
{
public:
  using TaskCreator = environment_interface::action::TaskCreator;
  using GoalHandleTaskCreator = rclcpp_action::ClientGoalHandle<TaskCreator>;

  explicit TaskCreatorActionClient(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : Node("task_creator_action_client", node_options), goal_done_(false)
  {
    this->client_ptr_ = rclcpp_action::create_client<TaskCreator>(
      this->get_node_base_interface(),
      this->get_node_graph_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "task_creator");
  }
  
  void input_arguments_setup();

  void setup_timer(environment_interface::msg::Block block,size_t operation_id,std::string planner_id)
  {
    this->timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        [this, block, operation_id, planner_id]() {
            // Call the new send_goal function with the arguments
            this->send_goal(block, operation_id, planner_id);
        }
      );
    this->goal_done_ = false;
  }

  bool is_goal_done() const
  {
    return this->goal_done_;
  }

  void send_goal(environment_interface::msg::Block block, size_t operation_id, std::string planner_id)
  {
    using namespace std::placeholders;

    this->timer_->cancel();

    this->goal_done_ = false;

    if (!this->client_ptr_) {
      RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
    }

    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      this->goal_done_ = true;
      return;
    }

    auto goal_msg = TaskCreator::Goal();

    goal_msg.planner_id = planner_id;
    goal_msg.block = block;
    goal_msg.operation_id = operation_id;
    goal_msg.robot_name = robot_name_;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<TaskCreator>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&TaskCreatorActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&TaskCreatorActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&TaskCreatorActionClient::result_callback, this, _1);
    auto goal_handle_future = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

  std::string getRobotName() const {
    return robot_name_;
  }

  size_t getFirstBlock() const {
    return first_block_;
  }

  std::vector<std::vector<int>> getAssemblyPlan() {
    return assembly_plan_;
  }

  void setRobotName(const std::string& new_robot_name) {
    robot_name_ = new_robot_name;
  }

  void setFirstBlock(const size_t first_block) {
    first_block_ = first_block;
  }

  void setAssemblyPlan(const std::vector<std::vector<int>> assembly_plan) {
    assembly_plan_ = assembly_plan;
  }

  std::vector<std::vector<int>> getAssemblyPlan() const {
    return assembly_plan_;
  }

  environment_interface::msg::Block prepare_block_structure(size_t current_block)
  {  
    environment_interface::msg::Block block;
    block.x = assembly_plan_[current_block][1];
    block.y = assembly_plan_[current_block][2];
    block.z = assembly_plan_[current_block][3];
    if(assembly_plan_[current_block][8] == 1)
    {
      block.color = getColor(SUPPORT);
      block.is_support = true;
    }else 
    {
      block.color = getColor(assembly_plan_[current_block][7]);
      block.is_support = false;
    }
    block.number = current_block+1;
    
    if((assembly_plan_[current_block][4] == 2 && assembly_plan_[current_block][5] == 1) || (assembly_plan_[current_block][4] == 1 && assembly_plan_[current_block][5] == 2)) //Special case for 2x1 block due its different positioning in the parts feeder
    {
      block.name = "21";
      block.x_size = 2;
      block.y_size = 1;
    }else if(assembly_plan_[current_block][4] < assembly_plan_[current_block][5])
    {
      block.name = std::to_string(assembly_plan_[current_block][4]) + std::to_string(assembly_plan_[current_block][5]);
      block.x_size = assembly_plan_[current_block][4];
      block.y_size = assembly_plan_[current_block][5];
    }else
    {
      block.name = std::to_string(assembly_plan_[current_block][5]) + std::to_string(assembly_plan_[current_block][4]);
      block.x_size = assembly_plan_[current_block][5];
      block.y_size = assembly_plan_[current_block][4];
    }

    block.x_size = assembly_plan_[current_block][4]; // Correct the sizes for the assembly
    block.y_size = assembly_plan_[current_block][5];

    return block;
  }

  std_msgs::msg::ColorRGBA getColor(int color_index)
  {
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("block_service_client");  
    rclcpp::Client<environment_interface::srv::GetBlockColor>::SharedPtr client =                
      node->create_client<environment_interface::srv::GetBlockColor>("get_block_color_service");          

    std_msgs::msg::ColorRGBA color;
    auto request = std::make_shared<environment_interface::srv::GetBlockColor::Request>();       
    
    request->index = color_index;
    while (!client->wait_for_service(std::chrono::seconds(1))) {
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

private:
  rclcpp_action::Client<TaskCreator>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool goal_done_;
  std::string robot_name_;
  std::vector<std::vector<int>> assembly_plan_;
  size_t first_block_;
  std::string myFilePath;

  void goal_response_callback(GoalHandleTaskCreator::SharedPtr goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(GoalHandleTaskCreator::SharedPtr, const std::shared_ptr<const TaskCreator::Feedback> feedback)
  {
    std::stringstream ss;
    ss << "Comptuing the Task:";
    ss << feedback->partial_output;
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }
  
  void result_callback(const GoalHandleTaskCreator::WrappedResult & result)
  {
    this->goal_done_ = true;
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    std::stringstream ss;
    ss << "Result received: ";
    ss << result.result->task_resposnse;
    if(result.result->task_resposnse != "Everything should have went right, yahooo")
    {
      RCLCPP_ERROR(this->get_logger(), "Task FAILED");
      sleep(10000);
    }
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    //rclcpp::shutdown();
  }
};  // class TaskCreatorActionClient

void call_service(int operation_id, environment_interface::msg::Block block, std::shared_ptr<TaskCreatorActionClient> action_client)
{
  std::string planner_id = "RRTConnectkConfigDefault";
  action_client->setup_timer(block, operation_id, planner_id);
  while (!action_client->is_goal_done()) {
    rclcpp::spin_some(action_client->shared_from_this());
  }
  //rclcpp::shutdown();
}

void TaskCreatorActionClient::input_arguments_setup()
{
  std::string myFilePath = this->get_parameter("csv_file_path").as_string();
  std::string robot_name = this->get_parameter("robot_name").as_string();
  size_t first_block = this->get_parameter("first_block").as_int();
  //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "robot_name: %s", robot_name.c_str());
  //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "csv file path: %s", myFilePath.c_str());
  //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "first_block: %ld", first_block);
  this->setRobotName(robot_name);
  this->setFirstBlock(first_block);

  //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "parameter name: %s", robot_name.c_str());
  std::ifstream assemblyfile;
  assemblyfile.open(myFilePath);

  std::vector<std::vector<int>> assembly_plan;
  if(assemblyfile.fail())
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Csv file not found.");
    return;
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
  this->setAssemblyPlan(assembly_plan);
}

moveit_msgs::msg::CollisionObject request_block(environment_interface::msg::Block& block)
{ 
  moveit_msgs::msg::CollisionObject replacement_block;
  moveit::planning_interface::PlanningSceneInterface psi;
  moveit_msgs::msg::CollisionObject old_block;
  moveit_msgs::msg::CollisionObject new_block;
  std::vector<std::string> old_block_name_vector = {block.name,"/0"};
  std::map<std::string, moveit_msgs::msg::CollisionObject> old_block_name_map;
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  
  old_block_name_map = psi.getObjects(old_block_name_vector);

  old_block = old_block_name_map.begin()->second;
  new_block = old_block;
  replacement_block = old_block; //save old block to restore later

  old_block.operation = old_block.REMOVE;
  collision_objects.insert(collision_objects.end(),old_block);
  block.name = "block_" + std::to_string(block.number);

  if(block.is_support)
  {
    new_block.id = block.name;
    new_block.header.frame_id = "world";
    new_block.primitives.resize(1);
    new_block.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    new_block.primitives[0].dimensions = { block_size_z - minimum_resolution, block_size_x/2 - minimum_resolution };
    new_block.pose = old_block.pose;
    new_block.operation = new_block.ADD;
  }else{
    new_block.id = block.name;
    new_block.header.frame_id = "world";
    new_block.operation = new_block.ADD;
  }
  collision_objects.insert(collision_objects.end(),new_block);
  psi.applyCollisionObjects(collision_objects);

  return replacement_block;
}

void correct_color(environment_interface::msg::Block block)
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

void refil_block(moveit_msgs::msg::CollisionObject refil_object, std_msgs::msg::ColorRGBA refil_color)
{
  //std_msgs::msg::ColorRGBA refil_color = this->getColor(GRAY);
  refil_object.operation = refil_object.ADD;
  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(refil_object, refil_color);
}

int main(int argc, char **argv)
{
  auto start = std::chrono::high_resolution_clock::now();
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto action_client = std::make_shared<TaskCreatorActionClient>(options);
  action_client->input_arguments_setup();

  std::vector<std::vector<int>> assembly_plan = action_client->getAssemblyPlan();
  std::string robot_name = action_client->getRobotName();
  size_t first_block = action_client->getFirstBlock();

  environment_interface::msg::Block block;
  moveit_msgs::msg::CollisionObject replacement_block;
  for (size_t current_block = first_block - 1; current_block < assembly_plan.size(); current_block++)
  {
    block = action_client->prepare_block_structure(current_block);
    replacement_block = request_block(block);
    for (size_t operation_id = 1; operation_id < 9; operation_id++)
    { 
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\nSending operation %ld to block %ld", operation_id, current_block);
      call_service(operation_id, block, action_client);
      sleep(0.5);
    }
    std_msgs::msg::ColorRGBA refil_color = action_client->getColor(GRAY);
    refil_block(replacement_block, refil_color);
    correct_color(block);
  }
  call_service(GO_HOME, block, action_client);
  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Assembly time was: %.2f minutes", static_cast< float >(duration.count())/60000000);
  rclcpp::shutdown();
  return 0;
}