#include <memory>
#include <stdlib.h>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <unistd.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <cmath>
#include "boost/multi_array.hpp"

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "joint_states_reproducer",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );
  // Create a ROS logger
  auto const logger = rclcpp::get_logger("joint_states_reproducer");

  std::string fname = "../Block_Printer_System/Models/plan/joint_states.csv";
	std::vector<std::vector<std::string>> content;
	std::vector<std::string> row;
	std::string line, word;
	std::fstream file (fname, std::ios::in);

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "cobotta_arm");
  move_group_interface.setMaxAccelerationScalingFactor(1.0);
  move_group_interface.setMaxVelocityScalingFactor(1.0);

  if(file.is_open())
	{
		while(getline(file, line))
		{
			row.clear();
 
			std::stringstream str(line);
 
			while(getline(str, word, '"'))
				row.push_back(word);
			content.push_back(row);
		}
	}
	else
		std::cout<<"Could not open the file\n";

  size_t content_size = content.size();
  double joint_values[7];

	for(size_t i=1;i<content_size;i++) //jump first row
	{
    line.clear();
    line = content[i][5];
    for(size_t k=line.size(); k>=1; k--) //Remove bad caracters
    {
      if(line[k-1] == '[' || line[k-1] == ']' )
      {
        line.erase(k-1,1);
      }
    }
    size_t index = 0;
    size_t joint_number = 0;
    std::string buffer;
    //std::cout << "line: " << line << std::endl;
    for(size_t j = 0; j<line.size(); j++)
    {
      if(line[j] != ',') //New number delimiter
      {
        index++;
      }else
      {
        buffer = line.substr((j - index), index);
        //std::cout << "buffer: " << buffer << std::endl;
        
        joint_values[joint_number] = std::stod(buffer);
        buffer.clear();
        index = 0;
        if(joint_number == 6)
        {
          joint_number = 0;
          break;
        }else 
        {
          joint_number++;
        }
      }
	  }
    std::map<std::string, double> variable_values;
    variable_values.insert(std::pair<std::string, double>("joint_1", joint_values[0]));
    variable_values.insert(std::pair<std::string, double>("joint_2", joint_values[1]));
    variable_values.insert(std::pair<std::string, double>("joint_3", joint_values[4]));
    variable_values.insert(std::pair<std::string, double>("joint_4", joint_values[2]));
    variable_values.insert(std::pair<std::string, double>("joint_5", joint_values[3]));
    variable_values.insert(std::pair<std::string, double>("joint_6", joint_values[5]));
    
    std::cout << "time_stamp: " << i <<"/" << content_size << std::endl;
    for (auto j = variable_values.begin(); j != variable_values.end(); j++)
      std::cout << j->first << "	 " << j->second << std::endl;

    move_group_interface.setJointValueTarget(variable_values);

    // Create a plan to that target pose
    
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto const success = static_cast<bool>(move_group_interface.plan(plan));

    // Execute the plan
    if(success) {
      move_group_interface.execute(plan);
    } else {
      RCLCPP_ERROR(logger, "Planing failed!");
    }
  }
  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}