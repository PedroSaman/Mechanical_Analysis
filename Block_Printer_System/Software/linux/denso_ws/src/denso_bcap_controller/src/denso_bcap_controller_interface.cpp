#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/GripperCommandAction.h>
#include <robot_movement_interface/CommandsAction.h>
#include <block_printer_moveit_interface/CapableNames.h>
#include <denso_bcap_controller/macro.h>
#include <denso_bcap_controller/FollowJointTrajectoryInterface.h>
#include <denso_bcap_controller/GripperCommandInterface.h>
#include <denso_bcap_controller/CommandsActionInterface.h>
#include <denso_bcap_controller/CapableNames.h>

using namespace sensor_msgs;
using namespace control_msgs;
using namespace actionlib;
using namespace robot_movement_interface;
using namespace denso_bcap_controller;

static JointState jointState;

static void executeJointTrajectoryAction(const FollowJointTrajectoryGoalConstPtr &goal,
                                         SimpleActionServer<FollowJointTrajectoryAction> &server)
{
    FollowJointTrajectoryInterface service;
    service.request.trajectory = goal->trajectory;
    if (ros::service::call(denso_bcap_controller::JOINT_TRAJECTORY_SERVICE, service))
    {
        server.setSucceeded();
    }
    else
    {
        server.setAborted();
    }
}

static void executeGripperAction(const GripperCommandGoalConstPtr &goal,
                                 SimpleActionServer<GripperCommandAction> &server)
{
    GripperCommandInterface service;
    service.request.command = goal->command;
    if (ros::service::call(denso_bcap_controller::GRIPPER_SERVICE, service))
    {
        server.setSucceeded();
    }
    else
    {
        server.setAborted();
    }
}

static void executeCommandsAction(const CommandsGoalConstPtr &goal,
                                  SimpleActionServer<CommandsAction> &server)
{
    CommandsActionInterface service;
    service.request.command = goal->commands.commands.front();
    if (ros::service::call(denso_bcap_controller::COMMANDS_SERVICE, service))
    {
        server.setSucceeded();
    }
    else
    {
        server.setAborted();
    }
}

static void jointStateCallback(const JointState &topic)
{
    jointState = topic;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "denso_bcap_joint_state_publisher");
    ros::NodeHandle node;
    auto jointStatePublisher = node.advertise<JointState>("/joint_states", 10);
    auto subscriber = node.subscribe(denso_bcap_controller::INTERNAL_JOINT_STATE, 10, jointStateCallback);
    ros::Rate sleeper(50);
    //create action server
    SimpleActionServer<FollowJointTrajectoryAction> followJointTrajectoryServer(node,
                                                                                "/denso_bcap_controller/follow_joint_trajectory",
                                                                                boost::bind(executeJointTrajectoryAction, _1, boost::ref(followJointTrajectoryServer)),
                                                                                false);
    SimpleActionServer<CommandsAction> commandsActionServer(node,
                                                            block_printer_moveit_interface::COMMANDS_ACTION,
                                                            boost::bind(executeCommandsAction, _1, boost::ref(commandsActionServer)),
                                                            false);

    SimpleActionServer<GripperCommandAction> gripperCommandServer(node,
                                                                  block_printer_moveit_interface::GRIPPER_ACTION,
                                                                  boost::bind(executeGripperAction, _1, boost::ref(gripperCommandServer)),
                                                                  false);
    //
    followJointTrajectoryServer.start();
    commandsActionServer.start();
    gripperCommandServer.start();
    PRINT_INFO("ready");
    //
    while (ros::ok())
    {
        jointState.header.stamp = ros::Time::now();
        jointStatePublisher.publish(jointState);
        ros::spinOnce();
        sleeper.sleep();
    }
    return 0;
}
