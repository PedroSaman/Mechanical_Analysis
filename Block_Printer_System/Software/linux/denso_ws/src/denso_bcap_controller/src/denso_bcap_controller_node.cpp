#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <block_printer_motion_commander/CapableNames.h>
#include <denso_bcap_controller/macro.h>
#include <denso_bcap_controller/function.h>
#include <denso_bcap_controller/Manipulator.h>
#include <denso_bcap_controller/Hand.h>
#include <denso_bcap_controller/FollowJointTrajectoryInterface.h>
#include <denso_bcap_controller/GripperCommandInterface.h>
#include <denso_bcap_controller/CommandsActionInterface.h>
#include <denso_bcap_controller/CapableNames.h>

using namespace sensor_msgs;
using namespace control_msgs;
using namespace actionlib;
using namespace robot_movement_interface;
using namespace denso_bcap_controller;

static bool executeFollowJointTrajectory(FollowJointTrajectoryInterfaceRequest &request,
                                         FollowJointTrajectoryInterfaceResponse &response,
                                         Manipulator &manipulator,
                                         const ros::Publisher &jointStatePublisher,
                                         bool enableTrajectoryFilter)
{
    return manipulator.executeAction(request.trajectory, jointStatePublisher, enableTrajectoryFilter);
}

static bool executeGripperCommand(GripperCommandInterfaceRequest &request, GripperCommandInterfaceResponse &response, Hand &hand)
{
    return hand.executeCommand(request.command);
}

static bool executeCommandsAction(CommandsActionInterfaceRequest &request, CommandsActionInterfaceResponse &response,
                                  Manipulator &manipulator)
{
    {
        auto processIndex = request.command.additional_parameters.front();
        auto enablePreciseGoal = processIndex == block_printer_motion_commander::TO_PICK_APPROACH || processIndex == block_printer_motion_commander::TO_PLACE_APPROACH;
        manipulator.setPreciseGoal(enablePreciseGoal);
    }
    if (request.command.velocity_type != "factor")
    {
        PRINT_WARN("velocity factor is not factor");
        PRINT_WARN("no velocity change will be applied");
    }
    else
    {
        manipulator.setVelocityFactor(request.command.velocity.front());
    }
    if (request.command.force_threshold_type != "percent")
    {
        PRINT_WARN("you must specity force threshold by 'percent'. actual command:" << request.command);
        PRINT_WARN("no force threshold change will be applied");
        return false;
    }
    auto forces = request.command.force_threshold;
    while (forces.size() < 6)
    {
        forces.push_back(100.0);
    }
    manipulator.setForceLimitPercents(forces[0], forces[1], forces[2], forces[3], forces[4], forces[5]);
    return true;
}

int main(int argc, char **argv)
{
    //initialize ros node
    ros::init(argc, argv, "denso_bcap_controller");
    ros::NodeHandle node;
    ros::NodeHandle paramNode("~");
    //load paramters
    auto robotIp = paramNode.param<std::string>("robot_ip", "0.0.0.0");
    auto robotPort = paramNode.param<std::string>("robot_port", "0");
    auto connectingMode = paramNode.param<std::string>("connecting_mode", "TCP");
    auto bcapConnectingMode = (connectingMode == "TCP") ? BCapNet::ConnectingMode::BCAP_TCP : BCapNet::ConnectingMode::BCAP_UDP;
    auto useSlaveMode = paramNode.param<bool>("use_slave_mode", false);
    auto useTrajectoryReducer = paramNode.param<bool>("use_trajectory_reducer", false);
    auto useHand = paramNode.param<bool>("use_hand", false);
    auto readonly = paramNode.param<bool>("readonly", true);
    auto robotDescriptionName = paramNode.param<std::string>("robot_description_name", "robot_description");
    //start b-cap communication
    PRINT_INFO("connecting to " << robotIp << " (" << robotPort << ", " << connectingMode << ")...");
    RcController controller(robotIp, robotPort, bcapConnectingMode, readonly);
    //publisher
    auto jointStatePublisher = node.advertise<JointState>(denso_bcap_controller::INTERNAL_JOINT_STATE, 10);
    //initialize robot
    PRINT_INFO("initializing a manipulator..");
    Manipulator manipulator(controller, robotDescriptionName);
    auto setSlaveModeResult = manipulator.setSlaveMode(useSlaveMode && !readonly);
    if (!isBcapSuccess(setSlaveModeResult))
    {
        PRINT_ERROR("failed to set slave-mode of the manipulator: " << getBcapMessage(setSlaveModeResult));
        ros::shutdown();
        return -1;
    }
    PRINT_INFO("initializing a hand..");
    Hand hand(controller, useHand && !readonly);
    //create service server
    auto followJointTrajectoryServer = node.advertiseService<FollowJointTrajectoryInterfaceRequest, FollowJointTrajectoryInterfaceResponse>(denso_bcap_controller::JOINT_TRAJECTORY_SERVICE, boost::bind(executeFollowJointTrajectory, _1, _2, boost::ref(manipulator), boost::ref(jointStatePublisher), useTrajectoryReducer));
    auto gripperCommandServer = node.advertiseService<GripperCommandInterfaceRequest, GripperCommandInterfaceResponse>(denso_bcap_controller::GRIPPER_SERVICE, boost::bind(executeGripperCommand, _1, _2, boost::ref(hand)));
    auto commandsActionServer = node.advertiseService<CommandsActionInterfaceRequest, CommandsActionInterfaceResponse>(denso_bcap_controller::COMMANDS_SERVICE, boost::bind(executeCommandsAction, _1, _2, boost::ref(manipulator)));
    //
    PRINT_INFO("ready");
    //
    while (ros::ok())
    {
        //error check
        if (controller.hasError())
        {
            PRINT_ERROR(controller.getErrorMessage());
            break;
        }
        //publish current joint state
        auto jointState = manipulator.getJointState();
        if (!isBcapSuccess(jointState.second))
        {
            PRINT_ERROR("while getting feedback from robot: " << getBcapMessage(jointState.second));
            break;
        }
        jointStatePublisher.publish(jointState.first);
        //
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
    return 0;
}
