#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/GetPositionFK.h>
#include <moveit/move_group/capability_names.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/conversions.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/GripperCommandAction.h>
#include <robot_movement_interface/CommandsAction.h>
#include <block_printer_motion_commander/CapableNames.h>
#include <block_printer_visualizer/CapableNames.h>
#include <block_printer_visualizer/SpawnBlock.h>
#include <block_printer_visualizer/AttachBlock.h>
#include <block_printer_visualizer/DetachBlock.h>
#include <block_printer_visualizer/DeleteBlock.h>
#include <block_printer_moveit_interface/common.h>
#include <block_printer_moveit_interface/MoveitPlanner.h>
#include <block_printer_moveit_interface/ForceConverter.h>
#include <block_printer_moveit_interface/CapableNames.h>

using namespace geometry_msgs;
using namespace sensor_msgs;
using namespace control_msgs;
using namespace actionlib;
using namespace moveit::planning_interface;
using namespace robot_movement_interface;
using namespace block_printer_visualizer;
using namespace block_printer_moveit_interface;

static const double CARTESIAN_MOVE_EEF_POINT_STEP = 0.002;
static const double CARTESIAN_MOVE_JUMP_THRESHOLD = 1000.0;

static std::vector<ros::Duration> planningTimes;
static std::vector<ros::Duration> executingTimes;

template <typename T>
static bool contains(const std::vector<T> &v, const T &key)
{
    for (auto e : v)
    {
        if (e == key)
        {
            return true;
        }
    }
    return false;
}

///robot_movement_interface/Command/additional_parametersの文字列配列から最初に見つかった整数を返すために使用する
static int getBlockIndex(const std::vector<std::string> &v)
{
    for (auto e : v)
    {
        try
        {
            auto index = std::stoi(e);
            return index;
        }
        catch (std::invalid_argument)
        {
            continue;
        }
    }
    PRINT_ERROR("failed to get a block index from strings");
    return -1;
}

static Pose toGeometryPose(const std::vector<float> &pose)
{
    Pose value;
    value.position.x = pose.at(0);
    value.position.y = pose.at(1);
    value.position.z = pose.at(2);
    std::vector<float> orientation;
    for (int i = 3; i < pose.size(); i++)
    {
        orientation.push_back(pose[i]);
    }
    if (orientation.size() == 3)
    {
        value.orientation = tf::createQuaternionMsgFromRollPitchYaw(
            orientation[0],
            orientation[1],
            orientation[2]);
    }
    else if (orientation.size() == 4)
    {
        value.orientation.x = orientation[0];
        value.orientation.y = orientation[1];
        value.orientation.z = orientation[2];
        value.orientation.w = orientation[3];
    }
    else
    {
        PRINT_ERROR("invalied oriantation vector size: " << pose.size());
    }
    return value;
}

static void savePlanningAndExecutingTimes(const std::string &filename, const MoveGroupInterface &moveGroup)
{
    std::ofstream writer(filename);
    writer << "movegroup Id," << moveGroup.getName() << std::endl;
    writer << "planner Id," << moveGroup.getPlannerId() << std::endl;
    writer << "default planner Id," << moveGroup.getDefaultPlannerId() << std::endl;
    writer << "planning,executing" << std::endl;
    if (writer.fail())
    {
        PRINT_ERROR("failed to open a file \'" << filename << "\' to save planning times");
        return;
    }
    for (auto i = 0;; i++)
    {
        auto written = false;
        if (i < planningTimes.size())
        {
            writer << planningTimes[i].sec + planningTimes[i].nsec * 1e-9;
            written = true;
        }
        writer << ",";
        if (i < executingTimes.size())
        {
            writer << executingTimes[i].sec + executingTimes[i].nsec * 1e-9;
            written = true;
        }
        writer << std::endl;
        if (!written)
        {
            break;
        }
    }
    writer.close();
    PRINT_INFO("successfully save planning times to \'" << filename << "\'");
}

static MoveItErrorCode moveManipulator(const CommandsGoalConstPtr &ptr,
                                       MoveitPlanner &moveitPlanner)
{
    MoveItErrorCode value;
    ros::Duration planningTime, executingTime;
    //
    moveitPlanner.clearWaypoints();
    //
    auto command = ptr->commands.commands.front();
    //
    if (contains<std::string>(command.additional_parameters, block_printer_motion_commander::MOVEIT_INTERPOLATION_COMMAND))
    {
        if (!contains<std::string>(command.additional_parameters, block_printer_motion_commander::TO_INITIAL_POSE))
        {
            auto firstPose = moveitPlanner.getCurrentPose().pose;
            firstPose.position.z += 0.02;
            moveitPlanner.addWaypoint(InterpolationType::CARTESIAN, firstPose);
        }
        auto secondPose = toGeometryPose(command.pose);
        moveitPlanner.addWaypoint(InterpolationType::DYNAMIC, secondPose);
        //
        value = moveitPlanner.move(command.velocity.front(),
                                   1.0,
                                   CARTESIAN_MOVE_EEF_POINT_STEP,
                                   CARTESIAN_MOVE_JUMP_THRESHOLD,
                                   planningTime,
                                   executingTime);
    }
    else
    {
        auto pose = toGeometryPose(command.pose);
        moveitPlanner.addWaypoint(InterpolationType::CARTESIAN, pose);
        value = moveitPlanner.move(command.velocity.front(),
                                   1.0,
                                   CARTESIAN_MOVE_EEF_POINT_STEP,
                                   CARTESIAN_MOVE_JUMP_THRESHOLD,
                                   planningTime,
                                   executingTime);
    }
    //
    planningTimes.push_back(planningTime);
    executingTimes.push_back(executingTime);
    //
    return value;
}

static SimpleClientGoalState sendManipulatorSetting(const CommandsGoalConstPtr &ptr,
                                                    SimpleActionClient<CommandsAction> &commandsActionClient,
                                                    const MoveGroupInterface &moveGroup,
                                                    const JointState &jointState)
{
    if (!commandsActionClient.isServerConnected())
    {
        PRINT_WARN("commands action is not connected to a server");
        return SimpleClientGoalState::SUCCEEDED;
    }
    //motion_commanderから受信したコマンド列の先頭だけを取り出す
    auto command = ptr->commands.commands.front();
    //目標位置情報は使用しないので削除する
    command.pose_type = "";
    command.pose_reference = "";
    command.pose = {};
    //
    CommandsGoal goal;
    goal.commands.header = ptr->commands.header;
    goal.commands.commands = {command};
    commandsActionClient.sendGoal(goal);
    commandsActionClient.waitForResult();
    return commandsActionClient.getState();
}

static SimpleClientGoalState moveGripper(const CommandsGoalConstPtr &ptr, SimpleActionClient<GripperCommandAction> &gripperActionClient)
{
    if (!gripperActionClient.isServerConnected())
    {
        PRINT_WARN("gripper action is not connected to a server");
        return SimpleClientGoalState::SUCCEEDED;
    }
    auto command = ptr->commands.commands.front();
    GripperCommandGoal goal;
    goal.command.position = command.pose.front();
    goal.command.max_effort = command.effort.front();
    gripperActionClient.sendGoal(goal);
    if (!gripperActionClient.waitForResult(ros::Duration(60)))
    {
        PRINT_WARN("gripper action timeout");
        return SimpleClientGoalState::SUCCEEDED;
    }
    return gripperActionClient.getState();
}

static void executeAction(const CommandsGoalConstPtr &ptr,
                          SimpleActionServer<CommandsAction> &server,
                          SimpleActionClient<CommandsAction> &commandsActionClient,
                          SimpleActionClient<GripperCommandAction> &gripperActionClient,
                          MoveitPlanner &moveitPlanner,
                          const JointState &jointState)
{
    if (ptr->commands.commands.empty())
    {
        PRINT_ERROR("no command exists in action goal");
        server.setAborted();
        return;
    }
    auto front = ptr->commands.commands.front();
    if (contains<std::string>(front.additional_parameters, block_printer_motion_commander::TO_FINISH_POSE))
    {
        std::string filename = "/home/kohama/Desktop/block_printing_moveit_";
        auto time = ros::Time::now();
        filename += std::to_string(time.sec) + "_" + std::to_string(time.nsec);
        filename += ".csv";
        savePlanningAndExecutingTimes(filename, moveitPlanner.getMoveGroup());
    }
    //spawn, attach or detach block
    if (contains<std::string>(front.additional_parameters, block_printer_motion_commander::TO_PICK_APPROACH))
    {
        SpawnBlock spawnBlock;
        spawnBlock.request.block_index = getBlockIndex(front.additional_parameters);
        spawnBlock.request.reference_frame = moveitPlanner.getMoveGroup().getPoseReferenceFrame();
        if (!ros::service::call(block_printer_visualizer::SPAWN_BLOCK_SERVICE, spawnBlock))
        {
            PRINT_WARN("failed to call block spawner service. request: " << spawnBlock.request);
        }
    }
    else if (contains<std::string>(front.additional_parameters, block_printer_motion_commander::PICK_MOVE_GRIPPER))
    {
        AttachBlock attachBlock;
        attachBlock.request.attach_link = moveitPlanner.getMoveGroup().getEndEffectorLink();
        attachBlock.request.block_index = getBlockIndex(front.additional_parameters);
        if (!ros::service::call(block_printer_visualizer::ATTACH_BLOCK_SERVICE, attachBlock))
        {
            PRINT_WARN("failed to call block attacher service");
        }
    }
    else if (contains<std::string>(front.additional_parameters, block_printer_motion_commander::PLACE_MOVE_GRIPPER))
    {
        DetachBlock detachBlock;
        if (!ros::service::call(block_printer_visualizer::DETACH_BLOCK_SERVICE, detachBlock))
        {
            PRINT_WARN("failed to call block detacher service");
        }
    }
    //move manipulator or gripper
    auto commandType = front.command_type;
    if (commandType == ros::param::param<std::string>("/block_printer/motion_commander/manipulator/command_type", ""))
    {
        sendManipulatorSetting(ptr, commandsActionClient, moveitPlanner.getMoveGroup(), jointState);
        auto errorCode = moveManipulator(ptr, moveitPlanner);
        if (!errorCode)
        {
            server.setAborted();
        }
        else
        {
            server.setSucceeded();
        }
        PRINT_INFO("set action result");
    }
    else if (commandType == ros::param::param<std::string>("/block_printer/motion_commander/gripper/command_type", ""))
    {
        auto state = moveGripper(ptr, gripperActionClient);
        if (state == SimpleClientGoalState::SUCCEEDED)
        {
            server.setSucceeded();
        }
        else
        {
            server.setAborted();
        }
    }
    else
    {
        ROS_ERROR_STREAM("unexpected command type: " << commandType);
        server.setAborted();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "block_printer_moveit_interface");
    ros::NodeHandle node;
    JointState jointState;
    //moveitの動作計画，fk，ikを使用するために必要
    ros::AsyncSpinner spinner(2);
    spinner.start();
    //
    auto moveGroupName = ros::param::param<std::string>("/block_printer/moveit_interface/manipulator/move_group", "");
    auto referenceFrame = ros::param::param<std::string>("/block_printer/moveit_interface/manipulator/reference_frame", "");
    auto endEffectorLink = ros::param::param<std::string>("/block_printer/moveit_interface/manipulator/tool_link", "");
    MoveitPlanner moveitPlanner(node, moveGroupName, referenceFrame, endEffectorLink);
    //
    auto callback = boost::function<void(const JointState &)>([&jointState](const JointState &subscription) { jointState = subscription; });
    auto jointStateSubscriber = node.subscribe<JointState>("/joint_states", 10, callback);
    //
    SimpleActionClient<CommandsAction> commandsActionClient(COMMANDS_ACTION);
    SimpleActionClient<GripperCommandAction> gripperCommandActionClient(GRIPPER_ACTION);
    SimpleActionServer<CommandsAction> commandsActionServer(node,
                                                            block_printer_motion_commander::MOTION_COMMAND_ACTION,
                                                            boost::bind(executeAction, _1,
                                                                        boost::ref(commandsActionServer),
                                                                        boost::ref(commandsActionClient),
                                                                        boost::ref(gripperCommandActionClient),
                                                                        boost::ref(moveitPlanner),
                                                                        boost::ref(jointState)),
                                                            false);
    commandsActionServer.start();
    PRINT_INFO("ready to plan!");
    ros::waitForShutdown();
    return 0;
}
