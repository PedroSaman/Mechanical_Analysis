#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <block_printer_model_reader/GetBlockState.h>
#include <block_printer_model_reader/GetComponentBlock.h>
#include <block_printer_pick/GetPickPose.h>
#include <block_printer_place/GetPlacePose.h>
#include <robot_movement_interface/CommandsAction.h>
#include <block_printer_model_reader/CapableNames.h>
#include <block_printer_pick/CapableNames.h>
#include <block_printer_place/CapableNames.h>
#include <block_printer_motion_commander/CapableNames.h>
#include <block_printer_motion_commander/Pose.h>

using namespace actionlib;
using namespace block_printer_model_reader;
using namespace block_printer_pick;
using namespace block_printer_place;
using namespace robot_movement_interface;
using namespace block_printer_motion_commander;

static const std::string logHeader = "motion commander";

#define PRINT_INFO(args) ROS_INFO_STREAM(logHeader << ": " << args)
#define PRINT_WARN(args) ROS_WARN_STREAM(logHeader << ": " << args)
#define PRINT_ERROR(args) ROS_ERROR_STREAM(logHeader << ": " << args)

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

static void getVelocitySettings(bool highVelocity, std::string &velocityType, std::vector<float> &velocities)
{
    velocityType = ros::param::param<std::string>("/block_printer/motion_commander/manipulator/velocity/type", "");
    velocities.clear();
    if (highVelocity)
    {
        velocities = ros::param::param<std::vector<float>>("/block_printer/motion_commander/manipulator/velocity/high", {});
    }
    else
    {
        velocities = ros::param::param<std::vector<float>>("/block_printer/motion_commander/manipulator/velocity/low", {});
    }
}

static void getAccelerationSettings(bool highAcceleration, std::string &accelerationType, std::vector<float> &accelerations)
{
    accelerationType = ros::param::param<std::string>("/block_printer/motion_commander/manipulator/acceleration/type", "");
    accelerations.clear();
    if (highAcceleration)
    {
        accelerations = ros::param::param<std::vector<float>>("/block_printer/motion_commander/manipulator/acceleration/high", {});
    }
    else
    {
        accelerations = ros::param::param<std::vector<float>>("/block_printer/motion_commander/manipulator/acceleration/low", {});
    }
}

static void getForceThresholdSettings(bool highForceThreshold, std::string &forceThresholdType, std::vector<float> &forceThresholds)
{
    forceThresholdType = ros::param::param<std::string>("/block_printer/motion_commander/manipulator/force_threshold/type", "");
    forceThresholds.clear();
    if (highForceThreshold)
    {
        forceThresholds = ros::param::param<std::vector<float>>("/block_printer/motion_commander/manipulator/force_threshold/high", {});
    }
    else
    {
        forceThresholds = ros::param::param<std::vector<float>>("/block_printer/motion_commander/manipulator/force_threshold/low", {});
    }
}

static Command getManipulatorCommand(const Pose &pose,
                                     int blockIndex,
                                     const ProcessIndex &processIndex,
                                     bool highVelocity,
                                     bool highAcceleration,
                                     bool highForceThreshold,
                                     bool useMoveit)
{
    Command value;
    value.command_id = 0;
    value.command_type = ros::param::param<std::string>("/block_printer/motion_commander/manipulator/command_type", "");
    value.pose_reference = ros::param::param<std::string>("/block_printer/motion_commander/manipulator/pose_reference", "");
    value.pose_type = ros::param::param<std::string>("/block_printer/motion_commander/manipulator/pose_type", "");
    pose.toVector<float>(value.pose);
    getVelocitySettings(highVelocity, value.velocity_type, value.velocity);
    getAccelerationSettings(highAcceleration, value.acceleration_type, value.acceleration);
    getForceThresholdSettings(highForceThreshold, value.force_threshold_type, value.force_threshold);
    value.additional_parameters.push_back(processIndex);
    value.additional_parameters.push_back(std::to_string(blockIndex));
    if (useMoveit)
    {
        value.additional_parameters.push_back(MOVEIT_INTERPOLATION_COMMAND);
    }
    return value;
}

static Command getGripperCommand(int blockSize,
                                 bool enableMargin,
                                 int blockIndex,
                                 const ProcessIndex &processIndex)
{
    Command value;
    value.command_id = 0;
    value.command_type = ros::param::param<std::string>("/block_printer/motion_commander/gripper/command_type", "");
    auto pose = ros::param::param<std::vector<float>>("/block_printer/motion_commander/gripper/poses", {}).at(blockSize);
    if (enableMargin)
    {
        pose += ros::param::param<float>("/block_printer/motion_commander/gripper/margin", 0);
    }
    value.pose.push_back(pose);
    value.effort_type = ros::param::param<float>("/block_printer/motion_commander/gripper/effort_type", 0);
    value.effort.push_back(ros::param::param<float>("/block_printer/motion_commander/gripper/effort", 0));
    value.additional_parameters.push_back(processIndex);
    value.additional_parameters.push_back(std::to_string(blockIndex));
    return value;
}

static CommandsGoal getCommandsGoal(const Command &command, bool replacePreviousCommands = false)
{
    static int seq = 0;
    CommandsGoal goal;
    goal.commands.header.seq = seq++;
    goal.commands.header.stamp = ros::Time::now();
    goal.commands.replace_previous_commands = replacePreviousCommands;
    goal.commands.commands = {command};
    return goal;
}

static CommandsGoal getFirstAction()
{
    auto initialPosition = ros::param::param<std::vector<double>>("/block_printer/motion_commander/manipulator/initial_pose/position", {});
    auto initialOrientation = ros::param::param<std::vector<double>>("/block_printer/motion_commander/manipulator/initial_pose/orientation", {});
    Pose pose(initialPosition, initialOrientation);
    auto command = getManipulatorCommand(pose, -1, TO_INITIAL_POSE, false, true, true, true);
    return getCommandsGoal(command);
}

static CommandsGoal getFinishAction()
{
    auto finishPosition = ros::param::param<std::vector<double>>("/block_printer/motion_commander/manipulator/finish_pose/position", {});
    auto finishOrientation = ros::param::param<std::vector<double>>("/block_printer/motion_commander/manipulator/finish_pose/orientation", {});
    Pose pose(finishPosition, finishOrientation);
    auto command = getManipulatorCommand(pose, -1, TO_FINISH_POSE, false, true, true, true);
    return getCommandsGoal(command);
}

static bool getActionGoal(int blockIndex, ProcessIndex processIndex, CommandsGoal &goal, int &nextBlockIndex, ProcessIndex &nextProcess)
{
    static int currentBlockIndex = INT_MIN;
    static GetBlockState getBlockState;
    static GetComponentBlock getComponentBlock;
    static int blockSizeMin;
    static GetPickPose getPickPose;
    static GetPlacePose getPlacePoseWithShift;
    static GetPlacePose getPlacePoseWithoutShift;
    //次に扱うブロックの情報を更新
    if (currentBlockIndex != blockIndex)
    {
        PRINT_INFO("update block info. current index:" << currentBlockIndex << " next:" << blockIndex);
        currentBlockIndex = blockIndex;
        //ブロックサイズとピック・プレイス時のポーズを取得
        if (!ros::service::call(block_printer_model_reader::BLOCK_STATE_SERVICE, getBlockState))
        {
            PRINT_ERROR("failed to get block state via ros service");
            return false;
        }
        getComponentBlock.request.block_index = blockIndex;
        if (!ros::service::call<GetComponentBlock>(block_printer_model_reader::COMPONENT_BLOCK_SERVICE, getComponentBlock))
        {
            PRINT_WARN("failed to get component block state via ros service");
            PRINT_INFO("all blocks have been assembled.");
            //GetComponentBlockサービスの失敗はすべてのブロックが組み立てられたということなので，終了用モーションを生成する
            goal = getFinishAction();
            return true;
        }
        blockSizeMin = std::min(getComponentBlock.response.size_x, getComponentBlock.response.size_y);
        getPickPose.request.block_index = blockIndex;
        if (!ros::service::call<GetPickPose>(block_printer_pick::PICK_POSE_SERVICE, getPickPose))
        {
            PRINT_ERROR("failed to get pick pose via ros service");
            return false;
        }
        getPlacePoseWithShift.request.assembly_pad_index = getComponentBlock.response.assembly_pad_index;
        getPlacePoseWithoutShift.request.assembly_pad_index = getComponentBlock.response.assembly_pad_index;
        getPlacePoseWithShift.request.block_index = blockIndex;
        getPlacePoseWithoutShift.request.block_index = blockIndex;
        getPlacePoseWithShift.request.enable_shift = true;
        getPlacePoseWithoutShift.request.enable_shift = false;
        if (!ros::service::call<GetPlacePose>(block_printer_place::PLACE_POSE_SERVICE, getPlacePoseWithShift))
        {
            PRINT_ERROR("failed to get place pose via ros service");
            return false;
        }
        if (!ros::service::call<GetPlacePose>(block_printer_place::PLACE_POSE_SERVICE, getPlacePoseWithoutShift))
        {
            PRINT_ERROR("failed to get place pose via ros service");
            return false;
        }
        PRINT_INFO("successfully updated block info");
    }
    //工程に合わせてモーションを生成する
    if (processIndex == TO_INITIAL_POSE)
    {
        goal = getFirstAction();
        nextProcess = TO_PICK_APPROACH;
    }
    else if (processIndex == TO_PICK_APPROACH)
    {
        Pose pickApproachPose(getPickPose.response);
        pickApproachPose.position.z += getBlockState.response.block_size_height * 3;
        auto command = getManipulatorCommand(pickApproachPose, blockIndex, processIndex, true, true, true, true);
        goal = getCommandsGoal(command);
        nextProcess = PICK_ADJUST_GRIPPER;
    }
    else if (processIndex == PICK_ADJUST_GRIPPER)
    {
        auto command = getGripperCommand(blockSizeMin, true, blockIndex, processIndex);
        goal = getCommandsGoal(command);
        nextProcess = TO_PICK;
    }
    else if (processIndex == TO_PICK)
    {
        Pose pickPose(getPickPose.response);
        auto command = getManipulatorCommand(pickPose, blockIndex, processIndex, false, true, true, false);
        goal = getCommandsGoal(command);
        nextProcess = PICK_MOVE_GRIPPER;
    }
    else if (processIndex == PICK_MOVE_GRIPPER)
    {
        auto command = getGripperCommand(blockSizeMin, false, blockIndex, processIndex);
        goal = getCommandsGoal(command);
        nextProcess = TO_PLACE_APPROACH;
    }
    else if (processIndex == TO_PLACE_APPROACH)
    {
        Pose placeApproachPose(getPlacePoseWithShift.response);
        placeApproachPose.position.z += getBlockState.response.block_size_height * 3;
        auto command = getManipulatorCommand(placeApproachPose, blockIndex, processIndex, true, true, true, true);
        goal = getCommandsGoal(command);
        nextProcess = getComponentBlock.response.need_force_limit ? PLACE_START_FORCE_LIMIT : TO_PLACE;
    }
    else if (processIndex == PLACE_START_FORCE_LIMIT)
    {
        Pose pose(getPlacePoseWithoutShift.response);
        pose.position.z += getBlockState.response.block_size_height / 2.0;
        auto command = getManipulatorCommand(pose, blockIndex, processIndex, false, true, true, false);
        goal = getCommandsGoal(command);
        nextProcess = TO_PLACE;
    }
    else if (processIndex == TO_PLACE)
    {
        Pose pose(getPlacePoseWithoutShift.response);
        auto command = getManipulatorCommand(pose, blockIndex, processIndex, false, true, !getComponentBlock.response.need_force_limit, false);
        goal = getCommandsGoal(command);
        nextProcess = PLACE_MOVE_GRIPPER;
    }
    else if (processIndex == PLACE_MOVE_GRIPPER)
    {
        auto command = getGripperCommand(blockSizeMin, true, blockIndex, processIndex);
        goal = getCommandsGoal(command);
        nextProcess = TO_PICK_APPROACH;
    }
    else if (processIndex == PICK_RECOVER_GRIPPER)
    {
        auto command = getGripperCommand(blockSizeMin, true, blockIndex, processIndex);
        goal = getCommandsGoal(command);
        nextProcess = PICK_RECOVER_DISAPPROACH;
    }
    else if (processIndex == PICK_RECOVER_DISAPPROACH)
    {
        Pose pickApproachPose(getPickPose.response);
        pickApproachPose.position.z += getBlockState.response.block_size_height * 3;
        auto command = getManipulatorCommand(pickApproachPose, blockIndex, processIndex, true, true, true, false);
        goal = getCommandsGoal(command);
        nextProcess = TO_PICK;
    }
    else
    {
        PRINT_ERROR("unexpected process index: " << processIndex);
        return false;
    }
    //次の操作対象となるブロック番号を決定
    if (processIndex != TO_INITIAL_POSE && nextProcess == TO_PICK_APPROACH)
    {
        nextBlockIndex = blockIndex + 1;
    }
    else
    {
        nextBlockIndex = blockIndex;
    }
    //
    return true;
}

void pauseAndChooseNextMotion(int currentBlockIndex,
                              const ProcessIndex &currentProcessIndex,
                              int &nextBlockIndex,
                              ProcessIndex &nextProcessIndex,
                              ros::Duration &recoveryDuration)
{
    static const std::vector<ProcessIndex> processIndexes = {
        TO_INITIAL_POSE,
        TO_PICK_APPROACH,
        PICK_ADJUST_GRIPPER,
        TO_PICK,
        PICK_MOVE_GRIPPER,
        TO_PLACE_APPROACH,
        PLACE_START_FORCE_LIMIT,
        TO_PLACE,
        PLACE_MOVE_GRIPPER,
        PICK_RECOVER_GRIPPER,
        PICK_RECOVER_DISAPPROACH,
        TO_FINISH_POSE};
    auto pauseStart = ros::Time::now();
    PRINT_WARN("block printing is stopping");
    PRINT_WARN("current block index: " << currentBlockIndex);
    PRINT_WARN("current process: " << currentProcessIndex);
    std::string input;
    while (true)
    {
        while (true)
        {
            PRINT_INFO("choose block index which you want to reassemble:");
            std::cin >> input;
            try
            {
                nextBlockIndex = std::stoi(input);
                break;
            }
            catch (...)
            {
                PRINT_ERROR("write an integer as block index");
                continue;
            }
        }
        while (true)
        {
            std::string processes = "[";
            for (auto process : processIndexes)
            {
                processes += process + ", ";
            }
            processes += "]";
            PRINT_INFO(processes);
            PRINT_INFO("choose process index from above which you want to restart:");
            std::cin >> input;
            if (contains(processIndexes, input))
            {
                nextProcessIndex = input;
                break;
            }
            else
            {
                PRINT_ERROR("write correct process index");
                continue;
            }
        }
        PRINT_WARN("reassemble from block" << nextBlockIndex << "." << nextProcessIndex << ". [y/n]");
        std::cin >> input;
        if (input == "y")
        {
            break;
        }
        else
        {
            PRINT_INFO("reselect");
        }
    }
    ros::Duration(5.0).sleep();
    auto pauseEnd = ros::Time::now();
    recoveryDuration = pauseEnd - pauseStart;
}

static void showAndSaveDuration(const ros::Duration &whole, const std::vector<ros::Duration> &pauses)
{
    ros::Duration pauseSum = ros::Duration(0);
    for (auto pause : pauses)
    {
        pauseSum += pause;
    }
    auto taskDuration = whole - pauseSum;
    PRINT_INFO("all task finished after " << whole.toSec() << " seconds(with pauses), " << taskDuration.toSec() << " seconds(without pauses)");
    PRINT_INFO("pause count: " << pauses.size());
    PRINT_INFO("total pause duration: " << pauseSum.toSec() << " seconds");
    auto filename = "/home/kohama/Desktop/block_printing_commander_" + std::to_string(ros::Time::now().toNSec()) + ".csv";
    std::ofstream stream(filename);
    if (stream.fail())
    {
        PRINT_ERROR("failed to save time");
        return;
    }
    stream << "whole time," << whole.toSec() << std::endl;
    stream << "whole time without pause," << taskDuration.toSec() << std::endl;
    stream << "pause count," << pauses.size() << std::endl;
    stream << "total pause duration," << pauseSum.toSec() << std::endl;
    stream.close();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "block_printer_motion_interface");
    ros::NodeHandle node;
    //
    SimpleActionClient<CommandsAction> commandsAction(node, MOTION_COMMAND_ACTION);
    auto blockIndex = 0;
    auto processIndex = TO_INITIAL_POSE;
    std::vector<ros::Duration> recoveryDurations;
    if (!commandsAction.waitForServer(ros::Duration(120)))
    {
        PRINT_ERROR("no action server exists");
        return -1;
    }
    PRINT_INFO("found a action client. now block assembling starts.");
    auto startTime = ros::Time::now();
    while (true)
    {
        CommandsGoal goal;
        int nextBlockIndex;
        ProcessIndex nextProcessIndex;
        PRINT_INFO("create process:" << processIndex);
        if (!getActionGoal(blockIndex, processIndex, goal, nextBlockIndex, nextProcessIndex))
        {
            PRINT_ERROR("failed to get action. block index:" << blockIndex << " process index:" << processIndex);
            break;
        }
        commandsAction.sendGoal(goal);
        PRINT_INFO("sent process:" << processIndex);
        if (!commandsAction.waitForResult(ros::Duration(60)))
        {
            PRINT_ERROR("commands action timeout");
            ros::Duration recoveryDuration;
            pauseAndChooseNextMotion(blockIndex, processIndex, nextBlockIndex, nextProcessIndex, recoveryDuration);
            recoveryDurations.push_back(recoveryDuration);
        }
        PRINT_INFO("finish process:" << processIndex);
        if (commandsAction.getState().state_ == actionlib::SimpleClientGoalState::ABORTED)
        {
            PRINT_ERROR("failed to execute commandsAction");
            ros::Duration recoveryDuration;
            pauseAndChooseNextMotion(blockIndex, processIndex, nextBlockIndex, nextProcessIndex, recoveryDuration);
            recoveryDurations.push_back(recoveryDuration);
        }
        if (contains<std::string>(goal.commands.commands.front().additional_parameters, TO_FINISH_POSE))
        {
            break;
        }
        blockIndex = nextBlockIndex;
        processIndex = nextProcessIndex;
    }
    //かかった時間を表示
    auto end = ros::Time::now();
    auto duration = end - startTime;
    showAndSaveDuration(duration, recoveryDurations);
    //
    PRINT_INFO("waiting for shutdown...");
    ros::waitForShutdown();
    return 0;
}
