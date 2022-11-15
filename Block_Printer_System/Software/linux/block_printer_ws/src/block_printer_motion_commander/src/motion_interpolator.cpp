#include <memory>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <robot_movement_interface/CommandsAction.h>
#include <block_printer_motion_commander/CapableNames.h>
#include <block_printer_motion_commander/Pose.h>

using namespace actionlib;
using namespace robot_movement_interface;
using namespace block_printer_motion_commander;

static void sendInterpolatedCommands(const CommandsGoalConstPtr &ptr);

static SimpleActionServer<CommandsAction> commandsActionServer(MOTION_COMMAND_ACTION, sendInterpolatedCommands, false);
static SimpleActionClient<CommandsAction> interpolatedCommandsActionClient(MOTION_COMMAND_ACTION_INTERPOLATED);

static Pose previousPose;

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

static CommandsGoal getCommandsGoal(const CommandsGoal &source, const std::vector<Pose> &poses)
{
    CommandsGoal value;
    auto front = source.commands.commands.front();
    for (auto pose : poses)
    {
        //pose以外の要素はすべて先頭コマンドと同じにする
        Command command = front;
        pose.toVector<float>(command.pose);
    }
    return value;
}

static CommandsGoal interpolateManipulatorAction(const CommandsGoal &source)
{
    auto command = source.commands.commands.front();
    //moveitによる機動生成が必要ないコマンドは何も加工せずに返す
    if (command.additional_parameters.empty() || !contains<std::string>(command.additional_parameters, MOVEIT_INTERPOLATION_COMMAND))
    {
        return source;
    }
    //moveitによる軌道生成が必要なコマンドなら，moveitに代わってここで経由点をつくる
    std::vector<Pose> poses;
    auto previous = previousPose;
    auto next = Pose(command.pose);
    {
        //pose1...前ポーズとほぼ同じポーズで，高さだけをより高い方へ合わせたもの
        auto pose1 = previous;
        pose1.position.z = std::max(previous.position.z, next.position.z);
        poses.push_back(pose1);
    }
    {
        //pose2...次ポーズと同じ姿勢で，高さだけをより高い方へ合わせたもの
        auto pose2 = next;
        pose2.position.z = std::max(previous.position.z, next.position.z);
        poses.push_back(pose2);
        //pose2がもともとの目標位置よりも高い位置にあるなら，もともとの目標位置を追加
        if (pose2.position.z > next.position.z)
        {
            poses.push_back(next);
        }
    }
    //------------------グローバル変数書き換え-------------------------
    previousPose = next;
    //-----------------------------------------------------------------
    return getCommandsGoal(source, poses);
}

void sendInterpolatedCommands(const CommandsGoalConstPtr &ptr)
{
    auto interpolated = interpolateManipulatorAction(*ptr);
    interpolatedCommandsActionClient.sendGoal(interpolated);
    interpolatedCommandsActionClient.waitForResult();
    auto state = interpolatedCommandsActionClient.getState();
    if (state == SimpleClientGoalState::SUCCEEDED)
    {
        commandsActionServer.setSucceeded();
    }
    else if (state == SimpleClientGoalState::PREEMPTED)
    {
        commandsActionServer.setPreempted();
    }
    else
    {
        commandsActionServer.setAborted();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "block_printer_motion_interface");
    ros::NodeHandle node;
    //
    commandsActionServer.start();
    ros::spin();
    return 0;
}