#include <denso_bcap_controller/macro.h>
#include <denso_bcap_controller/function.h>
#include <denso_bcap_controller/Hand.h>

static const int TARGET_POSITION_VARIABLE_INDEX = 10;

static const std::string INIT_TASK_PATH = "xdntaiyoesg.BlockPrinterInit";
static const std::string POSITION_SET_TASK_PATH = "xdntaiyoesg.BlockPrinterHandSet";

using namespace control_msgs;
using namespace actionlib;
using namespace denso_bcap_controller;

Hand::Hand(RcController &controller, bool useHand)
    : Object(controller),
      targetPositionVariable(controller, TARGET_POSITION_VARIABLE_INDEX),
      initTask(controller, INIT_TASK_PATH),
      moveTask(controller, POSITION_SET_TASK_PATH)
{
    if (!useHand)
    {
        return;
    }
    //ハンドのモーターオンと原点復帰
    this->initTask.run(true);
    //原点復帰が完了するまで待機
    while (true)
    {
        auto running = this->initTask.isRunning();
        if (!isBcapSuccess(running.second))
        {
            PRINT_ERROR("while initializing a hand: " << getBcapMessage(running.second));
        }
        if (!running.first)
        {
            break;
        }
        ros::Duration(0.05).sleep();
    }
}

Hand::~Hand() {}

bool Hand::executeCommand(const control_msgs::GripperCommand &command)
{
    //
    auto millimeter = command.position * 1000.0;
    auto setResult = this->targetPositionVariable.set((float)millimeter);
    if (!isBcapSuccess(setResult))
    {
        PRINT_ERROR("while setting a target position of hand: " << getBcapMessage(setResult));
        return false;
    }
    //
    auto runResult = this->moveTask.run(true);
    if (!isBcapSuccess(runResult))
    {
        PRINT_ERROR("while try to run a task to move a hand: " << getBcapMessage(runResult));
        return false;
    }
    //wait for task-finish
    while (true)
    {
        if (this->controller.hasError())
        {
            this->moveTask.stop();
            return false;
        }
        auto runningResult = this->moveTask.isRunning();
        if (!isBcapSuccess(runningResult.second))
        {
            PRINT_ERROR("while waiting for finish of hand-move task: " << getBcapMessage(runningResult.second));
            return false;
        }
        if (!runningResult.first)
        {
            break;
        }
        ros::spinOnce();
        ros::Duration(0.05).sleep();
    }
    //
    return true;
}
