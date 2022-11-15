#include <denso_bcap_controller/macro.h>
#include <denso_bcap_controller/function.h>
#include <denso_bcap_controller/Task.h>

using namespace denso_bcap_controller;

Task::Task(RcController &controller, const std::string &taskName)
    : Object(controller)
{
    this->controller.ControllerGetTask(this->controller.getHandle(), taskName, "", &this->taskHandle);
    this->controller.TaskGetVariable(this->taskHandle, "@STATUS", "", &this->taskStateHandle);
    PRINT_INFO("get a task \'" << taskName << "\'");
}

Task::~Task()
{
    if (this->isRunning().first)
    {
        this->stop();
    }
    this->controller.VariableRelease(this->taskStateHandle);
    this->controller.TaskRelease(this->taskHandle);
}

std::pair<bool, BCAP_HRESULT> Task::isRunning()
{
    int32_t value = 1;
    auto result = this->controller.VariableGetValue(this->taskStateHandle, &value);
    return std::make_pair(value != 1, result);
}

BCAP_HRESULT Task::run(bool stopExecutingTask)
{
    if (this->isRunning().first)
    {
        if (!stopExecutingTask)
        {
            return BCAP_HRESULT::BCAP_E_FAIL;
        }
        auto stopResult = this->stop();
        if (!isBcapSuccess(stopResult))
        {
            return stopResult;
        }
    }
    return this->controller.TaskStart(this->taskHandle, 2, "");
}

BCAP_HRESULT Task::stop()
{
    return this->controller.TaskStop(this->taskHandle, 4, "");
}