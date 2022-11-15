#include <denso_bcap_controller/RadianJoint.h>
#include <denso_bcap_controller/function.h>

static const int BCAP_JOINT_VARIABLE_ARRAY_LENGTH = 6;

using namespace denso_bcap_controller;

RadianJoint::RadianJoint(RcController &controller, int index)
    : Variable<std::vector<double>>(controller, 'J', index)
{
}

RadianJoint::~RadianJoint() {}

std::pair<std::vector<double>, BCAP_HRESULT> RadianJoint::get()
{
    float degrees[BCAP_JOINT_VARIABLE_ARRAY_LENGTH];
    auto result = this->controller.VariableGetValue(this->variableHandle, degrees);
    std::vector<double> value;
    for (auto degree : degrees)
    {
        value.push_back(toRadian(degree));
    }
    return std::make_pair(value, result);
}

BCAP_HRESULT RadianJoint::set(const std::vector<double> &value)
{
    float degrees[BCAP_JOINT_VARIABLE_ARRAY_LENGTH];
    for (int i = 0; i < BCAP_JOINT_VARIABLE_ARRAY_LENGTH; i++)
    {
        if (i >= value.size())
        {
            degrees[i] = 0.0f;
            continue;
        }
        degrees[i] = (float)toDegree(value[i]);
    }
    return this->controller.VariablePutValue(this->variableHandle, VT_R4 | VT_ARRAY, BCAP_JOINT_VARIABLE_ARRAY_LENGTH, degrees);
}