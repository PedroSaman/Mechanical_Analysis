#include <denso_bcap_controller/Float32.h>

using namespace denso_bcap_controller;

Float32::Float32(RcController &controller, int index)
    : Variable<float>(controller, 'F', index)
{
}

Float32::~Float32()
{
}

std::pair<float, BCAP_HRESULT> Float32::get()
{
    float value;
    auto result = this->controller.VariableGetValue(this->variableHandle, &value);
    return std::make_pair(value, result);
}

BCAP_HRESULT Float32::set(const float &value)
{
    auto temp = value;
    return this->controller.VariablePutValue(this->variableHandle, VT_R4, 1, &temp);
}