#include <denso_bcap_controller/Int32.h>

using namespace denso_bcap_controller;

Int32::Int32(RcController &controller, int index)
    : Variable<int32_t>(controller, 'I', index)
{
}

Int32::~Int32()
{
}

std::pair<int32_t, BCAP_HRESULT> Int32::get()
{
    int32_t value;
    auto result = this->controller.VariableGetValue(this->variableHandle, &value);
    return std::make_pair(value, result);
}

BCAP_HRESULT Int32::set(const int32_t &value)
{
    auto temp = value;
    return this->controller.VariablePutValue(this->variableHandle, VT_I4, 1, &temp);
}