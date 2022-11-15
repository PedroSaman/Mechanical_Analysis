#pragma once
#include <denso_bcap_controller/macro.h>
#include <denso_bcap_controller/Object.h>

namespace denso_bcap_controller
{
static const int DENSO_BCAP_VARIABLE_INDEX_MIN = 0;
static const int DENSO_BCAP_VARIABLE_INDEX_MAX = 99;
static const int DENSO_BCAP_VARIABLE_NUM = DENSO_BCAP_VARIABLE_INDEX_MAX - DENSO_BCAP_VARIABLE_INDEX_MIN + 1;

template <class T>
class Variable : public Object
{
  public:
    Variable(RcController &controller, char prefix, int index) : Object(controller)
    {
        if (index < DENSO_BCAP_VARIABLE_INDEX_MIN || index > DENSO_BCAP_VARIABLE_INDEX_MAX)
        {
            PRINT_ERROR("unexpected variable index:" << index << " index must be in range [" << DENSO_BCAP_VARIABLE_INDEX_MIN << "," << DENSO_BCAP_VARIABLE_INDEX_MAX << "]");
            return;
        }
        auto variableName = std::string(1, prefix) + std::to_string(index);
        controller.ControllerGetVariable(this->controller.getHandle(), variableName.c_str(), "", &this->variableHandle);
    }
    virtual ~Variable()
    {
        this->controller.VariableRelease(this->variableHandle);
    }
    virtual std::pair<T, BCAP_HRESULT> get() = 0;
    virtual BCAP_HRESULT set(const T &value) = 0;

  protected:
    BCapHandle variableHandle;
};
} // namespace denso_bcap_controller