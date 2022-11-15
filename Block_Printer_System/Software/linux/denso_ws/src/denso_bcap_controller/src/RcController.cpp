#include <sstream>
#include <denso_bcap_controller/macro.h>
#include <denso_bcap_controller/RcController.h>

using namespace denso_bcap_controller;

RcController::RcController(const std::string &ip, const std::string &port, BCapNet::ConnectingMode mode, bool readonly)
    : BCapNet(ip, port, mode), readonly(readonly)
{
    this->ServiceStart();
    this->ControllerConnect("", "", "", "", &this->controllerHandle);
    this->ControllerGetVariable(this->controllerHandle, "@ERROR_CODE", "", &this->errorVariableHandle);
    if (readonly)
    {
        return;
    }
    //外部モードに設定
    uint16_t mode16 = 2;
    long tempResult;
    this->ControllerExecute2(this->controllerHandle, "PutAutoMode", VT_I2, 1, &mode16, &tempResult);
    PRINT_INFO("connected to the controller");
}

RcController::~RcController()
{
    if (!this->readonly)
    {
        //外部モードを解除
        uint16_t mode16 = 1;
        long result;
        this->ControllerExecute2(this->controllerHandle, "PutAutoMode", VT_I2, 1, &mode16, &result);
    }
    //
    this->VariableRelease(this->errorVariableHandle);
    this->ControllerDisconnect(this->controllerHandle);
    this->ServiceStop();
    PRINT_INFO("disconnected to the controller");
}

int32_t RcController::getErrorCode()
{
    int32_t value;
    this->VariableGetValue(this->errorVariableHandle, &value);
    return value;
}

bool RcController::hasError()
{
    return this->getErrorCode() != 0;
    int32_t value;
    this->VariableGetValue(this->errorVariableHandle, &value);
    return value != 0;
}

std::string RcController::getErrorMessage()
{
    auto code = this->getErrorCode();
    if (code != 0)
    {
        std::stringstream ss;
        ss << "RC controller error: " << std::hex << code;
        return ss.str();
    }
    return "RC controller has no error";
}

BCapHandle RcController::getHandle() const
{
    return this->controllerHandle;
}