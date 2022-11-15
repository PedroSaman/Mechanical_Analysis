#include <math.h>
#include <denso_bcap_controller/function.h>

double denso_bcap_controller::toDegree(double radian)
{
    return radian / M_PI * 180.0;
}

double denso_bcap_controller::toRadian(double degree)
{
    return degree / 180.0 * M_PI;
}

bool denso_bcap_controller::isBcapSuccess(BCAP_HRESULT result)
{
    return result == BCAP_HRESULT::BCAP_S_OK;
}

std::string denso_bcap_controller::getBcapMessage(BCAP_HRESULT result)
{
    if (denso_bcap_controller::isBcapSuccess(result))
    {
        return "b-Cap success";
    }
    std::stringstream ss;
    ss << "b-Cap error: " << std::hex << result;
    return ss.str();
}