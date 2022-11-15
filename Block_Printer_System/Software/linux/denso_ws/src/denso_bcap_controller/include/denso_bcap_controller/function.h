#pragma once
#include <string>
#include <sstream>
#include <denso_bcap/bcap_base.h>

namespace denso_bcap_controller
{
double toDegree(double radian);
double toRadian(double degree);

bool isBcapSuccess(BCAP_HRESULT result);
std::string getBcapMessage(BCAP_HRESULT result);
}