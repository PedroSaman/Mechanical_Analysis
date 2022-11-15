#pragma once
#include <denso_bcap_controller/Variable.h>

namespace denso_bcap_controller
{

class Int32 : public Variable<int32_t>
{
public:
  Int32(RcController &controller, int index);
  virtual ~Int32();
  std::pair<int32_t, BCAP_HRESULT> get() override;
  BCAP_HRESULT set(const int32_t &value) override;
};
} // namespace denso_bcap_controller