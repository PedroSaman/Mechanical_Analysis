#pragma once
#include <denso_bcap_controller/Variable.h>

namespace denso_bcap_controller
{

class Float32 : public Variable<float>
{
public:
  Float32(RcController &controller, int index);
  virtual ~Float32();
  std::pair<float, BCAP_HRESULT> get() override;
  BCAP_HRESULT set(const float &value) override;
};
} // namespace denso_bcap_controller