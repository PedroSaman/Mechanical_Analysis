#pragma once
#include <vector>
#include <denso_bcap_controller/Variable.h>

namespace denso_bcap_controller
{

class RadianJoint : public Variable<std::vector<double>>
{
public:
  RadianJoint(RcController &controller, int index);
  virtual ~RadianJoint();
  std::pair<std::vector<double>, BCAP_HRESULT> get() override;
  BCAP_HRESULT set(const std::vector<double> &value) override;
};
} // namespace denso_bcap_controller