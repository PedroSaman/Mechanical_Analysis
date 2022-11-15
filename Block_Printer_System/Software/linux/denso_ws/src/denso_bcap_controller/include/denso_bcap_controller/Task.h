#pragma once
#include <denso_bcap_controller/Object.h>

namespace denso_bcap_controller
{

class Task : public Object
{
public:
  Task(RcController &controller, const std::string &taskName);
  virtual ~Task();
  std::pair<bool, BCAP_HRESULT> isRunning();
  BCAP_HRESULT run(bool stopExecutingTask);
  BCAP_HRESULT stop();

private:
  BCapHandle taskHandle;
  BCapHandle taskStateHandle;
};
} // namespace denso_bcap_controller