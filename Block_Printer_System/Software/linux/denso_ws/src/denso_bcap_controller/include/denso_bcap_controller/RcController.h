#pragma once
#include <string>
#include <denso_bcap/bcap_net.h>
#include <denso_bcap_controller/type.h>

namespace denso_bcap_controller
{
class RcController : public BCapNet
{
public:
  RcController(const std::string &ip, const std::string &port, BCapNet::ConnectingMode mode, bool readonly);
  virtual ~RcController();
  bool hasError();
  std::string getErrorMessage();
  BCapHandle getHandle() const;

private:
  bool readonly;
  BCapHandle controllerHandle;
  BCapHandle errorVariableHandle;
  int32_t getErrorCode();
};
} // namespace denso_bcap_controller