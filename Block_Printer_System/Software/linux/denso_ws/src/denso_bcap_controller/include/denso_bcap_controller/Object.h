#pragma once
#include <memory>
#include <denso_bcap_controller/type.h>
#include <denso_bcap_controller/RcController.h>

namespace denso_bcap_controller
{
class Object
{
public:
  Object(RcController &controller);
  virtual ~Object();

protected:
  RcController &controller;
};
} // namespace denso_bcap_controller