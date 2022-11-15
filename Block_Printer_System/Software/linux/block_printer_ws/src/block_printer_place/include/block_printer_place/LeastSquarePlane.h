#pragma once
#include <vector>
#include <block_printer_place/Point2.h>

namespace block_printer_place
{
class LeastSquarePlane
{
public:
  LeastSquarePlane();
  LeastSquarePlane(const LeastSquarePlane &leastSquarePlane);
  LeastSquarePlane(const std::vector<double> &xPositions, const std::vector<double> &yPositions, const std::vector<double> &values);
  double valueAt(double x, double y) const;
  double valueAt(const Point2<double> &position) const;
  double getRsme() const;

private:
  double xCoefficient;
  double yCoefficient;
  double constant;
  double rsme;
};
}