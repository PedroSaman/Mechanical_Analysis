#pragma once
#include <vector>
#include <initializer_list>

namespace block_printer_place
{
class ColumnVector
{
public:
  ColumnVector(int line);
  ColumnVector(const ColumnVector &columnVector);
  ColumnVector(const std::initializer_list<double> &elements);
  int line() const;
  double get(int line) const;
  void set(int line, double value);
  private:
  std::vector<double> elements;
};
}