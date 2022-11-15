#pragma once
#include <initializer_list>
#include <block_printer_place/Matrix.h>
#include <block_printer_place/ColumnVector.h>

namespace block_printer_place
{
class SquareMatrix : public block_printer_place::Matrix
{
public:
  SquareMatrix(int size);
  SquareMatrix(const SquareMatrix &squareMatrix);
  SquareMatrix(const std::initializer_list<ColumnVector> &columnVectors);
  int size() const;
  SquareMatrix inverse() const;
  SquareMatrix &operator=(const SquareMatrix &right);
  static SquareMatrix createUnit(int size);
};
}