#pragma once
#include <vector>
#include <initializer_list>
#include <block_printer_place/ColumnVector.h>

namespace block_printer_place
{

class Matrix
{
public:
  Matrix(int line, int column);
  Matrix(const Matrix &matrix);
  Matrix(const std::initializer_list<ColumnVector> &columnVectors);
  int getLine() const;
  int getColumn() const;
  double get(int line, int column) const;
  void set(int line, int column, double value);
  ColumnVector getColumnVector(int column) const;
  Matrix operator+(const Matrix &right) const;
  Matrix operator-(const Matrix &right) const;
  Matrix operator*(const Matrix &right) const;
  ColumnVector operator *(const ColumnVector &right) const;
  Matrix &operator=(const Matrix &right);

private:
  std::vector<std::vector<double>> elements;
  int line;
  int column;
};
}