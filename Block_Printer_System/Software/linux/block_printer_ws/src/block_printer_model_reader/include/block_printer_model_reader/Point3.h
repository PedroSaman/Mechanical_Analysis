#pragma once

namespace block_printer_model_reader
{
class Point3
{
public:
  int x;
  int y;
  int z;
  Point3();
  Point3(const Point3 &other);
  Point3(int x, int y, int z);
  virtual ~Point3();
  Point3 operator+(const Point3 &right) const;
  Point3 operator-(const Point3 &right) const;
  Point3 &operator=(const Point3 &right);
  bool operator==(const Point3 &right) const;
  bool operator!=(const Point3 &right) const;
};
} // namespace block_printer_model_reader