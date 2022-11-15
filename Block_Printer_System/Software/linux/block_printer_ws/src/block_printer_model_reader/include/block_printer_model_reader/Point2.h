#pragma once

namespace block_printer_model_reader
{
class Point2
{
public:
  int x;
  int y;
  Point2();
  Point2(const Point2 &other);
  Point2(int x, int y);
  virtual ~Point2();
  Point2 operator+(const Point2 &right) const;
  Point2 operator-(const Point2 &right) const;
  Point2 &operator=(const Point2 &right);
  bool operator==(const Point2 &right) const;
  bool operator!=(const Point2 &right) const;
};
} // namespace block_printer_model_reader