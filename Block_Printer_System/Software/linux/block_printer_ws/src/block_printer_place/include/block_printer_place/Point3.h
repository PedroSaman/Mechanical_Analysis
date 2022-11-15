#pragma once
#include <block_printer_place/Point2.h>

namespace block_printer_place
{
template <class T>
class Point3
{
  public:
    T x;
    T y;
    T z;
    Point3() : x(), y(), z() {}
    Point3(const Point3<T> &point):x(point.x),y(point.y),z(point.z){}
    Point3(T x, T y, T z) : x(x), y(y), z(z) {}
    Point2<T> toPoint2() const
    {
        return Point2<T>(this->x, this->y);
    }
    template <class U>
    Point3<U> cast() const
    {
        return Point3<U>((U)this->x, (U)this->y, (U)this->z);
    }
    Point3<T> operator+(const Point3<T> &right) const
    {
        return Point3<T>(this->x + right.x, this->y + right.y, this->z + right.z);
    }
    Point3<T> operator-(const Point3<T> &right) const
    {
        return Point3<T>(this->x - right.x, this->y - right.y, this->z - right.z);
    }
    Point3<T> &operator=(const Point3<T> &right)
    {
        this->x = right.x;
        this->y = right.y;
        this->z = right.z;
        return *this;
    }
    bool operator==(const Point3<T> &right) const
    {
        return this->x == right.x && this->y == right.y && this->z == right.z;
    }
    bool operator!=(const Point3<T> &right) const
    {
        return this->x != right.x || this->y != right.y || this->z != right.z;
    }
};
} // namespace block_printer_place