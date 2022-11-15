#pragma once

namespace block_printer_place
{
template <class T>
class Point2
{
  public:
    T x;
    T y;
    Point2() : x(), y() {}
    Point2(const Point2<T> &point) : x(point.x), y(point.y) {}
    Point2(T x, T y) : x(x), y(y) {}
    template <class U>
    Point2<U> cast() const
    {
        return Point2<U>((U)this->x, (U)this->y);
    }
    Point2<T> operator+(const Point2<T> &right) const
    {
        return Point2<T>(this->x + right.x, this->y + right.y);
    }
    Point2<T> operator-(const Point2<T> &right) const
    {
        return Point2<T>(this->x - right.x, this->y - right.y);
    }
    Point2<T> operator*(double right) const
    {
        return Point2<T>(this->x * right, this->y * right);
    }

    Point2<T> &operator=(const Point2<T> &right)
    {
        this->x = right.x;
        this->y = right.y;
        return *this;
    }
    Point2<T> &operator+=(const Point2<T> &right)
    {
        this->x += right.x;
        this->y += right.y;
        return *this;
    }
    bool operator==(const Point2<T> &right) const
    {
        return this->x == right.x && this->y == right.y;
    }
    bool operator!=(const Point2<T> &right) const
    {
        return this->x != right.x || this->y != right.y;
    }
};
} // namespace block_printer_place
