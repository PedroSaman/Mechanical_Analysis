#include <block_printer_model_reader/Point2.h>

using namespace block_printer_model_reader;

Point2::Point2() : x(0), y(0) {}
Point2::Point2(const Point2 &other) : x(other.x), y(other.y) {}
Point2::Point2(int x, int y) : x(x), y(y) {}
Point2::~Point2() {}
Point2 Point2::operator+(const Point2 &right) const
{
    return Point2(this->x + right.x, this->y + right.y);
}
Point2 Point2::operator-(const Point2 &right) const
{
    return Point2(this->x - right.x, this->y - right.y);
}
Point2 &Point2::operator=(const Point2 &right)
{
    this->x = right.x;
    this->y = right.y;
    return *this;
}
bool Point2::operator==(const Point2 &right) const
{
    return this->x == right.x && this->y == right.y;
}
bool Point2::operator!=(const Point2 &right) const
{
    return this->x != right.x || this->y != right.y;
}