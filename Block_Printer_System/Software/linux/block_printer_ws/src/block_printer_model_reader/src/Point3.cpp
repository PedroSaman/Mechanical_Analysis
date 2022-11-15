#include <block_printer_model_reader/Point3.h>

using namespace block_printer_model_reader;

Point3::Point3() : x(0), y(0), z(0) {}
Point3::Point3(const Point3 &other) : x(other.x), y(other.y), z(other.z) {}
Point3::Point3(int x, int y, int z) : x(x), y(y), z(z) {}
Point3::~Point3() {}
Point3 Point3::operator+(const Point3 &right) const
{
    return Point3(this->x + right.x, this->y + right.y, this->z + right.z);
}
Point3 Point3::operator-(const Point3 &right) const
{
    return Point3(this->x - right.x, this->y - right.y, this->z - right.z);
}
Point3 &Point3::operator=(const Point3 &right)
{
    this->x = right.x;
    this->y = right.y;
    this->z = right.z;
    return *this;
}
bool Point3::operator==(const Point3 &right) const
{
    return this->x == right.x && this->y == right.y && this->z == right.z;
}
bool Point3::operator!=(const Point3 &right) const
{
    return this->x != right.x || this->y != right.y || this->z != right.z;
}