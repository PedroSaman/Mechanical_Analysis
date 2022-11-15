#include <block_printer_pick/Lane.h>

using namespace block_printer_pick;

Lane::Lane() {}

Lane::Lane(int sizeX,
           int sizeY,
           const std::string &color,
           int capacity,
           const std::vector<double> &pickPosition,
           const std::vector<double> &pickOrientation) : sizeX(sizeX),
                                                         sizeY(sizeY),
                                                         color(color),
                                                         capacity(capacity),
                                                         pickPosition(pickPosition),
                                                         pickOrientation(pickOrientation) {}

Lane::Lane(const Lane &other) : sizeX(other.sizeX),
                                sizeY(other.sizeY),
                                color(other.color),
                                capacity(other.capacity),
                                pickPosition(other.pickPosition),
                                pickOrientation(other.pickOrientation) {}

bool Lane::checkSameBlock(int sizeX, int sizeY, const std::string &color) const
{
    auto max = std::max(sizeX, sizeY);
    auto min = std::min(sizeX, sizeY);
    return max == this->sizeX && min == this->sizeY && color == this->color;
}

bool Lane::canPick() const
{
    return this->capacity > 0 || this->capacity == PICK_LANE_CAPACITY_INFINITY;
}

bool Lane::pick(std::vector<double> &pickPosition, std::vector<double> &pickOrientation)
{
    if (!this->canPick())
    {
        return false;
    }
    if (this->capacity != PICK_LANE_CAPACITY_INFINITY)
    {
        this->capacity--;
    }
    pickPosition = this->pickPosition;
    pickOrientation = this->pickOrientation;
    return true;
}

Lane &Lane::operator=(const Lane &other)
{
    this->sizeX = other.sizeX;
    this->sizeY = other.sizeY;
    this->color = other.color;
    this->capacity = other.capacity;
    this->pickPosition = other.pickPosition;
    this->pickOrientation = other.pickOrientation;
}
