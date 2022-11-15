#pragma once
#include <string>
#include <vector>

namespace block_printer_pick
{
static const int PICK_LANE_CAPACITY_INFINITY = -1;

class Lane
{
  public:
    Lane();
    Lane(int sizeX,
         int sizeY,
         const std::string &color,
         int capacity,
         const std::vector<double> &pickPosition,
         const std::vector<double> &pickOrientation);
    Lane(const Lane &other);
    bool checkSameBlock(int sizeX, int sizeY, const std::string &color) const;
    bool canPick() const;
    bool pick(std::vector<double> &pickPosition, std::vector<double> &pickOrientation);
    Lane &operator=(const Lane &other);

  private:
    int sizeX;
    int sizeY;
    std::string color;
    int capacity;
    std::vector<double> pickPosition;
    std::vector<double> pickOrientation;
};
} // namespace block_printer_pick