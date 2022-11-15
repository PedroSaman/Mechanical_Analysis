#pragma once
#include <string>
#include <vector>
#include <block_printer_place/GetPlacePose.h>
#include <block_printer_place/Point2.h>
#include <block_printer_place/LeastSquarePlane.h>

namespace block_printer_place
{
class WorkPad
{
public:
  WorkPad();
  WorkPad(const WorkPad &other);
  virtual ~WorkPad();
  bool getPoseAt(GetPlacePoseRequest &request, GetPlacePoseResponse &response) const;
  std::string toString() const;
  WorkPad &operator=(const WorkPad &other);
  static WorkPad fromRosParam(const std::string &workPadParamPath);
  static std::vector<WorkPad> createWorkPads();

private:
  class BlockCalibration
  {
  public:
    Point2<int> blockSize;
    LeastSquarePlane planeX;
    LeastSquarePlane planeY;
    LeastSquarePlane planeZ;
    std::vector<double> orientation;
    BlockCalibration();
    BlockCalibration(const BlockCalibration &other);
    BlockCalibration &operator=(const BlockCalibration &other);
  };
  int padIndex;
  Point2<int> size;
  double shiftGridRatio;
  std::vector<BlockCalibration> calibrations;
};
} // namespace block_printer_place