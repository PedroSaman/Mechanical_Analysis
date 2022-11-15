#pragma once
#include <block_printer_model_reader/Point2.h>
#include <block_printer_model_reader/Point3.h>
#include <block_printer_model_reader/Block.h>

namespace block_printer_model_reader
{

class ComponentBlock : public Block
{
public:
  int assemblyPadIndex;
  bool needForceLimit;
  Point3 position;
  Point2 shift;
  ComponentBlock(int assemblyPadIndex, Point2 size, BlockColor color, Point3 position, bool needForceLimit,Point2 shift);
  ComponentBlock(const ComponentBlock &other);
  virtual ~ComponentBlock();
  ComponentBlock &operator=(const ComponentBlock &right);
  bool operator==(const ComponentBlock &right) const;
};
} // namespace block_printer_model_reader