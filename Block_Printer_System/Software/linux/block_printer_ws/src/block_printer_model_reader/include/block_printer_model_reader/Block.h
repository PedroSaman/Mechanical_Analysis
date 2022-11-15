#pragma once
#include <block_printer_model_reader/Point2.h>
#include <block_printer_model_reader/BlockColor.h>

namespace block_printer_model_reader
{
class Block
{
public:
  Point2 size;
  BlockColor color;
  Block(Point2 size, BlockColor color);
  Block(const Block &block);
  virtual ~Block();
  Block &operator=(const Block &right);
  bool operator==(const Block &right) const;
  bool operator!=(const Block &right) const;
};
} // namespace block_printer_model_reader