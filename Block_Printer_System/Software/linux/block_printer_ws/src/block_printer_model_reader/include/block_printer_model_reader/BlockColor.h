#pragma once
#include <string>

namespace block_printer_model_reader
{
class BlockColor
{
public:
  BlockColor(int number);
  BlockColor(const std::string &name);
  BlockColor(const BlockColor &blockColor);
  virtual std::string toString() const;
  BlockColor &operator=(const BlockColor &right);
  bool operator==(const BlockColor &right) const;
  bool operator!=(const BlockColor &right) const;

private:
  std::string name;
};
} // namespace block_printer_model_reader