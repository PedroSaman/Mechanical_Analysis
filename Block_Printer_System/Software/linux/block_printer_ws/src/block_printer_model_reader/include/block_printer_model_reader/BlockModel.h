#pragma once
#include <string>
#include <vector>
#include <block_printer_model_reader/ComponentBlock.h>

namespace block_printer_model_reader
{
class BlockModel
{
public:
  BlockModel();
  BlockModel(const BlockModel &other);
  bool getComponent(int index, ComponentBlock &component) const;
  static bool read(const std::string &filename, const std::string &type, BlockModel &blockModel);

private:
  std::vector<ComponentBlock> componentBlocks;
};
} // namespace block_printer_model_reader