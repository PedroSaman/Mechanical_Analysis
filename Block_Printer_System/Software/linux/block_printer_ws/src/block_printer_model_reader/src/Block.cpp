#include <block_printer_model_reader/Block.h>

using namespace block_printer_model_reader;

Block::Block(Point2 size, BlockColor color) : size(size), color(color) {}

Block::Block(const Block &block) : size(block.size), color(block.color) {}

Block::~Block(){}

Block &Block::operator=(const Block &right)
{
    this->size = right.size;
    this->color = right.color;
    return *this;
}

bool Block::operator==(const Block &right) const
{
    return this->size == right.size && this->color == right.color;
}

bool Block::operator!=(const Block &right) const
{
    return this->size != right.size || this->color != right.color;
}