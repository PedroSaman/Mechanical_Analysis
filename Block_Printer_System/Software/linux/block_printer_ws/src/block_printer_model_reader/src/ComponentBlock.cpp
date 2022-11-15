#include <block_printer_model_reader/ComponentBlock.h>

using namespace block_printer_model_reader;

ComponentBlock::ComponentBlock(int assemblyPadIndex, Point2 size, BlockColor color, Point3 position, bool needForceLimit,Point2 shift)
    : Block(size, color),
      assemblyPadIndex(assemblyPadIndex),
      position(position),
      needForceLimit(needForceLimit),
      shift(shift) {}

ComponentBlock::ComponentBlock(const ComponentBlock &other)
    : Block(other),
      assemblyPadIndex(other.assemblyPadIndex),
      position(other.position),
      needForceLimit(other.needForceLimit),
      shift(other.shift) {}

ComponentBlock::~ComponentBlock() {}

ComponentBlock &ComponentBlock::operator=(const ComponentBlock &right)
{
    this->assemblyPadIndex = right.assemblyPadIndex;
    this->size = right.size;
    this->color = right.color;
    this->position = right.position;
    this->needForceLimit = right.needForceLimit;
    this->shift=right.shift;
    return *this;
}

bool ComponentBlock::operator==(const ComponentBlock &right) const
{
    return this->assemblyPadIndex == right.assemblyPadIndex &&
           this->size == right.size &&
           this->color == right.color &&
           this->position == right.position &&
           this->needForceLimit == right.needForceLimit&&
           this->shift==right.shift;
}
