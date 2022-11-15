#include <ros/param.h>
#include <block_printer_model_reader/BlockColor.h>

using namespace block_printer_model_reader;

static std::vector<std::string> colorStrings;

BlockColor::BlockColor(int number)
{
    if (colorStrings.empty())
    {
        colorStrings = ros::param::param<std::vector<std::string>>("/block_printer/block/colors", {});
    }
    if (number < 0 || number >= colorStrings.size())
    {
        ROS_ERROR_STREAM("block color[" << number << "] does not exist");
    }
    else
    {
        this->name = colorStrings[number];
    }
}

BlockColor::BlockColor(const std::string &name) : name(name) {}

BlockColor::BlockColor(const BlockColor &blockColor) : name(blockColor.name) {}

std::string BlockColor::toString() const
{
    return this->name;
}

BlockColor &BlockColor::operator=(const BlockColor &right)
{
    this->name = right.name;
    return *this;
}

bool BlockColor::operator==(const BlockColor &right) const
{
    return this->name == right.name;
}

bool BlockColor::operator!=(const BlockColor &right) const
{
    return this->name != right.name;
}
