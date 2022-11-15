#include <fstream>
#include <regex>
#include <block_printer_model_reader/macro.h>
#include <block_printer_model_reader/BlockModel.h>

using namespace block_printer_model_reader;
/*
static bool readBlockFromLine_kohama2019(const std::string &line, ComponentBlock &component)
{
    //pad,x,y,z,sizeX,sizeY,sizeZ,color,press,shiftX,shiftY
    std::regex regex("(\\d+),(\\d+),(\\d+),(\\d+),(\\d+),(\\d+),(\\d+),(\\d+),(\\d+),([-]?\\d+),([-]?\\d+),");
    std::smatch results;
    if (!std::regex_match(line, results, regex))
    {
        PRINT_WARN("does not match block format: " << line);
        return false;
    }
    auto padIndex = atoi(results[1].str().c_str());
    auto x = atoi(results[2].str().c_str());
    auto y = atoi(results[3].str().c_str());
    auto z = atoi(results[4].str().c_str());
    auto sizeX = atoi(results[5].str().c_str());
    auto sizeY = atoi(results[6].str().c_str());
    auto sizeZ = atoi(results[7].str().c_str());
    auto color = BlockColor(atoi(results[8].str().c_str()));
    bool needForceLimit = std::stoi(results[9].str()) == 0;
    auto shiftX = atoi(results[10].str().c_str());
    auto shiftY = atoi(results[11].str().c_str());
    Point3 position(x, y, z);
    Point2 size(sizeX, sizeY);
    Point2 shift(shiftX, shiftY);
    component = ComponentBlock(padIndex, size, color, position, needForceLimit, shift);
    return true;
}*/

static bool readBlockFromLine_kohama2019(const std::string &line, ComponentBlock &component)
{
    //pad,x,y,z,sizeX,sizeY,sizeZ,color,isSupport,press,shiftX,shiftY
    std::regex regex("(\\d+),(\\d+),(\\d+),(\\d+),(\\d+),(\\d+),(\\d+),(\\d+),(\\d+),(\\d+),([-]?\\d+),([-]?\\d+),");
    std::smatch results;
    if (!std::regex_match(line, results, regex))
    {
        PRINT_WARN("does not match block format: \'" << line<<"\'");
        return false;
    }
    auto padIndex = atoi(results[1].str().c_str());
    auto x = atoi(results[2].str().c_str());
    auto y = atoi(results[3].str().c_str());
    auto z = atoi(results[4].str().c_str());
    auto sizeX = atoi(results[5].str().c_str());
    auto sizeY = atoi(results[6].str().c_str());
    auto sizeZ = atoi(results[7].str().c_str());
    auto color = BlockColor(atoi(results[8].str().c_str()));
    auto isSupport = std::stoi(results[9].str()) == 1;
    if (isSupport)
    {
        color = 7;
    }
    bool needForceLimit = std::stoi(results[10].str()) == 0;
    auto shiftX = atoi(results[11].str().c_str());
    auto shiftY = atoi(results[12].str().c_str());
    Point3 position(x, y, z);
    Point2 size(sizeX, sizeY);
    Point2 shift(shiftX, shiftY);
    component = ComponentBlock(padIndex, size, color, position, needForceLimit, shift);
    return true;
}

static bool readBlockFromLine(const std::string &line, const std::string &formatType, ComponentBlock &component)
{
    if (formatType == "kohama2019")
    {
        return readBlockFromLine_kohama2019(line, component);
    }
    else
    {
        PRINT_ERROR("invalid format type: " << formatType);
        return false;
    }
}

BlockModel::BlockModel() {}

BlockModel::BlockModel(const BlockModel &other) : componentBlocks(other.componentBlocks) {}

bool BlockModel::getComponent(int index, ComponentBlock &component) const
{
    if (index < 0 || index >= (int)this->componentBlocks.size())
    {
        return false;
    }
    component = this->componentBlocks.at(index);
    return true;
}

bool BlockModel::read(const std::string &filename, const std::string &type, BlockModel &blockModel)
{
    PRINT_INFO("format type:"<<type);
    blockModel = BlockModel();
    std::ifstream stream(filename);
    if (stream.fail())
    {
        PRINT_ERROR("failed to open a block model file: " << filename);
        return false;
    }
    std::string line;
    while (getline(stream, line))
    {
        std::string s;
        for(auto c: line)
        {
            if(c=='\n'||c=='\r') continue;
            s.push_back(c);
        }
        ComponentBlock component(0, Point2(0, 0), BlockColor(""), Point3(0, 0, 0), false, Point2(0, 0));
        if (!readBlockFromLine(s, type, component))
        {
            continue;
        }
        blockModel.componentBlocks.push_back(component);
    }
    stream.close();
    PRINT_INFO("succeeded to load a file. block num: " << blockModel.componentBlocks.size());
    return true;
}
