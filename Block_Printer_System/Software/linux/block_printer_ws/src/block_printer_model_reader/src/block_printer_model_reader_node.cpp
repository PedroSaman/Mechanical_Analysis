#include <ros/ros.h>
#include <block_printer_model_reader/macro.h>
#include <block_printer_model_reader/CapableNames.h>
#include <block_printer_model_reader/GetComponentBlock.h>
#include <block_printer_model_reader/GetBlockState.h>
#include <block_printer_model_reader/BlockModel.h>

using namespace block_printer_model_reader;

static BlockModel blockModel;

bool getComponentBlock(GetComponentBlockRequest &request, GetComponentBlockResponse &response)
{
    ComponentBlock componentBlock(0, Point2(0, 0), BlockColor(""), Point3(0, 0, 0), false, Point2(0, 0));
    if (!blockModel.getComponent(request.block_index, componentBlock))
    {
        PRINT_ERROR("could not get component block. request: " << request);
        return false;
    }
    response.assembly_pad_index = componentBlock.assemblyPadIndex;
    response.size_x = componentBlock.size.x;
    response.size_y = componentBlock.size.y;
    response.color = componentBlock.color.toString();
    response.position_x = componentBlock.position.x;
    response.position_y = componentBlock.position.y;
    response.position_z = componentBlock.position.z;
    response.need_force_limit = componentBlock.needForceLimit;
    response.shift_x = componentBlock.shift.x;
    response.shift_y = componentBlock.shift.y;
    return true;
}

bool getBlockState(GetBlockStateRequest &request, GetBlockStateResponse &response)
{
    if (!ros::param::has("/block_printer/block/size/side") ||
        !ros::param::has("/block_printer/block/size/height") ||
        !ros::param::has("/block_printer/block/size/convex/radius") ||
        !ros::param::has("/block_printer/block/size/convex/height"))
    {
        PRINT_ERROR("block size parameter does not exist.");
        return false;
    }
    response.block_size_side = ros::param::param<double>("/block_printer/block/size/side", 0);
    response.block_size_height = ros::param::param<double>("/block_printer/block/size/height", 0);
    response.block_convex_radius = ros::param::param<double>("/block_printer/block/size/convex/radius", 0);
    response.block_convex_height = ros::param::param<double>("/block_printer/block/size/convex/height", 0);
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "block_printer_model_reader");
    ros::NodeHandle node;
    ros::NodeHandle paramNode("~");
    //
    auto filename = paramNode.param<std::string>("model_filename", "");
    auto filetype = paramNode.param<std::string>("model_type", "");
    if (!BlockModel::read(filename, filetype, blockModel))
    {
        PRINT_ERROR("failed to load block-model file.\nfilename:" << filename << "\nfiletype:" << filetype);
        return -1;
    }
    //
    auto getBlockStateService = node.advertiseService(BLOCK_STATE_SERVICE, getBlockState);
    auto getComponentBlockService = node.advertiseService(COMPONENT_BLOCK_SERVICE, getComponentBlock);
    ros::spin();
    return 0;
}
