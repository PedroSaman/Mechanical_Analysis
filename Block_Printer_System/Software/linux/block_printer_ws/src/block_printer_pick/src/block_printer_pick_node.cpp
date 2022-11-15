#include <ros/ros.h>
#include <block_printer_model_reader/CapableNames.h>
#include <block_printer_model_reader/GetComponentBlock.h>
#include <block_printer_pick/CapableNames.h>
#include <block_printer_pick/GetPickPose.h>
#include <block_printer_pick/Lane.h>

using namespace block_printer_model_reader;
using namespace block_printer_pick;

static const std::string logHeader = "pick";

#define PRINT_INFO(args) ROS_INFO_STREAM(logHeader << ": " << args)
#define PRINT_WARN(args) ROS_WARN_STREAM(logHeader << ": " << args)
#define PRINT_ERROR(args) ROS_ERROR_STREAM(logHeader << ": " << args)

void createLanesFromRosParam(std::vector<Lane> &lanes)
{
    lanes.clear();
    for (int i = 0;; i++)
    {
        std::string paramPath = "/block_printer/pick" + std::to_string(i);
        if (!ros::param::has(paramPath))
        {
            break;
        }
        auto sizeX = ros::param::param<int>(paramPath + "/block/size_x", 0);
        auto sizeY = ros::param::param<int>(paramPath + "/block/size_y", 0);
        auto color = ros::param::param<std::string>(paramPath + "/block/color", "");
        auto capacity = ros::param::param<int>(paramPath + "/capacity", block_printer_pick::PICK_LANE_CAPACITY_INFINITY);
        auto position = ros::param::param<std::vector<double>>(paramPath + "/position", {});
        auto orientation = ros::param::param<std::vector<double>>(paramPath + "/orientation", {});
        Lane lane(sizeX, sizeY, color, capacity, position, orientation);
        PRINT_INFO("loaded a picking lane for " << sizeX << "x" << sizeY << "(" << color << "). capacity:" << capacity);
        lanes.push_back(lane);
    }
    PRINT_INFO("loaded " << lanes.size() << " lanes for picking blocks");
}

bool getPickPose(GetPickPoseRequest &request, GetPickPoseResponse &response, std::vector<Lane> &lanes)
{
    GetComponentBlock getComponentBlock;
    getComponentBlock.request.block_index = request.block_index;
    if (!ros::service::call<GetComponentBlock>(block_printer_model_reader::COMPONENT_BLOCK_SERVICE, getComponentBlock))
    {
        PRINT_ERROR("failed to call service \'/block_printer/get_component_block\'");
        return false;
    }
    auto blockSizeX = getComponentBlock.response.size_x;
    auto blockSizeY = getComponentBlock.response.size_y;
    auto blockColor = getComponentBlock.response.color;
    for (int i = 0; i < lanes.size(); i++)
    {
        if (!lanes[i].checkSameBlock(blockSizeX, blockSizeY, blockColor) || !lanes[i].canPick())
        {
            continue;
        }
        std::vector<double> position;
        std::vector<double> orientation;
        if (!lanes[i].pick(position, orientation))
        {
            PRINT_ERROR("an error occurred while searching a pick pose");
            continue;
        }
        response.position.x = position.at(0);
        response.position.y = position.at(1);
        response.position.z = position.at(2);
        response.orientation = orientation;
        return true;
    }
    PRINT_ERROR("no lane has requested block:\n"
                << getComponentBlock.response);
    return false;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "block_printer_pick");
    ros::NodeHandle node;
    std::vector<Lane> pickLanes;
    createLanesFromRosParam(pickLanes);
    auto getPickPoseService = node.advertiseService<GetPickPoseRequest, GetPickPoseResponse>(PICK_POSE_SERVICE, boost::bind(&getPickPose, _1, _2, boost::ref(pickLanes)));
    ros::spin();
    return 0;
}
