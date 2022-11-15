#include <ros/ros.h>
#include <block_printer_place/CapableNames.h>
#include <block_printer_place/GetPlacePose.h>
#include <block_printer_place/WorkPad.h>

using namespace block_printer_place;

static std::vector<WorkPad> workPads;

bool getPlacePose(GetPlacePoseRequest &request, GetPlacePoseResponse &response)
{
    auto index = request.assembly_pad_index;
    if (index < 0 || index > workPads.size())
    {
        return false;
    }
    return workPads.at(index).getPoseAt(request, response);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "block_printer_place");
    ros::NodeHandle node;
    workPads = WorkPad::createWorkPads();
    auto getPlacePoseService = node.advertiseService(PLACE_POSE_SERVICE, getPlacePose);
    ros::spin();
    return 0;
}