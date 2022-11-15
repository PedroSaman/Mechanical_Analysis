#include <ros/ros.h>
#include <trajectory_reducer/CapableNames.h>
#include <trajectory_reducer/ReduceTrajectory.h>

using namespace trajectory_reducer;

static const std::string logHeader = "trajectory_reducer";

#define PRINT_INFO(args) ROS_INFO_STREAM(logHeader << ": " << args)

bool reduceTrajectoryService(ReduceTrajectoryRequest &request, ReduceTrajectoryResponse &response)
{
    auto inputPoint = (int)request.trajectory_source.points.size();
    auto maxPoint = request.max_point;
    PRINT_INFO("requested trajectory contains " << inputPoint << " points");
    //copy header
    response.trajectory.header = request.trajectory_source.header;
    response.trajectory.joint_names = request.trajectory_source.joint_names;
    if (inputPoint > maxPoint)
    {
        response.trajectory.points.reserve(maxPoint);
        //Add first point to output trajectory
        response.trajectory.points.push_back(request.trajectory_source.points[0]);
        auto intermediate_points = maxPoint - 2; //subtract the first and last elements
        auto int_point_increment = double(inputPoint) / double(intermediate_points + 1.0);
        // The intermediate point index is determined by the following equation:
        //     int_point_index = i * int_point_increment
        for (auto i = 1; i <= intermediate_points; i++)
        {
            auto int_point_index = int(double(i) * int_point_increment);
            response.trajectory.points.push_back(request.trajectory_source.points[int_point_index]);
        }
        //Add last point to output trajectory
        response.trajectory.points.push_back(request.trajectory_source.points.back());
        //
        response.reduced = true;
    }
    else
    {
        response.trajectory = request.trajectory_source;
        response.reduced = false;
    }
    response.point = (int)response.trajectory.points.size();
    PRINT_INFO("response trajectory contains " << response.point << " points");
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_reducer");
    ros::NodeHandle node;
    auto reduceTrajectoryServer = node.advertiseService(REDUCE_TRAJECTORY_SERVICE, reduceTrajectoryService);
    PRINT_INFO("ready to serve trajectories");
    ros::spin();
    return 0;
}