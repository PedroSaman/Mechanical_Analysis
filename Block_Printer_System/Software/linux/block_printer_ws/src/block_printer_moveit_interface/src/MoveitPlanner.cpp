#include <ros/console.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/GetPositionFK.h>
#include <moveit/move_group/capability_names.h>
#include <block_printer_moveit_interface/common.h>
#include <block_printer_moveit_interface/MoveitPlanner.h>

#define START_BENCH \
    do              \
    {               \
    auto _bench_start = ros::Time::now()
#define END_BENCH(_duration)                 \
    auto _bench_end = ros::Time::now();      \
    (_duration) = _bench_end - _bench_start; \
    }                                        \
    while (0)

using namespace geometry_msgs;
using namespace sensor_msgs;
using namespace trajectory_msgs;
using namespace moveit_msgs;
using namespace moveit::planning_interface;
using namespace block_printer_moveit_interface;

MoveitPlanner::MoveitPlanner(ros::NodeHandle &node,
                             const std::string &moveGroupName,
                             const std::string &referenceFrame,
                             const std::string &endEffectorLink)
    : moveGroup(moveGroupName)
{
    if (!referenceFrame.empty())
    {
        this->moveGroup.setPoseReferenceFrame(referenceFrame);
    }
    if (!endEffectorLink.empty())
    {
        this->moveGroup.setEndEffectorLink(endEffectorLink);
    }
    this->fkServiceClient = node.serviceClient<GetPositionFK>(move_group::FK_SERVICE_NAME);
    this->ikServiceClient = node.serviceClient<GetPositionIK>(move_group::IK_SERVICE_NAME);
    this->fkServiceClient.waitForExistence();
    this->ikServiceClient.waitForExistence();
}

void MoveitPlanner::addWaypoint(InterpolationType interpolationType, const geometry_msgs::Pose &pose)
{
    geometry_msgs::PoseStamped poseStamped;
    poseStamped.header.frame_id = this->moveGroup.getPoseReferenceFrame();
    poseStamped.pose = pose;
    Waypoint waypoint(interpolationType, poseStamped);
    this->waypoints.push_back(waypoint);
}

void MoveitPlanner::clearWaypoints()
{
    this->waypoints.clear();
}

const MoveGroupInterface &MoveitPlanner::getMoveGroup() const
{
    return this->moveGroup;
}

geometry_msgs::PoseStamped MoveitPlanner::getCurrentPose()
{
    return this->moveGroup.getCurrentPose();
}

bool MoveitPlanner::toManipulatorJoint(const geometry_msgs::PoseStamped &poseStamped, sensor_msgs::JointState &jointState)
{
    GetPositionIK ik;
    ik.request.ik_request.group_name = this->moveGroup.getName();
    robotStateToRobotStateMsg(*this->moveGroup.getCurrentState(), ik.request.ik_request.robot_state);
    ik.request.ik_request.pose_stamped = poseStamped;
    ik.request.ik_request.timeout = ros::Duration(1.0);
    auto value = ikServiceClient.call(ik);
    jointState = ik.response.solution.joint_state;
    return value;
}

bool MoveitPlanner::toPose(const std::vector<double> &joints, geometry_msgs::PoseStamped &pose)
{
    GetPositionFK fk;
    fk.request.header.frame_id = this->moveGroup.getEndEffectorLink();
    fk.request.fk_link_names = {this->moveGroup.getEndEffectorLink()};
    robotStateToRobotStateMsg(*this->moveGroup.getCurrentState(), fk.request.robot_state);
    fk.request.robot_state.joint_state.position = joints;
    auto value = fkServiceClient.call(fk);
    pose = fk.response.pose_stamped.at(0);
    return value;
}

MoveItErrorCode MoveitPlanner::move(double maxVelocityFactor,
                                    double maxAccelerationFactor,
                                    double cartesianMovePointStep,
                                    double cartesianMoveJumpThreshold,
                                    ros::Duration &planDuration,
                                    ros::Duration &executeDuration)
{
    MoveItErrorCode value;
    auto groups = splitWaypointGroupsByInterpolation(this->waypoints);
    RobotState startState;
    robotStateToRobotStateMsg(*this->moveGroup.getCurrentState(), startState);
    MoveGroupInterface::Plan plan;
    plan.start_state_ = startState;
    //start bench for planning time
    START_BENCH;
    //plan for each group
    for (auto group : groups)
    {
        if (group.empty())
        {
            continue;
        }
        switch (group.front().interpolation)
        {
        case InterpolationType::DYNAMIC:
        {
            //dynamicパスの場合はgroupにひとつしか要素がないはず
            if (group.size() != 1)
            {
                PRINT_ERROR("waypoints of dynamic trajectory must have only 1 point");
                value = MoveItErrorCodes::PLANNING_FAILED;
                goto method_end;
            }
            //
            MoveGroupInterface::Plan dynamicPlan;
            value = this->planDynamicPath(group.front(), startState, maxVelocityFactor, maxAccelerationFactor, dynamicPlan);
            if (!value)
            {
                goto method_end;
            }
            if (dynamicPlan.trajectory_.joint_trajectory.points.empty())
            {
                continue;
            }
            //update plan and start state
            this->appendPlan(plan, dynamicPlan, startState);
        }
        break;
        case InterpolationType::CARTESIAN:
        {
            MoveGroupInterface::Plan cartesianPlan;
            value = this->planCartesianPath(group, startState, maxVelocityFactor, maxAccelerationFactor, cartesianMovePointStep, cartesianMoveJumpThreshold, cartesianPlan);
            if (!value)
            {
                goto method_end;
            }
            //update plan and start state
            this->appendPlan(plan, cartesianPlan, startState);
        }
        break;
        default:
            PRINT_ERROR("unexpected interpolation type: " << group.front().interpolation);
            continue;
        }
    }
    //reset start state of movegroup
    {
        RobotState initialStartState;
        robotStateToRobotStateMsg(*this->moveGroup.getCurrentState(), initialStartState);
        this->moveGroup.setStartState(initialStartState);
        plan.start_state_ = initialStartState;
    }
    //end bench for planning time
    END_BENCH(planDuration);
    //execute
    START_BENCH;
    value = this->moveGroup.execute(plan);
    END_BENCH(executeDuration);
    //
method_end:
    return value;
}

MoveItErrorCode MoveitPlanner::planDynamicPath(const Waypoint &waypoint,
                                               const RobotState &startState,
                                               double maxVelocityFactor,
                                               double maxAccelerationFactor,
                                               moveit::planning_interface::MoveGroupInterface::Plan &plan)
{
    //
    if (waypoint.interpolation != InterpolationType::DYNAMIC)
    {
        PRINT_ERROR("MoveitPlanner::planDynamicPath() allows only waypoints with InterpolationType::DYNAMIC");
        return MoveItErrorCodes::PLANNING_FAILED;
    }
    //set start
    this->moveGroup.setStartState(startState);
    //set goal (joint type)
    JointState jointState;
    if (!this->toManipulatorJoint(waypoint.poseStamped, jointState))
    {
        PRINT_ERROR("failed to call ik service.");
        return MoveItErrorCode::NO_IK_SOLUTION;
    }
    this->moveGroup.setJointValueTarget(jointState);
    //plan
    return this->moveGroup.plan(plan);
}

MoveItErrorCode MoveitPlanner::planCartesianPath(const WaypointGroup &group,
                                                 const RobotState &startState,
                                                 double maxVelocityFactor,
                                                 double maxAccelerationFactor,
                                                 double pointStep,
                                                 double jumpThreshold,
                                                 moveit::planning_interface::MoveGroupInterface::Plan &plan)
{
    if (group.empty())
    {
        PRINT_WARN("MoveitPlanner::planCartesianPath() was requested to plan with empty group. no motion plan was made.");
        return MoveItErrorCodes::SUCCESS;
    }
    for (auto p : group)
    {
        if (p.interpolation != InterpolationType::CARTESIAN)
        {
            PRINT_ERROR("MoveitPlanner::planCartesianPath() allows only group with InterpolationType::CARTESIAN");
            return MoveItErrorCodes::PLANNING_FAILED;
        }
    }
    //get waypoints(Pose)
    std::vector<Pose> waypointPoses;
    waypointPoses.reserve(group.size());
    for (auto p : group)
    {
        waypointPoses.push_back(p.poseStamped.pose);
    }
    //set start
    this->moveGroup.setStartState(startState);
    //plan
    MoveItErrorCode value;
    plan.start_state_ = startState;
    auto followed = this->moveGroup.computeCartesianPath(waypointPoses, pointStep, jumpThreshold, plan.trajectory_, false, &value);
    if (followed < 0.9)
    {
        PRINT_WARN("cartesian path followed only " << followed * 100.0 << "%% of requested trajectory.");
        PRINT_WARN("the waypoints are as below");
        for (auto pose : waypointPoses)
        {
            PRINT_WARN(pose);
        }
        PRINT_WARN("you really want execute this cartesian path? [y/n]");
        std::string input;
        std::cin >> input;
        if (input != "y")
        {
            return MoveItErrorCodes::PLANNING_FAILED;
        }
    }
    //apply velocity and acceleration
    this->applyVelocityAndAccelerationFactors(plan.trajectory_.joint_trajectory, maxVelocityFactor, maxAccelerationFactor);
    return value;
}

void MoveitPlanner::appendPlan(MoveGroupInterface::Plan &previous,
                               const MoveGroupInterface::Plan &next,
                               RobotState &nextStartState) const
{
    auto hasPlanned = !previous.trajectory_.joint_trajectory.points.empty();
    auto timeAdjustment = hasPlanned ? previous.trajectory_.joint_trajectory.points.back().time_from_start : ros::Duration(0);
    //これが初めてのプラニングなら，軌跡の点以外の情報をコピー
    if (!hasPlanned)
    {
        previous.trajectory_.joint_trajectory.header = next.trajectory_.joint_trajectory.header;
        previous.trajectory_.joint_trajectory.joint_names = next.trajectory_.joint_trajectory.joint_names;
    }
    //add trajectory point
    for (int i = 0; i < next.trajectory_.joint_trajectory.points.size(); i++)
    {
        //以前のプランの最後の点=追加するプランの最初の点なので，追加するプランの最初の点をとばす
        if (hasPlanned && i == 0)
        {
            continue;
        }
        //
        auto trajectoryPoint = next.trajectory_.joint_trajectory.points[i];
        auto timeFromStart = trajectoryPoint.time_from_start + timeAdjustment;
        auto addedPoint = trajectoryPoint;
        trajectoryPoint.time_from_start = timeFromStart;
        previous.trajectory_.joint_trajectory.points.push_back(trajectoryPoint);
    }
    //update start state
    nextStartState.joint_state.position = previous.trajectory_.joint_trajectory.points.back().positions;
    nextStartState.joint_state.velocity = previous.trajectory_.joint_trajectory.points.back().velocities;
    nextStartState.joint_state.effort = previous.trajectory_.joint_trajectory.points.back().effort;
}

void MoveitPlanner::applyVelocityAndAccelerationFactors(trajectory_msgs::JointTrajectory &trajectory, double maxVelocityFactor, double maxAccelerationFactor)
{
    //経由点が2点以下だと速度情報が入っていないので，その場合はデータを加工せず終了
    if (trajectory.points.size() < 3)
    {
        PRINT_WARN("At least 3 points are needed to apply velocity or acceleration factor to a trajectory.");
        return;
    }
    //get limits of each joint's velocity and accelaration
    auto modelPtr = moveGroup.getRobotModel();
    auto jointNames = moveGroup.getJointNames();
    std::vector<double> maxVelocities;
    std::vector<double> maxAccelerations;
    maxVelocities.reserve(jointNames.size());
    maxAccelerations.reserve(jointNames.size());
    for (auto name : jointNames)
    {
        auto bounds = modelPtr->getJointModel(name)->getVariableBounds();
        auto maxVelocity = 1.0;
        auto maxAcceleration = 1.0;
        for (auto bound : bounds)
        {
            if (bound.velocity_bounded_)
            {
                maxVelocity = std::max(maxVelocity, bound.max_velocity_);
            }
            else
            {
                PRINT_WARN("the velocity of " << name << " is not bound.");
            }
            if (bound.acceleration_bounded_)
            {
                maxAcceleration = std::max(maxAcceleration, bound.max_acceleration_);
            }
            else
            {
                PRINT_WARN("the acceleration of " << name << " is not bound.");
            }
        }
        maxVelocities.push_back(maxVelocity);
        maxAccelerations.push_back(maxAcceleration);
    }
    //get max velocity factor of source trajectory
    auto previousMaxVelocityFactor = std::numeric_limits<double>::min();
    auto previousMaxAccelerationFactor = std::numeric_limits<double>::min();
    for (auto point : trajectory.points)
    {
        for (int jointIndex = 0; jointIndex < jointNames.size(); jointIndex++)
        {
            auto vFactor = point.velocities[jointIndex] / maxVelocities[jointIndex];
            auto aFactor = point.accelerations[jointIndex] / maxAccelerations[jointIndex];
            previousMaxVelocityFactor = std::max(previousMaxVelocityFactor, vFactor);
            previousMaxAccelerationFactor = std::max(previousMaxAccelerationFactor, aFactor);
        }
    }
    //以前の速度，加速度の何倍にするか決定
    double velocityChangeFactor, accelerationChangeFactor;
    {
        auto vChangeFactor = maxVelocityFactor / previousMaxVelocityFactor;
        auto aChangeFactor = maxAccelerationFactor / previousMaxAccelerationFactor;
        velocityChangeFactor = std::min(vChangeFactor, std::pow(aChangeFactor, 0.5));
        accelerationChangeFactor = std::pow(velocityChangeFactor, 2.0);
    }
    //軌跡各点の時刻を調節するための変数
    ros::Duration lastTimeFromStart = trajectory.points.front().time_from_start;
    //
    for (int pointIndex = 0; pointIndex < trajectory.points.size(); pointIndex++)
    {
        //update velocities and accelerations
        for (int jointIndex = 0; jointIndex < jointNames.size(); jointIndex++)
        {
            trajectory.points[pointIndex].velocities[jointIndex] *= velocityChangeFactor;
            trajectory.points[pointIndex].accelerations[jointIndex] *= accelerationChangeFactor;
        }
        //update time stamp
        auto previousStart = trajectory.points[pointIndex].time_from_start;
        auto duration = previousStart - lastTimeFromStart;
        auto changedDuration = duration;
        auto updatedStart = previousStart + changedDuration.fromNSec(duration.toNSec() / velocityChangeFactor);
        trajectory.points[pointIndex].time_from_start = updatedStart;
        lastTimeFromStart = updatedStart;
    }
}