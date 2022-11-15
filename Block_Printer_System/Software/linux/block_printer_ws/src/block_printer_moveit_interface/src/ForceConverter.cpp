
#include <ros/ros.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <urdf/model.h>
#include <block_printer_moveit_interface/common.h>
#include <block_printer_moveit_interface/ForceConverter.h>

using namespace sensor_msgs;
using namespace moveit::core;
using namespace moveit::planning_interface;
using namespace block_printer_moveit_interface;

static const int FORCE_AXIS_NUM = 6;
static std::vector<double> maxJointEfforts = {};

static bool getMaxEffortsFromUrdf(MoveGroupInterface &moveGroup)
{
    urdf::Model model;
    if (!maxJointEfforts.empty())
    {
        return true;
    }
    if (!model.initParam("robot_description"))
    {
        PRINT_ERROR("failed to get robot model");
        return false;
    }
    auto jointNames = moveGroup.getJointNames();
    for (auto jointName : jointNames)
    {
        auto joint = model.getJoint(jointName);
        if (!joint)
        {
            PRINT_ERROR("the robot model does not have joint \'" << jointName << "\'");
        }
        if (joint->type != urdf::Joint::REVOLUTE || joint->type != urdf::Joint::PRISMATIC)
        {
            continue;
        }
        auto maxEffort = joint->limits->effort;
        if (maxEffort < std::numeric_limits<double>::min())
        {
            PRINT_WARN("the effort of joint \'" << jointName << "\' cannot be less than " << std::numeric_limits<double>::min());
            PRINT_WARN("instead, use default value 1.0");
            maxEffort = 1.0;
        }
        maxJointEfforts.push_back(maxEffort);
    }
    return true;
}

static std::vector<double> toStdVector(const Eigen::VectorXd &eigenV)
{
    std::vector<double> value;
    value.reserve(eigenV.size());
    for (int i = 0; i < eigenV.size(); i++)
    {
        value.push_back(eigenV[i]);
    }
    return value;
}

static Eigen::VectorXd toEigenVector(const std::vector<double> &stdV)
{
    Eigen::VectorXd value;
    value.resize(stdV.size());
    for (int i = 0; i < stdV.size(); i++)
    {
        value[i] = stdV[i];
    }
    return value;
}

static bool getJacobian(const MoveGroupInterface &moveGroup, const JointState &jointState, Eigen::MatrixXd &jacobian)
{
    RobotState robotState(moveGroup.getRobotModel());
    {
        moveit_msgs::RobotState robotStateMsg;
        robotStateMsg.joint_state = jointState;
        robotStateMsgToRobotState(robotStateMsg, robotState, false);
    }
    auto robotModelPtr = moveGroup.getRobotModel();
    auto jointModelGroupNames = robotModelPtr->getJointModelGroupNames();
    if (jointModelGroupNames.empty())
    {
        PRINT_ERROR("there are no JointModelGroup");
        return false;
    }
    for (auto name : jointModelGroupNames)
    {
        PRINT_INFO("joint model group: " << name);
    }
    auto jointModelGroupPtr = robotModelPtr->getJointModelGroup(jointModelGroupNames.front());
    auto linkPtr = robotModelPtr->getLinkModel(moveGroup.getEndEffectorLink());
    Eigen::Vector3d referencePointPosition(0, 0, 0);
    return robotState.getJacobian(jointModelGroupPtr, linkPtr, referencePointPosition, jacobian);
}

static bool getMaxForces(const Eigen::MatrixXd &jacobian, const std::vector<double> &gravityCompensationJointEfforts, std::vector<double> &maxForces)
{
    try
    {
        auto inverse = jacobian.transpose().inverse();
        maxForces.resize(FORCE_AXIS_NUM);
        std::vector<double> gravityCompensations(FORCE_AXIS_NUM, 0.0);
        if (maxJointEfforts.size() == gravityCompensationJointEfforts.size())
        {
            gravityCompensations = gravityCompensationJointEfforts;
        }
        else
        {
            PRINT_WARN("gravity compensation vector has different num elements");
            PRINT_WARN("No gravity compensation will be applied");
        }
        for (int forceIndex = 0; forceIndex < FORCE_AXIS_NUM; forceIndex++)
        {
            double maxEffort = 0.0;
            for (int jointIndex = 0; jointIndex < maxJointEfforts.size(); jointIndex++)
            {
                double matrixElement = inverse(forceIndex, jointIndex);
                auto maxJointElement = (matrixElement >= 0) ? maxJointEfforts[jointIndex] : -maxJointEfforts[jointIndex];
                auto gravityElement = gravityCompensations[jointIndex];
                maxEffort += matrixElement * (maxJointElement - gravityElement);
            }
            //
            maxForces[forceIndex] = maxEffort;
        }
    }
    catch (...)
    {
        return false;
    }
    return true;
}

static bool getDesiredForcePercents(const std::vector<double> &constraints, const std::vector<double> &maxForces, std::vector<double> &forceFactors)
{
    //デフォルト設定では，全方向の力を最大(100%)まで出して良いことにする
    forceFactors = std::vector<double>(FORCE_AXIS_NUM, 100.0);
    if (constraints.size() != FORCE_AXIS_NUM)
    {
        PRINT_WARN("constraints have different size. desired:" << FORCE_AXIS_NUM << " actual:" << constraints.size());
        PRINT_WARN("no constraints will be applied");
        return false;
    }
    if (maxForces.size() != FORCE_AXIS_NUM)
    {
        PRINT_WARN("maxForces have different size. desired:" << FORCE_AXIS_NUM << " actual:" << maxForces.size());
        PRINT_WARN("no constraints will be applied");
        return false;
    }
    //
    for (int constraintIndex = 0; constraintIndex < constraints.size(); constraintIndex++)
    {
        if (constraints[constraintIndex] < 0)
        {
            continue;
        }
        //制約が設けれられている方向があれば，その方向の目標発揮力[%]を代入する
        forceFactors[constraintIndex] = std::abs(constraints[constraintIndex] / maxForces[constraintIndex]) * 100.0;
    }
    return true;
}

bool block_printer_moveit_interface::getForceLimits(const std::string &forceLimitType,
                                                    const MoveGroupInterface &moveGroup,
                                                    const JointState &jointState,
                                                    const std::vector<double> &constraints,
                                                    std::vector<double> &forceFactors)
{
    //力がN単位なら，与えられた制約条件をそのまま解にする
    if (forceLimitType == "N")
    {
        forceFactors = constraints;
        return true;
    }
    //get jacobian
    Eigen::MatrixXd jacobian;
    if (!getJacobian(moveGroup, jointState, jacobian))
    {
        PRINT_ERROR("failed to get jacobian");
        return false;
    }
    //get max force vector
    std::vector<double> maxForces;
    if (!getMaxForces(jacobian, jointState.effort, maxForces))
    {
        PRINT_ERROR("failed to get max force vector");
        return false;
    }
    //
    //get desired force vector
    if (!getDesiredForcePercents(constraints, maxForces, forceFactors))
    {
        PRINT_ERROR("failed to get desired force vector");
        return false;
    }
    return true;
}
