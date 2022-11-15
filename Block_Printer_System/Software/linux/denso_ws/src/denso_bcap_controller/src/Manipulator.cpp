#include <ros/service_client.h>
#include <trajectory_reducer/CapableNames.h>
#include <trajectory_reducer/ReduceTrajectory.h>
#include <denso_bcap_controller/macro.h>
#include <denso_bcap_controller/function.h>
#include <denso_bcap_controller/Manipulator.h>
#include <urdf/model.h>

static std::vector<std::string> JOINT_NAMES = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

static const int ManipulatorSplinePointNumMax = denso_bcap_controller::DENSO_BCAP_VARIABLE_NUM;

//I variable
static const int CURVE_NUM_INDEX = 0;
static const int PRECISE_GOAL_INDEX = 1;

//F variable
static const int FORCE_LIMIT_VARIABLE_FIRST_INDEX = 0;
static const int FORCE_LIMIT_VARIABLE_NUM = 6;

static const int TRAJECTORY_SPEED_INDEX = 6;

//task
static const std::string POSE_TRAJECTORY_TASK_PATH = "xdntaiyoesg.BlockPrinterPoseTrajectory";
static const std::string JOINT_TRAJECTORY_TASK_PATH = "xdntaiyoesg.BlockPrinterJointTrajectory";
static const std::string SLAVE_TASK = "RobSlave";

//commands for slave mode
static const std::string SLAVE_CHANGE_MODE_COMMAND = "slvChangeMode";
static const std::string SLAVE_MOVE_COMMAND = "slvMove";

using namespace control_msgs;
using namespace sensor_msgs;
using namespace trajectory_msgs;
using namespace actionlib;
using namespace trajectory_reducer;
using namespace denso_bcap_controller;

//スレーブモードの指定
enum BCAP_SLAVE_MODE : int32_t
{
    //スレーブモードをOFFにする
    NOSLAVE = 0x000,
    //Joint型指令を非同期的に送信する．タイムアウト検出はコントローラ側で設定できる
    SLAVE_JOINT_ASYNC = 0x102,
};

Manipulator::Manipulator(RcController &controller, const std::string &robotDescriptionName)
    : Object(controller),
      curveNumVariable(controller, CURVE_NUM_INDEX),
      preciseGoalVariable(controller, PRECISE_GOAL_INDEX),
      speedVariable(controller, TRAJECTORY_SPEED_INDEX),
      jointTrajectoryTask(controller, JOINT_TRAJECTORY_TASK_PATH),
      slaveTask(controller, SLAVE_TASK),
      isSlaveMode(false)
{
    //get robot status from robot description
    {
        urdf::Model model;
        if (!model.initParam(robotDescriptionName))
        {
            PRINT_ERROR("failed to get robot description from param \'" << robotDescriptionName << "\'");
        }
        //get root link name
        rootLinkName = model.getRoot()->name;
    }
    //connect to the controller
    this->controller.ControllerGetRobot(this->controller.getHandle(), "", "", &this->robotHandle);
    this->controller.RobotGetVariable(this->robotHandle, "@CURRENT_ANGLE", "", &this->angleHandle);
    //
    this->forceLimitVariables.reserve(FORCE_LIMIT_VARIABLE_NUM);
    for (int i = FORCE_LIMIT_VARIABLE_FIRST_INDEX; i < FORCE_LIMIT_VARIABLE_FIRST_INDEX + FORCE_LIMIT_VARIABLE_NUM; i++)
    {
        this->forceLimitVariables.push_back(Float32(controller, i));
    }
    this->setForceLimitPercents(100.0, 100.0, 100.0, 100.0, 100.0, 100.0);
    //allocate all J type variables for run trajectory
    this->jointTrajectoryVariables.reserve(DENSO_BCAP_VARIABLE_NUM);
    for (int i = DENSO_BCAP_VARIABLE_INDEX_MIN; i <= DENSO_BCAP_VARIABLE_INDEX_MAX; i++)
    {
        this->jointTrajectoryVariables.push_back(RadianJoint(controller, i));
    }
}

Manipulator::~Manipulator()
{
    this->controller.VariableRelease(this->angleHandle);
    this->controller.RobotRelease(this->robotHandle);
}

BCAP_HRESULT Manipulator::setSlaveMode(bool slave)
{
    auto runningResult = this->slaveTask.isRunning();
    if (!isBcapSuccess(runningResult.second))
    {
        return runningResult.second;
    }
    if (slave && !runningResult.first)
    {
        auto result = this->slaveTask.run(true);
        if (!isBcapSuccess(result))
        {
            return result;
        }
        //wait for slave task to start
        ros::Duration(1.0).sleep();
    }
    else if (!slave && runningResult.first)
    {
        auto result = this->slaveTask.stop();
        if (!isBcapSuccess(result))
        {
            return result;
        }
    }
    this->isSlaveMode = slave;
    auto slaveMode = slave ? BCAP_SLAVE_MODE::SLAVE_JOINT_ASYNC : BCAP_SLAVE_MODE::NOSLAVE;
    long result;
    return this->controller.RobotExecute2(this->robotHandle, SLAVE_CHANGE_MODE_COMMAND, VT_I4, 1, &slaveMode, &result);
}

std::pair<sensor_msgs::JointState, BCAP_HRESULT> Manipulator::getJointState()
{
    static int sequence = 0;
    JointState value;
    value.header.frame_id = this->rootLinkName;
    value.header.seq = sequence++;
    value.header.stamp = ros::Time::now();
    value.name = JOINT_NAMES;
    //
    float jointDegrees[8];
    auto result = this->controller.VariableGetValue(this->angleHandle, jointDegrees);
    if (!isBcapSuccess(result))
    {
        PRINT_ERROR("failed to get joint positions");
        return std::make_pair(value, result);
    }
    for (int i = 0; i < JOINT_NAMES.size(); i++)
    {
        value.position.push_back(toRadian(jointDegrees[i]));
    }
    return std::make_pair(value, result);
}

BCAP_HRESULT Manipulator::setPreciseGoal(bool enablePreciseGoal)
{
    int32_t value = enablePreciseGoal ? 1 : 0;
    return this->preciseGoalVariable.set(value);
}

BCAP_HRESULT Manipulator::setVelocityFactor(double factor)
{
    auto rounded = std::max(1.0, std::min(100.0, factor * 100.0));
    return this->speedVariable.set((float)(rounded));
}

BCAP_HRESULT Manipulator::setForceLimitPercents(double x, double y, double z, double rx, double ry, double rz)
{
    std::array<double, FORCE_LIMIT_VARIABLE_NUM> v = {x, y, z, rx, ry, rz};
    for (int i = 0; i < FORCE_LIMIT_VARIABLE_NUM; i++)
    {
        auto percentage = std::max(10.0, std::min(100.0, v[i]));
        auto result = this->forceLimitVariables[i].set((float)percentage);
        if (!isBcapSuccess(result))
        {
            return result;
        }
    }
    this->isForceLimited = (x + y + z + rx + ry + rz) / 6.0 < 1.0;
    return BCAP_HRESULT::BCAP_S_OK;
}

bool Manipulator::executeAction(const JointTrajectory &trajectory, const ros::Publisher &jointStatePublisher, bool enableTrajectoryFilter)
{
    //力制限中はスレーブモードでは動作させない
    if (this->isSlaveMode && !this->isForceLimited)
    {
        this->executeActionSlave(trajectory);
    }
    else
    {
        this->executeActionNormal(trajectory, jointStatePublisher, enableTrajectoryFilter);
    }
}

bool Manipulator::executeActionSlave(const trajectory_msgs::JointTrajectory &trajectory)
{
    auto points = trajectory.points;
    //最初の点(points[0])はスタートと同じ姿勢なので飛ばす
    for (int i = 1; i < points.size(); i++)
    {
        //abort if the controller has error
        if (this->controller.hasError())
        {
            return false;
        }
        //
        auto startTime = ros::Time::now();
        auto previous = points[i - 1];
        auto next = points[i];
        //create send data
        float send[7];
        float response[7];
        for (int i = 0; i < next.positions.size(); i++)
        {
            send[i] = (float)toDegree(next.positions[i]);
        }
        send[6] = 0.0;
        //send a trajectory point
        auto result = this->controller.RobotExecute2(this->robotHandle, SLAVE_MOVE_COMMAND, VT_R4 | VT_ARRAY, 7, send, response);
        PRINT_INFO("executed trajectory point" << i << " by slave mode");
        if (!isBcapSuccess(result))
        {
            PRINT_ERROR("while calling ControllerExecute2: " << getBcapMessage(result));
            return false;
        }
        //create actual trajectory point
        JointTrajectoryPoint actualPoint;
        actualPoint.time_from_start = next.time_from_start;
        for (int i = 0; i < next.positions.size(); i++)
        {
            actualPoint.positions.push_back((float)toRadian(response[i]));
        }
        //wait for next point
        auto idealDuration = next.time_from_start - previous.time_from_start;
        auto actualDuration = ros::Time::now() - startTime;
        auto duration = idealDuration - actualDuration;
        duration.sleep();
    }
    return true;
}

bool Manipulator::executeActionNormal(const trajectory_msgs::JointTrajectory &trajectory, const ros::Publisher &jointStatePublisher, bool enableTrajectoryFilter)
{
    std::vector<JointTrajectory> trajectoryGroup;
    if (!this->getTrajectoryGroup(trajectory, trajectoryGroup, enableTrajectoryFilter))
    {
        return false;
    }
    //execute each trajectory
    for (auto seperatedTrajectory : trajectoryGroup)
    {
        //set trajectory point to controller's variables
        {
            auto setResult = this->curveNumVariable.set((int32_t)seperatedTrajectory.points.size());
            if (!isBcapSuccess(setResult))
            {
                PRINT_ERROR("while setting the number of waypoint: " << getBcapMessage(setResult));
                return false;
            }
        }
        for (int i = DENSO_BCAP_VARIABLE_INDEX_MIN; i < (int)seperatedTrajectory.points.size(); i++)
        {
            auto radians = seperatedTrajectory.points[i].positions;
            auto radianSetResult = this->jointTrajectoryVariables[i].set(radians);
            if (!isBcapSuccess(radianSetResult))
            {
                PRINT_ERROR("while setting a waypoint: " << getBcapMessage(radianSetResult));
                return false;
            }
        }
        //execute a task
        {
            auto runResult = this->jointTrajectoryTask.run(true);
            if (!isBcapSuccess(runResult))
            {
                PRINT_ERROR("while try to run trajectory task: " << getBcapMessage(runResult));
                return false;
            }
        }
        //wait for finish of the task
        {
            while (true)
            {
                auto runningResult = this->jointTrajectoryTask.isRunning();
                if (!isBcapSuccess(runningResult.second))
                {
                    PRINT_ERROR("while waiting for finish of trajectory task: " << getBcapMessage(runningResult.second));
                    return false;
                }
                if (!runningResult.first)
                {
                    break;
                }
                jointStatePublisher.publish(this->getJointState().first);
                ros::spinOnce();
                ros::Duration(0.05).sleep();
            }
        }
    }
    //軌道を実行し終えた後の位置を発行
    jointStatePublisher.publish(this->getJointState().first);
    return true;
}

bool Manipulator::getTrajectoryGroup(const JointTrajectory &trajectory, std::vector<JointTrajectory> &trajectoryGroup, bool enableTrajectoryFilter)
{
    auto desiredSize = trajectory.points.size();
    auto splineSize = this->jointTrajectoryVariables.size();
    trajectoryGroup.clear();
    if (enableTrajectoryFilter)
    {
        //reduce trajectory points
        if (desiredSize > splineSize)
        {
            ReduceTrajectory reduceTrajectory;
            reduceTrajectory.request.trajectory_source = trajectory;
            reduceTrajectory.request.max_point = splineSize;
            if (!ros::service::call(trajectory_reducer::REDUCE_TRAJECTORY_SERVICE, reduceTrajectory))
            {
                PRINT_ERROR("failed to call trajectory point reducer");
                return false;
            }
            if (reduceTrajectory.response.reduced)
            {
                PRINT_WARN("trajectory points were reduced from " << reduceTrajectory.request.trajectory_source.points.size() << " to " << reduceTrajectory.response.trajectory.points.size());
            }
            trajectoryGroup.push_back(reduceTrajectory.response.trajectory);
        }
        else
        {
            trajectoryGroup.push_back(trajectory);
        }
    }
    else
    {
        int trajectoryIndex = 0;
        JointTrajectory seperatedTrajectory;
        while (trajectoryIndex < desiredSize)
        {
            auto takeNum = (trajectoryIndex + splineSize > desiredSize) ? (desiredSize - trajectoryIndex) : splineSize;
            for (auto i = 0; i < takeNum; i++)
            {
                seperatedTrajectory.points.push_back(trajectory.points[trajectoryIndex + i]);
            }
            trajectoryGroup.push_back(seperatedTrajectory);
            seperatedTrajectory.points.clear();
            trajectoryIndex += takeNum;
        }
    }
    if (trajectoryGroup.size() > 1)
    {
        PRINT_WARN("trajectory with " << desiredSize << " points has been seperated into " << trajectoryGroup.size() << " groups");
    }
    return true;
}
