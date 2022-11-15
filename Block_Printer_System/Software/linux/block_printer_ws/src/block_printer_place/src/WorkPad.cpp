#include <ros/console.h>
#include <ros/param.h>
#include <ros/service.h>
#include <block_printer_model_reader/CapableNames.h>
#include <block_printer_model_reader/GetBlockState.h>
#include <block_printer_model_reader/GetComponentBlock.h>
#include <block_printer_place/WorkPad.h>

using namespace block_printer_model_reader;
using namespace block_printer_place;

static const std::string logHeader = "place";

#define PRINT_INFO(args) ROS_INFO_STREAM(logHeader << ": " << args)
#define PRINT_WARN(args) ROS_WARN_STREAM(logHeader << ": " << args)
#define PRINT_ERROR(args) ROS_ERROR_STREAM(logHeader << ": " << args)

WorkPad::BlockCalibration::BlockCalibration()
{
}

WorkPad::BlockCalibration::BlockCalibration(const WorkPad::BlockCalibration &other) : blockSize(other.blockSize),
                                                                                      planeX(other.planeX),
                                                                                      planeY(other.planeY),
                                                                                      planeZ(other.planeZ),
                                                                                      orientation(other.orientation)
{
}

WorkPad::BlockCalibration &WorkPad::BlockCalibration::operator=(const WorkPad::BlockCalibration &other)
{
    this->blockSize = other.blockSize;
    this->planeX = other.planeX;
    this->planeY = other.planeY;
    this->planeZ = other.planeZ;
    this->orientation = other.orientation;
    return *this;
}

WorkPad::WorkPad() {}

WorkPad::WorkPad(const WorkPad &other) : padIndex(other.padIndex),
                                         size(other.size),
                                         calibrations(other.calibrations),
                                         shiftGridRatio(other.shiftGridRatio) {}

WorkPad::~WorkPad() {}

bool WorkPad::getPoseAt(GetPlacePoseRequest &request, GetPlacePoseResponse &response) const
{
    //ブロックの設定を取得する
    GetBlockState getBlockState;
    if (!ros::service::call<GetBlockState>(block_printer_model_reader::BLOCK_STATE_SERVICE, getBlockState))
    {
        PRINT_ERROR("failed to get block state");
        return false;
    }
    //ブロックを置くときのオプションを取得する
    GetComponentBlock getComponentBlock;
    getComponentBlock.request.block_index = request.block_index;
    if (!ros::service::call<GetComponentBlock>(block_printer_model_reader::COMPONENT_BLOCK_SERVICE, getComponentBlock))
    {
        PRINT_ERROR("failed to get component block");
        return false;
    }
    //対応するキャリブレーションデータを取得する
    BlockCalibration calibration;
    bool found = false;
    for (auto item : this->calibrations)
    {
        if (item.blockSize.x == getComponentBlock.response.size_x && item.blockSize.y == getComponentBlock.response.size_y)
        {
            calibration = item;
            found = true;
            break;
        }
    }
    if (!found)
    {
        PRINT_ERROR("no place information of component block:" << getComponentBlock.response);
        return false;
    }
    //
    auto grid = Point2<double>(getComponentBlock.response.position_x, getComponentBlock.response.position_y);
    if (request.enable_shift)
    {
        grid += Point2<double>(getComponentBlock.response.shift_x, getComponentBlock.response.shift_y) * this->shiftGridRatio;
    }
    response.position.x = calibration.planeX.valueAt(grid);
    response.position.y = calibration.planeY.valueAt(grid);
    response.position.z = calibration.planeZ.valueAt(grid);
    response.orientation = calibration.orientation;
    response.position.z += (getComponentBlock.response.position_z - 1) * getBlockState.response.block_size_height;
    return true;
}

std::string WorkPad::toString() const
{
    std::string value = "WorkPad: ";
    value += "size: " + std::to_string(this->size.x) + "x" + std::to_string(this->size.y);
    return value;
}

WorkPad &WorkPad::operator=(const WorkPad &other)
{
    this->padIndex = other.padIndex;
    this->size = other.size;
    this->calibrations = other.calibrations;
    return *this;
}

WorkPad WorkPad::fromRosParam(const std::string &workPadParamPath)
{
    WorkPad value;
    auto width = ros::param::param<int>(workPadParamPath + "/size/x", 0);
    auto height = ros::param::param<int>(workPadParamPath + "/size/y", 0);
    value.size = Point2<int>(width, height);
    int index = 0;
    while (true)
    {
        auto placeParamPath = workPadParamPath + "/place" + std::to_string(index);
        if (!ros::param::has(placeParamPath))
        {
            break;
        }
        auto blockSizeX = ros::param::param<int>(placeParamPath + "/block/size_x", 0);
        auto blockSizeY = ros::param::param<int>(placeParamPath + "/block/size_y", 0);
        auto leftBottomGrid = Point2<int>(0, 0);
        auto rightBottomGrid = Point2<int>(width - blockSizeX, 0);
        auto leftTopGrid = Point2<int>(0, height - blockSizeY);
        auto rightTopGrid = Point2<int>(width - blockSizeX, height - blockSizeY);
        auto leftBottomPosition = ros::param::param<std::vector<double>>(placeParamPath + "/x_min_y_min", {});
        auto rightBottomPosition = ros::param::param<std::vector<double>>(placeParamPath + "/x_max_y_min", {});
        auto leftTopPosition = ros::param::param<std::vector<double>>(placeParamPath + "/x_min_y_max", {});
        auto rightTopPosition = ros::param::param<std::vector<double>>(placeParamPath + "/x_max_y_max", {});

        auto orientation = ros::param::param<std::vector<double>>(placeParamPath + "/orientation", {});
        std::vector<double> xGrids{(double)leftBottomGrid.x, (double)rightBottomGrid.x, (double)leftTopGrid.x, (double)rightTopGrid.x};
        std::vector<double> yGrids{(double)leftBottomGrid.y, (double)rightBottomGrid.y, (double)leftTopGrid.y, (double)rightTopGrid.y};
        std::vector<double> xValues{leftBottomPosition[0], rightBottomPosition[0], leftTopPosition[0], rightTopPosition[0]};
        std::vector<double> yValues{leftBottomPosition[1], rightBottomPosition[1], leftTopPosition[1], rightTopPosition[1]};
        std::vector<double> zValues{leftBottomPosition[2], rightBottomPosition[2], leftTopPosition[2], rightTopPosition[2]};
        LeastSquarePlane planeX(xGrids, yGrids, xValues);
        LeastSquarePlane planeY(xGrids, yGrids, yValues);
        LeastSquarePlane planeZ(xGrids, yGrids, zValues);
        BlockCalibration blockCalibration;
        blockCalibration.blockSize = Point2<int>(blockSizeX, blockSizeY);
        blockCalibration.planeX = planeX;
        blockCalibration.planeY = planeY;
        blockCalibration.planeZ = planeZ;
        blockCalibration.orientation = orientation;
        value.calibrations.push_back(blockCalibration);
        index++;
    }
    //
    return value;
}

std::vector<WorkPad> WorkPad::createWorkPads()
{
    std::vector<WorkPad> value;
    int index = 0;
    while (true)
    {
        std::string workPadParamPath = "/block_printer/pad" + std::to_string(index);
        if (!ros::param::has(workPadParamPath))
        {
            break;
        }
        WorkPad workPad = fromRosParam(workPadParamPath);
        workPad.padIndex = index;
        workPad.shiftGridRatio = ros::param::param<double>("/block_printer/shift_grid_ratio", 0);
        value.push_back(workPad);
        index++;
        PRINT_INFO("add calibration information of WorkPad" << index);
    }
    return value;
}
