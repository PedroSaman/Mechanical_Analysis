#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <block_printer_model_reader/CapableNames.h>
#include <block_printer_model_reader/GetBlockState.h>
#include <block_printer_model_reader/GetComponentBlock.h>
#include <block_printer_model_reader/GetBlockState.h>
#include <block_printer_pick/CapableNames.h>
#include <block_printer_pick/GetPickPose.h>
#include <block_printer_place/CapableNames.h>
#include <block_printer_place/GetPlacePose.h>
#include <block_printer_visualizer/CapableNames.h>
#include <block_printer_visualizer/SpawnObject.h>
#include <block_printer_visualizer/SpawnBlock.h>
#include <block_printer_visualizer/AttachBlock.h>
#include <block_printer_visualizer/DetachBlock.h>
#include <block_printer_visualizer/DeleteBlock.h>

using namespace moveit::planning_interface;
using namespace block_printer_model_reader;
using namespace block_printer_pick;
using namespace block_printer_place;
using namespace block_printer_visualizer;

static const std::string nodeName = "block_spawner";

#define PRINT_INFO(args) ROS_INFO_STREAM(nodeName << ": " << args)
#define PRINT_ERROR(args) ROS_ERROR_STREAM(nodeName << ": " << args)

static struct AttachedBlockInfo
{
    int index;
    std::string referenceFrame;
    GetBlockStateResponse blockState;
    GetComponentBlockResponse component;
} attachedBlock;

static std::string getBlockId(int blockIndex)
{
    return "block" + std::to_string(blockIndex);
}

static shape_msgs::SolidPrimitive getBlockShape(const GetBlockStateResponse &blockState, const GetComponentBlockResponse &component)
{
    shape_msgs::SolidPrimitive value;
    auto color = component.color;
    //ブロックの高さは凸部も含める
    auto height = blockState.block_size_height + blockState.block_convex_height;
    //サポートブロックのみ円筒形
    if (color == "support")
    {
        value.type = shape_msgs::SolidPrimitive::CYLINDER;
        value.dimensions.resize(2);
        value.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = height;
        value.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = blockState.block_size_side / 2.0;
    }
    //その他のブロックは直方体
    else
    {
        //ブロックの形状を作成．ブロックの高さは凸部も含める
        value.type = shape_msgs::SolidPrimitive::BOX;
        auto sizeMax = std::max(component.size_x, component.size_y);
        auto sizeMin = std::min(component.size_x, component.size_y);
        value.dimensions.resize(3);
        value.dimensions[shape_msgs::SolidPrimitive::BOX_X] = blockState.block_size_side * sizeMax;
        value.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = blockState.block_size_side * sizeMin;
        value.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = height;
    }
    return value;
}

static std_msgs::ColorRGBA getBlockColor(const std::string &colorName)
{
    std_msgs::ColorRGBA color;
    if (colorName == "white")
    {
        color.r = 1;
        color.g = 1;
        color.b = 1;
        color.a = 1;
    }
    else if (colorName == "red")
    {
        color.r = 1;
        color.g = 0;
        color.b = 0;
        color.a = 1;
    }
    else if (colorName == "orange")
    {
        color.r = 1;
        color.g = 0.65;
        color.b = 0;
        color.a = 1;
    }
    else if (colorName == "yellow")
    {
        color.r = 1;
        color.g = 1;
        color.b = 0;
        color.a = 1;
    }
    else if (colorName == "green")
    {
        color.r = 0;
        color.g = 1;
        color.b = 0;
        color.a = 1;
    }
    else if (colorName == "blue")
    {
        color.r = 0;
        color.g = 0;
        color.b = 1;
        color.a = 1;
    }
    else if (colorName == "black")
    {
        color.r = 0.2;
        color.g = 0.2;
        color.b = 0.2;
        color.a = 1;
    }
    else if (colorName == "support")
    {
        color.r = 0.8;
        color.g = 0.8;
        color.b = 0.8;
        color.a = 0.4;
    }
    else
    {
        PRINT_ERROR("unknown mesh color: " << colorName);
    }
    return color;
}

static geometry_msgs::Pose getBlockPoseFromRobotPose(const geometry_msgs::Pose &robotPose,
                                                     const GetBlockStateResponse &blockState,
                                                     const GetComponentBlockResponse &component)
{
    geometry_msgs::Pose value = robotPose;
    //1x1ブロックの場合，爪溝の片方だけを使って把持するので，ブロックの位置を調節
    if (component.size_x == 1 && component.size_y == 1)
    {
        value.position.x += blockState.block_size_side / 2.0;
    }
    //ブロック出現高さを調節．出現z座標=ロボットツール座標系z座標-ブロック高さ(除:凸部)/2
    value.position.z -= blockState.block_size_height / 2.0;
    return value;
}

static bool spawnBlock(SpawnBlockRequest &request, SpawnBlockResponse &response)
{
    GetBlockState getBlockState;
    GetComponentBlock getComponentBlock;
    GetPickPose getPickPose;
    getComponentBlock.request.block_index = request.block_index;
    getPickPose.request.block_index = request.block_index;
    if (!ros::service::call<GetBlockState>(block_printer_model_reader::BLOCK_STATE_SERVICE, getBlockState))
    {
        PRINT_ERROR("failed to get block state");
        return false;
    }
    if (!ros::service::call<GetComponentBlock>(block_printer_model_reader::COMPONENT_BLOCK_SERVICE, getComponentBlock))
    {
        PRINT_ERROR("failed to get conponent block");
        return false;
    }
    if (!ros::service::call<GetPickPose>(block_printer_pick::PICK_POSE_SERVICE, getPickPose))
    {
        PRINT_ERROR("failed to get pick pose");
        return false;
    }
    //ブロックを出現させる
    SpawnObject spawnObject;
    spawnObject.request.object.type = Object::SHAPE_PRIMITIVE;
    spawnObject.request.object.name = getBlockId(request.block_index);
    //ブロックの形状を作成
    spawnObject.request.object.primitive = getBlockShape(getBlockState.response, getComponentBlock.response);
    spawnObject.request.object.color = getBlockColor(getComponentBlock.response.color);
    //ブロックの出現位置を作成
    spawnObject.request.object.poseStamped.header.frame_id = request.reference_frame;
    geometry_msgs::Pose pose;
    pose.position = getPickPose.response.position;
    pose.orientation = tf::createQuaternionMsgFromYaw(0);
    spawnObject.request.object.poseStamped.pose = getBlockPoseFromRobotPose(pose, getBlockState.response, getComponentBlock.response);
    //グローバル変数書き換え
    attachedBlock.index = request.block_index;
    attachedBlock.referenceFrame = request.reference_frame;
    attachedBlock.blockState = getBlockState.response;
    attachedBlock.component = getComponentBlock.response;
    //サービス呼び出し
    return ros::service::call(SPAWN_OBJECT_SERVICE, spawnObject);
}

static bool attachBlock(AttachBlockRequest &request,
                        AttachBlockResponse &response,
                        MoveGroupInterface &moveGroup,
                        PlanningSceneInterface &planningScene)
{
    auto id = getBlockId(request.block_index);
    auto link = request.attach_link;
    auto value = moveGroup.attachObject(id, link);
    auto attachedBlock = planningScene.getAttachedObjects({id}).at(id);
    value &= planningScene.applyAttachedCollisionObject(attachedBlock);
    return value;
}

static bool deleteBlock(DeleteBlockRequest &request, DeleteBlockResponse &response, MoveGroupInterface &moveGroup, PlanningSceneInterface &planningScene)
{
    auto id = getBlockId(request.block_index);
    //attachされているブロックなら，削除の前にdetachする
    if (!planningScene.getAttachedObjects({id}).empty())
    {
        moveGroup.detachObject(id);
    }
    planningScene.removeCollisionObjects({id});
    return true;
}

static bool detachBlock(DetachBlockRequest &request, DetachBlockResponse &response, MoveGroupInterface &moveGroup, PlanningSceneInterface &planningScene)
{
    auto attachedBlockId = getBlockId(attachedBlock.index);
    //detachする．ここでブロックの色情報が失われる
    moveGroup.detachObject(attachedBlockId);
    //detachしたブロックを消去
    planningScene.removeCollisionObjects({attachedBlockId});
    //プレイス位置を取得
    geometry_msgs::Pose placePose;
    {
        GetPlacePose getPlacePose;
        getPlacePose.request.block_index = attachedBlock.index;
        getPlacePose.request.assembly_pad_index = attachedBlock.component.assembly_pad_index;
        getPlacePose.request.enable_shift = false;
        if (!ros::service::call<GetPlacePose>(block_printer_place::PLACE_POSE_SERVICE, getPlacePose))
        {
            PRINT_ERROR("failed to get place pose");
            return false;
        }
        placePose.position = getPlacePose.response.position;
        geometry_msgs::Quaternion orientation = tf::createQuaternionMsgFromYaw(0);
        //1x2ブロックは90度回転
        if (attachedBlock.component.size_x < attachedBlock.component.size_y)
        {
            orientation = tf::createQuaternionMsgFromYaw(M_PI_2);
        }
        placePose.orientation = orientation;
    }
    //同じブロックをプレイス位置に再登場させる(ブロックの色情報をsceneに反映させるため)
    SpawnObject spawnObject;
    spawnObject.request.object.type = Object::SHAPE_PRIMITIVE;
    spawnObject.request.object.name = attachedBlockId;
    //ブロックの形状を作成
    spawnObject.request.object.primitive = getBlockShape(attachedBlock.blockState, attachedBlock.component);
    spawnObject.request.object.color = getBlockColor(attachedBlock.component.color);
    //ブロックの出現位置を作成
    spawnObject.request.object.poseStamped.header.frame_id = attachedBlock.referenceFrame;
    spawnObject.request.object.poseStamped.pose = getBlockPoseFromRobotPose(placePose, attachedBlock.blockState, attachedBlock.component);
    //サービス呼び出し
    return ros::service::call(SPAWN_OBJECT_SERVICE, spawnObject);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, nodeName);
    ros::NodeHandle node;
    ros::NodeHandle paramNode("~");
    auto planningGroupName = paramNode.param<std::string>("reference_move_group", "");
    MoveGroupInterface moveGroup(planningGroupName);
    PlanningSceneInterface planningScene;
    ros::service::waitForService(SPAWN_OBJECT_SERVICE);
    auto spawnBlockService = node.advertiseService(SPAWN_BLOCK_SERVICE, spawnBlock);
    auto attachBlockService = node.advertiseService<AttachBlockRequest, AttachBlockResponse>(ATTACH_BLOCK_SERVICE, boost::bind(attachBlock, _1, _2, boost::ref(moveGroup), boost::ref(planningScene)));
    auto detachBlockService = node.advertiseService<DetachBlockRequest, DetachBlockResponse>(DETACH_BLOCK_SERVICE, boost::bind(detachBlock, _1, _2, boost::ref(moveGroup), boost::ref(planningScene)));
    auto deleteBlockService = node.advertiseService<DeleteBlockRequest, DeleteBlockResponse>(DELETE_BLOCK_SERVICE, boost::bind(deleteBlock, _1, _2, boost::ref(moveGroup), boost::ref(planningScene)));
    PRINT_INFO("you can start spawning blocks now");
    ros::spin();
    return 0;
}
