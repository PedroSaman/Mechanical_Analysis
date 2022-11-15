#include <fstream>
#include <iostream>
#include <ros/console.h>
#include <ros/service.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/ColorRGBA.h>
#include <shape_msgs/SolidPrimitive.h>
#include <shape_msgs/Plane.h>
#include <shape_msgs/Mesh.h>
#include <moveit_msgs/CollisionObject.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <block_printer_visualizer/CapableNames.h>
#include <block_printer_visualizer/Object.h>
#include <block_printer_visualizer/SpawnObject.h>
#include <block_printer_visualizer/SpawnSceneFile.h>

using namespace block_printer_visualizer;

static const std::string nodeName = "scene_spawner";

#define PRINT_INFO(args) ROS_INFO_STREAM(nodeName << ": " << args)
#define PRINT_ERROR(args) ROS_ERROR_STREAM(nodeName << ": " << args)

static std::string removeComment(const std::string &source, char commentChar)
{
    std::string value;
    for (auto c : source)
    {
        if (c == commentChar)
        {
            break;
        }
        value += c;
    }
    return value;
}

static std::vector<std::string> splitByWhiteSpace(const std::string &source)
{
    std::vector<std::string> value;
    std::string temp;
    auto removed = removeComment(source, '#');
    for (auto c : removed)
    {
        if (c == ' ' && !temp.empty())
        {
            value.push_back(temp);
            temp.clear();
        }
        else
        {
            temp.push_back(c);
        }
    }
    if (!temp.empty())
    {
        value.push_back(temp);
    }
    return value;
}

static std::vector<double> splitToDoublesByWhiteSpace(const std::string &source)
{
    static const double defaultSize = 1.0;
    std::vector<double> value;
    auto split = splitByWhiteSpace(source);
    for (auto s : split)
    {
        double d = defaultSize;
        try
        {
            d = std::stod(s);
        }
        catch (std::invalid_argument)
        {
            PRINT_ERROR("failed to convert string \'" << s << "\' to object size. use default value " << defaultSize << " instead.");
        }
        catch (std::out_of_range)
        {
            PRINT_ERROR("failed to convert string \'" << s << "\' to object size. use default value " << defaultSize << " instead.");
        }
        value.push_back(d);
    }
    return value;
}

bool getPrimitiveObject(const std::string &type,
                        const std::string &sizeString,
                        shape_msgs::SolidPrimitive &shape)
{
    if (type == "box")
    {
        shape.type = shape_msgs::SolidPrimitive::BOX;
    }
    else if (type == "sphere")
    {
        shape.type = shape_msgs::SolidPrimitive::SPHERE;
    }
    else if (type == "cylinder")
    {
        shape.type = shape_msgs::SolidPrimitive::CYLINDER;
    }
    else if (type == "cone")
    {
        shape.type = shape_msgs::SolidPrimitive::CONE;
    }
    else
    {
        PRINT_ERROR("unexpected shape type: " << type);
        return false;
    }
    shape.dimensions = splitToDoublesByWhiteSpace(sizeString);
    return true;
}

bool getPlaneObject(const std::string &sizeString,
                    shape_msgs::Plane &plane)
{
    auto coefs = splitToDoublesByWhiteSpace(sizeString);
    try
    {
        plane.coef = {coefs[0], coefs[1], coefs[2], coefs[3]};
    }
    catch (...)
    {
        PRINT_ERROR("failed to get plane coefficient :" << sizeString);
        return false;
    }
    return true;
}

bool readNonEmptyLine(std::ifstream &stream, std::string &s)
{
    while (true)
    {
        if (!std::getline(stream, s))
        {
            return false;
        }
        if (!s.empty())
        {
            break;
        }
    }
    return true;
}

bool spawnObjects(std::ifstream &stream, const std::string &referenceFrame)
{
    std::string sceneName;
    if (!readNonEmptyLine(stream, sceneName))
    {
        PRINT_ERROR("failed to read scene name");
        return false;
    }
    while (true)
    {
        std::string objectName;
        std::string num;
        std::string type;
        std::string sizeOrFilename;
        std::string position;
        std::string orientation;
        std::string color;
        if (!readNonEmptyLine(stream, objectName))
        {
            PRINT_ERROR("failed to read object name");
            return false;
        }
        if (objectName == ".")
        {
            break;
        }
        //物体の名前のフォーマットチェックと，最初の"* "を取り除く処理
        if (objectName.size() < 2 ||
            objectName[0] != '*' ||
            objectName[1] != ' ')
        {
            PRINT_ERROR("invalid format for object name: " << objectName);
            continue;
        }
        objectName = objectName.substr(2);
        //
        if (!readNonEmptyLine(stream, num) ||
            !readNonEmptyLine(stream, type) ||
            !readNonEmptyLine(stream, sizeOrFilename) ||
            !readNonEmptyLine(stream, position) ||
            !readNonEmptyLine(stream, orientation) ||
            !readNonEmptyLine(stream, color))
        {
            PRINT_ERROR("failed to read object status");
            return false;
        }
        //
        block_printer_visualizer::Object collisionObject;
        collisionObject.name = objectName;
        if (type == "mesh")
        {
            collisionObject.type = block_printer_visualizer::Object::SHAPE_MESH;
            collisionObject.mesh_filename = sizeOrFilename;
        }
        else if (type == "plane")
        {
            collisionObject.type = block_printer_visualizer::Object::SHAPE_PLANE;
            getPlaneObject(sizeOrFilename, collisionObject.plane);
        }
        else
        {
            collisionObject.type = block_printer_visualizer::Object::SHAPE_PRIMITIVE;
            getPrimitiveObject(type, sizeOrFilename, collisionObject.primitive);
        }
        collisionObject.poseStamped.header.frame_id = referenceFrame;
        {
            auto positionValues = splitToDoublesByWhiteSpace(position);
            if (positionValues.size() != 3)
            {
                PRINT_ERROR("invalid format for position: " << position);
                continue;
            }
            collisionObject.poseStamped.pose.position.x = positionValues[0];
            collisionObject.poseStamped.pose.position.y = positionValues[1];
            collisionObject.poseStamped.pose.position.z = positionValues[2];
        }
        {
            auto orientationValues = splitToDoublesByWhiteSpace(orientation);
            switch (orientationValues.size())
            {
            case 1:
                collisionObject.poseStamped.pose.orientation = tf::createQuaternionMsgFromYaw(orientationValues[0]);
                break;
            case 3:
                collisionObject.poseStamped.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(orientationValues[0],
                                                                                                       orientationValues[1],
                                                                                                       orientationValues[2]);
                break;
            case 4:
                collisionObject.poseStamped.pose.orientation.x = orientationValues[0];
                collisionObject.poseStamped.pose.orientation.y = orientationValues[1];
                collisionObject.poseStamped.pose.orientation.z = orientationValues[2];
                collisionObject.poseStamped.pose.orientation.w = orientationValues[3];
                break;
            default:
                PRINT_ERROR("invalid format for orientation: " << orientation);
                continue;
            }
        }
        {
            auto colors = splitToDoublesByWhiteSpace(color);
            if (colors.size() != 4)
            {
                PRINT_ERROR("invalid format for colorRGBA: " << color);
                continue;
            }
            collisionObject.color.r = colors[0];
            collisionObject.color.g = colors[1];
            collisionObject.color.b = colors[2];
            collisionObject.color.a = colors[3];
        }
        //send collision object
        block_printer_visualizer::SpawnObject spawnObject;
        spawnObject.request.object = collisionObject;
        if (!ros::service::waitForService(SPAWN_OBJECT_SERVICE) ||
            !ros::service::call(SPAWN_OBJECT_SERVICE, spawnObject))
        {
            PRINT_ERROR("failed to spawn object");
            return false;
        }
    }
    PRINT_INFO("successfully spawned a scene: " << sceneName);
    return true;
}

static bool spawnSceneFileService(SpawnSceneFileRequest &request, SpawnSceneFileResponse &response)
{
    std::ifstream stream(request.filename);
    if (stream.fail())
    {
        PRINT_ERROR("failed to open a scene file: " << request.filename);
        return false;
    }
    auto value = spawnObjects(stream, request.frame_id);
    PRINT_INFO("successfully loaded a scene file: " << request.filename);
    return value;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, nodeName);
    ros::NodeHandle node;
    ros::NodeHandle paramNode("~");
    ros::service::waitForService(SPAWN_OBJECT_SERVICE);
    auto spawnSceneFileServer = node.advertiseService(SPAWN_SCENE_FILE_SERVICE, spawnSceneFileService);
    PRINT_INFO("you can start spawning scene files now");
    for (int i = 1; i < argc; i++)
    {
        SpawnSceneFile service;
        service.request.filename = std::string(argv[i]);
        service.request.frame_id = paramNode.param<std::string>("reference_frame_id", "");
        spawnSceneFileService(service.request, service.response);
    }
    ros::spin();
}
