#include <ros/ros.h>
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

using namespace std_msgs;
using namespace shape_msgs;
using namespace geometry_msgs;
using namespace moveit::planning_interface;
using namespace block_printer_visualizer;

static const std::string nodeName = "object_spawner";

#define PRINT_INFO(args) ROS_INFO_STREAM(nodeName << ": " << args)
#define PRINT_ERROR(args) ROS_ERROR_STREAM(nodeName << ": " << args)

static bool spawnPrimitiveObject(const SolidPrimitive &primitive,
                                 const ColorRGBA &color,
                                 const PoseStamped &poseStamped,
                                 const std::string &objectId,
                                 PlanningSceneInterface &planningScene)
{
    moveit_msgs::CollisionObject collisionObject;
    collisionObject.header = poseStamped.header;
    collisionObject.id = objectId;
    collisionObject.primitives.push_back(primitive);
    collisionObject.primitive_poses.push_back(poseStamped.pose);
    collisionObject.operation = moveit_msgs::CollisionObject::ADD;
    return planningScene.applyCollisionObject(collisionObject, color);
}

static bool spawnPlane(const Plane &plane,
                       const ColorRGBA &color,
                       const PoseStamped &poseStamped,
                       const std::string &objectId,
                       PlanningSceneInterface &planningScene)
{
    moveit_msgs::CollisionObject collisionObject;
    collisionObject.header = poseStamped.header;
    collisionObject.id = objectId;
    collisionObject.planes.push_back(plane);
    collisionObject.plane_poses.push_back(poseStamped.pose);
    collisionObject.operation = moveit_msgs::CollisionObject::ADD;
    return planningScene.applyCollisionObject(collisionObject, color);
}

static bool spawnMesh(const std::string &filename,
                      const ColorRGBA &color,
                      const PoseStamped &poseStamped,
                      const std::string &objectId,
                      PlanningSceneInterface &planningScene)
{
    moveit_msgs::CollisionObject co;
    co.id = objectId;
    auto m = std::shared_ptr<shapes::Mesh>(shapes::createMeshFromResource(filename));
    if (!m)
    {
        PRINT_ERROR("could not load mesh file: " << filename);
        return false;
    }
    shapes::ShapeMsg mesh_msg;
    shapes::constructMsgFromShape(m.get(), mesh_msg);
    shape_msgs::Mesh mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

    co.meshes.resize(1);
    co.mesh_poses.resize(1);
    co.meshes[0] = mesh;
    co.header = poseStamped.header;
    co.mesh_poses[0] = poseStamped.pose;

    co.meshes.push_back(mesh);
    co.mesh_poses.push_back(co.mesh_poses[0]); //謎の呪文
    co.operation = moveit_msgs::CollisionObject::ADD;
    return planningScene.applyCollisionObject(co, color);
}

static bool spawnObject(const Object &object,
                        PlanningSceneInterface &planningScene)
{
    auto spawnResult = false;
    switch (object.type)
    {
    case Object::SHAPE_PRIMITIVE:
        spawnResult = spawnPrimitiveObject(object.primitive, object.color, object.poseStamped, object.name, planningScene);
        break;
    case Object::SHAPE_PLANE:
        spawnResult = spawnPlane(object.plane, object.color, object.poseStamped, object.name, planningScene);
        break;
    case Object::SHAPE_MESH:
        spawnResult = spawnMesh(object.mesh_filename, object.color, object.poseStamped, object.name, planningScene);
        break;
    default:
        PRINT_ERROR("unexpected object type: " << object.type);
        spawnResult = false;
        break;
    }
    if (spawnResult)
    {
        PRINT_INFO("successfully spawned an object: " << object.name);
    }
    return spawnResult;
}

static bool spawnObjectService(SpawnObjectRequest &request, SpawnObjectResponse &response, PlanningSceneInterface &planningScene)
{
    return spawnObject(request.object, planningScene);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, nodeName);
    ros::NodeHandle node;
    PlanningSceneInterface planningScene;
    auto spawnObjectServer = node.advertiseService<SpawnObjectRequest, SpawnObjectResponse>(SPAWN_OBJECT_SERVICE, boost::bind(spawnObjectService, _1, _2, boost::ref(planningScene)));
    PRINT_INFO("you can start spawning objects now");
    ros::spin();
    return 0;
}
