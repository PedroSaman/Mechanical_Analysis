shape_msgs/SolidPrimitive primitive
shape_msgs/Plane plane
#filename of mesh model
string mesh_filename
std_msgs/ColorRGBA color
geometry_msgs/PoseStamped poseStamped
#id of spawned object
string name
#object type
int8 type
int8 SHAPE_PRIMITIVE=1
int8 SHAPE_PLANE=2
int8 SHAPE_MESH=3
