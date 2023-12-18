from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    return LaunchDescription(
        [OpaqueFunction(function=launch_setup)]
    )

def launch_setup(context, *args, **kwargs):
    robot_name = "denso_vp6242"
    moveit_config = MoveItConfigsBuilder(robot_name).to_dict()

    # MTC Demo node
    pick_place_demo = Node(
        package="mtc_tutorial",
        executable="task_creator_action_server",
        output="screen",
        parameters=[
            moveit_config,
        ],
    )

    nodes_to_start = [
        pick_place_demo,
    ]

    return nodes_to_start