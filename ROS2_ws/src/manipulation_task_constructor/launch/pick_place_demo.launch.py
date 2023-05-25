from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "csv_file_path",
            default_value="no_csv_provided",
            description="Provides the assembly plan csv file path and name",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )

def launch_setup(context, *args, **kwargs):
    test = LaunchConfiguration("csv_file_path")
    moveit_config = MoveItConfigsBuilder("denso_cobotta").to_dict()

    # bag node
    bag_node = Node(
        package="manipulation_task_constructor",
        executable="manipulation_task_constructor",
        output="screen",
        parameters=[
            moveit_config,
            {"csv_file_path": test},
        ],
    )

    # MTC Demo node
    pick_place_demo = Node(
        package="bag_recorder_nodes",
        executable="simple_bag_recorder",
        output="screen",
    )

    nodes_to_start = [
        pick_place_demo,
        bag_node,
    ]

    return nodes_to_start