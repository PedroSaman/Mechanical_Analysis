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
            description="Provides the assembly plan csv file path and name.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_name",
            default_value="denso_vp6242",
            description="Determine the robot name for the MTC correct functioning.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "first_block",
            default_value="1",
            description="From what block should the planner start. Uses 'normal' notation, the first block is 1.",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )

def launch_setup(context, *args, **kwargs):
    csv_file_path = LaunchConfiguration("csv_file_path")
    robot_name = context.perform_substitution(LaunchConfiguration("robot_name"))
    moveit_config = MoveItConfigsBuilder(robot_name).to_dict()
    first_block = LaunchConfiguration("first_block")

    # MTC Demo node
    action_client_node = Node(
        package="mtc_tutorial",
        executable="task_creator_action_client",
        output="screen",
        parameters=[
            moveit_config,
            {"csv_file_path": csv_file_path},
            {"robot_name": robot_name},
            {"first_block": first_block},
        ],
    )

    # Bag node
    bag_node = Node(
        package="bag_recorder_nodes",
        executable="simple_bag_recorder",
        output="screen",
    )

    nodes_to_start = [
        action_client_node,
        bag_node,
    ]

    return nodes_to_start