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
    moveit_config = MoveItConfigsBuilder("denso_vp6242").to_dict()

    # MTC Demo node
    pick_place_demo = Node(
        package="environment_builder",
        executable="assembly_plan_visualizer",
        output="screen",
        parameters=[
            {"csv_file_path": test},
        ],
    )

    nodes_to_start = [
        pick_place_demo,
    ]

    return nodes_to_start