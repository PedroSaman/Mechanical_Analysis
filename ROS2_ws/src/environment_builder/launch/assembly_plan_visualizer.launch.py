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
    csv_file_path = LaunchConfiguration("csv_file_path")

    # Visualization Node
    assembly_plan_visualizer = Node(
        package="environment_builder",
        executable="assembly_plan_visualizer",
        output="screen",
        parameters=[
            {"csv_file_path": csv_file_path},
        ],
    )

    nodes_to_start = [
        assembly_plan_visualizer,
    ]

    return nodes_to_start