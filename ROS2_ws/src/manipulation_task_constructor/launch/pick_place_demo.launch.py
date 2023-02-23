from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("denso_cobotta").to_dict()

    # MTC Demo node
    pick_place_demo = Node(
        package="manipulation_task_constructor",
        executable="manipulation_task_constructor",
        output="screen",
        parameters=[
            moveit_config,
        ],
    )

    return LaunchDescription([pick_place_demo])