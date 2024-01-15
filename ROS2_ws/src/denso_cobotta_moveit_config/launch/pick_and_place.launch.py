import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    
    # planning_context
    moveit_config = (
        MoveItConfigsBuilder("denso_cobotta")
        .robot_description(file_path="config/denso_cobotta.urdf.xacro")
        .robot_description_semantic(file_path="config/denso_cobotta.srdf")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl"]
        )
        .to_moveit_configs()
    )

    # Load  ExecuteTaskSolutionCapability so we can execute found solutions in simulation
    move_group_capabilities = {
        "capabilities": "move_group/ExecuteTaskSolutionCapability"
    }

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(),
                    move_group_capabilities,
                   ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # RViz
    rviz_config_file = (
        get_package_share_directory("denso_cobotta_moveit_config") + "/launch/moveit.rviz"
    )
    rviz_node_moveit = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            moveit_config.robot_description,
        ],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("denso_cobotta_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.to_dict(), ros2_controllers_path],
        output="both",
    )

    block_services = Node(
        package="environment_builder",
        executable="block_services",
    )

    world_builder = Node(
        package="environment_builder",
        executable="world_builder",
    )

    # Load controllers
    load_controllers = []
    for controller in [
        "denso_cobotta_arm_controller",
        "denso_cobotta_hand_controller",
        "joint_state_broadcaster",
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {}".format(controller)],
                shell=True,
                output="screen",
            )
        ]

    return LaunchDescription(
        [
            rviz_node_moveit,
            static_tf,
            robot_state_publisher,
            run_move_group_node,
            ros2_control_node,
            block_services,
            world_builder,
        ]
        + load_controllers
    )
