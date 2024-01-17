from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_spawn_controllers_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("epson_t3", package_name="epson_t3_moveit_config").to_moveit_configs()
    return generate_spawn_controllers_launch(moveit_config)