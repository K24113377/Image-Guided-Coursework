from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch

# This file will start rviz and move_group files

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("myArm6DOF", package_name="my_moveit_robot")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )
    
    return generate_demo_launch(moveit_config)

