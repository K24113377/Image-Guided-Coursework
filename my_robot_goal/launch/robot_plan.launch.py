from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            "myArm6DOF", package_name="my_moveit_robot")
            .trajectory_execution(file_path="config/moveit_controllers.yaml"
        )
        .moveit_cpp(
            file_path=get_package_share_directory("my_robot_goal")
            + "/config/motion_planning.yaml"
        )
        .to_moveit_configs()
    )
    
    moveit_py_node = Node(
        name="moveit_py",
        package="my_robot_goal",
        executable="my_robot_goal",
        output="both",
        parameters=[moveit_config.to_dict()],
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="log",
        parameters=[moveit_config.robot_description],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[get_package_share_directory("my_moveit_robot")+"/config/ros2_controllers.yaml"],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="log",
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", get_package_share_directory("my_moveit_robot")
            +"/config/moveit.rviz"],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
    )



    load_controllers = []
    for controller in [
        "joint_state_broadcaster",
        "my_planning_group_controller",
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {}".format(controller)],
                shell=True,
                output="log",
            )
        ]

    return LaunchDescription(
        [
            moveit_py_node,
            robot_state_publisher,
            ros2_control_node,
            static_tf,
            rviz_node,
        ]
        + load_controllers
    )
    




    


