from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    return LaunchDescription([
        # First: spawn joint_state_broadcaster immediately
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen'
        ),

        # Second: spawn planning controller AFTER a delay
        TimerAction(
            period=2.0,  # Wait 2 seconds
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['my_planning_group_controller'],
                    output='screen'
                )
            ]
        )
    ])

