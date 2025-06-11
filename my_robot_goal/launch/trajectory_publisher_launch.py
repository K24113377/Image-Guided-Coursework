from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
import json

def interpolate_linear_steps(start, end, steps):
    return [
        {
            'x': round(start[0] + (end[0] - start[0]) * i / steps, 3),
            'y': round(start[1] + (end[1] - start[1]) * i / steps, 3),
            'z': round(start[2] + (end[2] - start[2]) * i / steps, 3),
        }
        for i in range(steps + 1)
    ]

def generate_launch_description():
    steps = 20
    entry = [196.439, 32.284, 126.646]
    target = [154.0, 74.0, 133.0]
    poses = interpolate_linear_steps(entry, target, steps)

    pub_cmds = []
    for i, p in enumerate(poses):
        msg = {
            "name": "pose_goal",
            "transform": {
                "translation": {"x": p["x"], "y": p["y"], "z": p["z"]},
                "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
            }
        }
        pub_cmds.append(
            TimerAction(
                period=0.2 * i,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            'ros2', 'topic', 'pub', '-1', '/IGTL_TRANSFORM_IN',
                            'ros2_igtl_bridge/msg/Transform',
                            json.dumps(msg)
                        ],
                        shell=False
                    )
                ]
            )
        )

    return LaunchDescription(pub_cmds)
