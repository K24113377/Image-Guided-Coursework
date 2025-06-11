# my_robot_goal – ROS2 Node for Trajectory Execution with MoveIt2

This package handles robotic execution by moving the robot to a specified pose received via ROS2 messaging. It integrates with MoveIt2 to compute motion plans and execute them using the robot model defined in `my_jointed_robot`.

---

## Features
- Subscribes to a `PoseStamped` message (typically parsed from an external transform)
- Executes the pose using MoveIt2 planning and control
- Includes optional launch file for simulating transform publishing

---

## Folder Structure

- `my_robot_goal/my_robot_goal.py` – Main script that executes incoming poses
- `launch/`
  - `robot_plan.launch.py` – Launches the execution node with MoveIt2
  - `trajectory_publisher_launch.py` – Simulated trajectory publisher (intended for testing in place of 3D Slicer)
- `config/motion_planning.yaml` – MoveIt2 motion planner parameters
- `resource/`, `test/` – Standard ROS2 package structure
- `package.xml`, `setup.py`, `setup.cfg` – ROS2 package and entry point metadata

---

## Usage Instructions

1. Build the package:

    ```bash
    colcon build
    source install/setup.bash
    ```

2. Launch the robot executor node:

    ```bash
    ros2 launch my_robot_goal robot_plan.launch.py
    ```

3. To simulate a transform without Slicer, you can (optionally) run:

    ```bash
    ros2 launch my_robot_goal trajectory_publisher_launch.py
    ```

    > **Note:** This publisher was designed to mimic 3D Slicer by publishing the transform to `/IGTL_TRANSFORM_IN`, but it was not fully integrated due to time constraints.

4. The following manual `ros2 topic pub` command was used instead:

    ```bash
    ros2 topic pub /IGTL_TRANSFORM_IN ros2_igtl_bridge/msg/Transform "{
      name: 'best_trajectory',
      transform: {
        translation: { x: 0.154, y: 0.2, z: 0.133 },
        rotation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }
      }
    }"
    ```

    This command successfully delivered the transform directly to ROS2, allowing the robot to move to the simulated pose.


## Notes
This node depends on the output of the my_pose_goal subscriber (which converts /IGTL_TRANSFORM into a PoseStamped).

The MoveIt2 planning group and joint limits must match the robot's configuration in my_moveit_robot.

Execution is visualized in RViz, and no real hardware is required for testing.

## Development Status

Pose reception and MoveIt2 execution are working and validated in simulation.

The trajectory_publisher_launch.py node for mimicking Slicer input was drafted but not fully completed due to time constraints.


