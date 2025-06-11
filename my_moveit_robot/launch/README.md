# my_moveit_robot – MoveIt2 Configuration and Execution Package

This package configures and launches a 6-DOF robot in MoveIt2 for motion planning and trajectory execution. It includes all required config and launch files for integrating the robot model (`my_jointed_robot`) with the MoveIt2 framework in ROS2.

---

## Features
- Full MoveIt2 setup for a 6-DOF arm using ROS2 Jazzy
- Launches RViz with Move Group interface for planning and execution
- Uses robot description from the `my_jointed_robot` package
- Includes SRDF, controllers, joint limits, kinematics, and Cartesian constraints

---

## Folder Structure

### `config/`
- `myArm6DOF.srdf` – Semantic robot description (planning groups, end effectors)
- `controllers.yaml` – Controller definitions
- `joint_limits.yaml` – Joint range and velocity limits
- `kinematics.yaml` – IK solver configuration
- `initial_positions.yaml` – Home position of the robot
- `moveit.rviz` – Preconfigured RViz setup
- `moveit_controllers.yaml`, `ros2_controllers.yaml` – Motion controller parameters
- `myArm6DOF.urdf.xacro` – Linked robot model file (URDF/Xacro)

### `launch/`
- `demo.launch.py` – Starts robot and MoveIt2 in demo mode with RViz
- `move_group.launch.py` – Core Move Group server
- `moveit_rviz.launch.py` – Starts RViz with MoveIt2 config
- `rsp.launch.py` – Loads robot state publisher
- `spawn_controllers.launch.py` – Spawns hardware interfaces/controllers
- `setup_assistant.launch.py` – Launches MoveIt Setup Assistant
- `static_virtual_joint_tfs.launch.py` – Static TFs for virtual joints

---

## Usage Instructions

1. **Build the workspace**:
   ```bash
   colcon build
   source install/setup.bash
   
2. **Launch the full planning environment (RViz + MoveIt2)**:
ros2 launch my_moveit_robot demo.launch.py

3. **Plan and execute trajectories** in RViz using the MoveIt2 Motion Planning panel.

## Usage Instructions
This package assumes the robot description is provided by my_jointed_robot.

Make sure the planning group in myArm6DOF.srdf matches the joint names in your URDF.

You can modify controller or IK settings in the config/ directory to suit specific applications.

Used in conjunction with my_pose_goal and my_robot_goal nodes to execute poses or trajectories received externally (e.g. from 3D Slicer).
