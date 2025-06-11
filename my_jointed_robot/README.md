# my_jointed_robot – URDF Robot Description Package

This package defines the 6-DOF robot model used for trajectory execution in ROS2. It includes the full URDF specification and package setup files for integration with MoveIt2 and RViz.

## Features
- Defines a multi-jointed robot arm in URDF format
- Includes visual and collision geometry
- Designed for use with MoveIt2 motion planning
- Structured as a standard ROS2 package with CMake and Python setup files

## Folder Structure
- `urdf/` – Contains the URDF file describing the robot’s links and joints  
- `CMakeLists.txt` – ROS2 build configuration  
- `package.xml` – ROS2 package metadata  
- `setup.py`, `setup.cfg` – Optional Python configuration for CLI tools or node registration

## Usage Instructions

1. Place this package inside your ROS2 workspace (e.g., `ros_ws/src/`).
2. Build the workspace:
   ```bash
   colcon build
   source install/setup.bash
3. Launch your robot in RViz using a compatible MoveIt2 launch file (e.g., from the my_moveit_robot package).
4. Ensure the planning group defined in your SRDF includes the correct joints (e.g., joint_13, joint_11, ..., joint_1).
5. Confirm that the robot appears and responds to planned motions in RViz.

## Notes
This robot model is referenced by other ROS2 packages in the project, including motion control and transform subscriber nodes.

The URDF is designed for compatibility with MoveIt2’s inverse kinematics and motion planning tools.

If you modify the robot’s joints or structure, ensure all connected packages (e.g., SRDF, motion planners, TF broadcasters) are updated accordingly.

