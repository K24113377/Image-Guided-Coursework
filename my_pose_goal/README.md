# my_pose_goal – ROS2 Subscriber for Incoming Pose Transforms

This package contains a ROS2 node that subscribes to a transform topic (e.g. `/IGTL_TRANSFORM`) and parses incoming 4×4 pose matrices into ROS2-friendly formats. It acts as the interface between external sources (e.g. 3D Slicer via OpenIGTLink) and the robot's motion planning pipeline.

---

## Features
- Subscribes to `/IGTL_TRANSFORM` published by `ros2_igtl_bridge`
- Parses the translation and rotation from the 4×4 matrix
- Converts the pose to a ROS2 `PoseStamped` message
- Makes the pose available for downstream execution by robot control nodes

---

## Folder Structure

- `my_pose_goal/member_subscriber.py` – Main subscriber node
- `my_pose_goal/__init__.py` – Package init file
- `resource/` – Placeholder for ROS2 resource registration (if needed)
- `package.xml` – ROS2 package metadata
- `setup.py` – Entry point definition for Python launch
- `setup.cfg` – Python build configuration

---

## Usage Instructions

1. Make sure this package is in your ROS2 workspace (e.g., `ros_ws/src/`).
2. Build your workspace:
   ```bash
   colcon build
   source install/setup.bash
3. Launch your ROS2 core and robot system (e.g., my_moveit_robot).
4. Run the subscriber node:
   ros2 run my_pose_goal member_subscriber
5. Use ros2 topic echo /IGTL_TRANSFORM in another terminal to confirm the node is listening to incoming      transforms.


## Usage Notes 
This node is typically used in conjunction with 3D Slicer + OpenIGTLink to receive trajectory targets.

It decodes the transform and prepares the pose data for execution, but does not execute movement itself (see my_robot_goal for motion planning).

Pose validation and logging can be added if real-time feedback is required.

   
