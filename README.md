# Image-Guided Robotic Navigation System

This repository contains the full pipeline for an image-guided robotic navigation system designed for minimally invasive surgical planning and execution. The system integrates medical image analysis, real-time data communication, and robotic motion planning to compute and execute safe linear trajectories from an entry point to a surgical target while avoiding critical anatomical structures.

## Project Overview

The system is divided into three core modules:

1. **3D Slicer Path Planning**  
   Calculates valid straight-line trajectories using anatomical imaging and spatial constraints.

2. **OpenIGTLink Interface**  
   Transmits trajectory data in real time from 3D Slicer to ROS2 using OpenIGTLink and `ros2_igtl_bridge`.

3. **ROS2 MoveIt2 Robot Control**  
   Receives the trajectory, computes inverse kinematics, and executes motion to the target pose.

## Module Descriptions

### 1. 3D Slicer Path Planning

This module allows users to:
- Load anatomical volumes and segmentations (e.g., target, ventricles)
- Define entry and target fiducials
- Compute all valid entry–target straight-line trajectories
- Reject paths intersecting critical structures using voxel-based label maps
- Score valid paths using Euclidean distance maps
- Select and visualise the safest, highest-scoring trajectory

### 2. OpenIGTLink Communication

After computing the trajectory, a `vtkMRMLLinearTransformNode` is created to encode the pose as a 4×4 matrix. This is transmitted from Slicer to ROS2 via a `vtkMRMLIGTLConnectorNode` in server mode, using port 18944. The OpenIGTLinkIF module pushes the transform continuously. On the ROS2 side, the `ros2_igtl_bridge` package receives the transform and republishes it as a ROS2 message on `/IGTL_TRANSFORM`.

### 3. ROS2 MoveIt2 Integration

The ROS2 node subscribes to the `/IGTL_TRANSFORM` topic and parses incoming transform data. Using MoveIt2, it:
- Converts the transform into a pose relative to the robot base
- Performs inverse kinematics
- Plans and executes motion to the target using a 6-DOF robotic arm

## Usage Instructions

### Slicer Path Planning
1. Open 3D Slicer and load the module.
2. Import:
   - A label map volume for the target structure
   - Optional critical structures (e.g., ventricles)
   - Entry and target fiducials
3. Run the path planner to compute and visualise the best trajectory.

### OpenIGTLink Setup
1. In Slicer:
   - Add a `vtkMRMLIGTLConnectorNode`
   - Link it to the trajectory transform node
   - Set port to 18944 and enable push on connect
2. In ROS2:
   - Launch the `ros2_igtl_bridge`
   - Echo the topic `/IGTL_TRANSFORM` to verify reception

### ROS2 MoveIt2 Execution
1. Build the ROS2 workspace with `colcon build`
2. Source the workspace and run the robot subscriber node
3. Verify planned execution in RViz or on a physical robot (if connected)

## Known Issue and Workaround

While an OpenIGTLink connection was successfully established between 3D Slicer and ROS2 (with the connector status showing as "Connected"), the transform messages were not received by ROS2 as expected. As a workaround, the final pose was manually published using the `ros2 topic pub` command on the `/IGTL_TRANSFORM_IN` topic. This simulated the transform that would have been sent by Slicer, allowing successful trajectory execution via MoveIt2. Future debugging may focus on message formatting or OpenIGTLink bridge compatibility.

---


## Dependencies

- 3D Slicer (v5.8.1)
- OpenIGTLinkIF extension
- ROS2 Jazzy
- MoveIt2
- Python 3.9 (via Slicer and ROS environments)

## Author

Safa Khan  
MSc Healthcare Technologies  
King’s College London

## License

This project is licensed under the MIT License.


