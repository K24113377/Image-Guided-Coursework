# Image-Guided Robotic Navigation System

This repository contains the full pipeline for an image-guided robotic navigation system designed for minimally invasive surgical planning and execution. The system integrates medical image analysis, real-time data communication, and robotic motion planning to compute and execute safe linear trajectories from an entry point to a surgical target while avoiding critical anatomical structures.

---

## Project Overview

The system is divided into three core modules:

1. **3D Slicer Path Planning**  
   Calculates valid straight-line trajectories using anatomical imaging and spatial constraints.

2. **OpenIGTLink Interface**  
   Transmits trajectory data in real time from 3D Slicer to ROS2 using OpenIGTLink and `ros2_igtl_bridge`.

3. **ROS2 MoveIt2 Robot Control**  
   Receives the trajectory, computes inverse kinematics, and executes motion to the target pose.

---

## Getting the Full System Working (Step-by-Step Guide)

> This section includes both the ideal OpenIGTLink connection workflow and a workaround using manual transform publishing.

---

## Ideal Workflow with Working OpenIGTLink Connection

If the OpenIGTLink communication between 3D Slicer and ROS2 functions correctly, follow these steps:

### In 3D Slicer:
1. Load the `FinaPathPlanner` module via Extension Wizard.
2. Load anatomical data and convert all volumes to label maps except `fakeBrain`.
3. Place entry and target fiducials.
4. Run the planner to generate the best trajectory.
5. A `vtkMRMLLinearTransformNode` will be created with the pose.
6. Open the **OpenIGTLinkIF** module:
   - Add a connector node in **Server mode** (port `18944`)
   - Click **Start**
   - Add the transform node as an **Outgoing** node and enable **Push on Connect**

### In ROS2:
Open three terminals.

**Terminal 1 – Start OpenIGTLink Bridge:**
```bash
source install/setup.bash
ros2 run ros2_igtl_bridge igtl_bridge_node
```


**Terminal 2 – Launch the robot planner:**

```bash
source install/setup.bash
ros2 launch my_robot_goal robot_plan.launch.py
```

**Terminal 3 – Run the subscriber node:**

```bash
source install/setup.bash
ros2 run my_pose_goal member_subscriber
```

Once the transform is pushed from Slicer, it will be received and executed by the robot via MoveIt2.

## Workaround (Manual Transform Publishing)

If 3D Slicer fails to stream the transform via OpenIGTLink (due to connection or compatibility issues), you can manually publish the planned pose to ROS2 using the following steps:

### In 3D Slicer

1. Use the `FinaPathPlanner` module to compute the best trajectory.
2. Note the target coordinates (e.g., `[154.0, 74.0, 133.0]`).
3. Convert the coordinates from millimetres to metres by dividing by 1000:
   - Result: `[0.154, 0.074, 0.133]`

### In ROS2

Use four terminals:

**Terminal 1 – Launch the robot executor**
```bash
source install/setup.bash
ros2 launch my_robot_goal robot_plan.launch.py
```

**Terminal 2 – Run the subscriber**
```bash
source install/setup.bash
ros2 run my_pose_goal member_subscriber
```

**Terminal 3 – Manually publish the transform**
This transform corresponds to the best trajectory selected in 3D Slicer when the hippocampus was set as the target and vessels as the critical structure, with the maximum trajectory length set to 100 mm.

```bash
ros2 topic pub /IGTL_TRANSFORM_IN ros2_igtl_bridge/msg/Transform "{
  name: 'best_trajectory',
  transform: {
    translation: { x: 0.154, y: 0.074, z: 0.133 },
    rotation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }
  }
}"
```
If you select a different entry–target pair in the Slicer path planner and obtain a new trajectory, convert the RAS coordinates (in millimetres) from Slicer into ROS2 coordinates (in metres) by dividing each value by 1000 before publishing.

Note on Y-axis constraint:  
Due to a workspace limitation in the current robot setup, the **Y translation value must be at least `0.2`**. If the path planner returns a Y-coordinate below this (e.g., `0.113`), the transform will not execute correctly in MoveIt2. As a result, even when using valid planned trajectories, you must **manually set the Y value to `0.2`** when publishing the transform.

And here’s the updated command example including the context:

```bash
ros2 topic pub /IGTL_TRANSFORM_IN ros2_igtl_bridge/msg/Transform "{
  name: 'best_trajectory',
  transform: {
    translation: { x: 0.154, y: 0.2, z: 0.133 },
    rotation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }
  }
}"
```

**Terminal 4 (Optional) – Monitor the topic**
```bash
ros2 topic echo /IGTL_TRANSFORM
```
This method bypasses OpenIGTLink and allows you to simulate trajectory delivery manually. The robot will then move to the provided pose using MoveIt2.

If all terminals are configured correctly, the robot should now visibly move to the specified target pose in RViz, confirming successful execution of the planned trajectory.



