# FinaPathPlanner – 3D Slicer Path Planning Module

This module computes safe straight-line surgical trajectories between user-defined entry and target points using anatomical constraints derived from label maps.

---

## Features
- Qt-based GUI built with `.ui` layout for ease of use
- Loads anatomical label maps (e.g. ventricles, hippocampus)
- Allows manual selection of entry and target fiducials
- Evaluates all entry–target combinations using:
  - Length constraints
  - Target region inclusion
  - Collision detection with critical structures
  - Distance map scoring
- Visualises the highest-scoring trajectory in the 3D view

---

## User Interface
The interface is defined in `FinalPathPlanner.ui` and provides the following controls:

- **Target Label Map**: Dropdown to select the region the trajectory must terminate in
- **Critical Structure Label Map**: Dropdown to select regions to avoid (e.g. ventricles)
- **Entry Points**: Fiducial list for all possible entry locations
- **Target Points**: Fiducial list for all possible target locations
- **Length Threshold (mm)**: Numeric input for maximum trajectory length
- **Apply**: Computes all valid trajectories based on the selected inputs and displays the best one in red
- **Run All Tests**: Executes built-in unit tests for validation (positive, negative, and edge cases). Output is shown in the Python console.

---

## Usage Instructions

### Module Setup in Slicer
1. Open **3D Slicer**.
2. Go to **Developer Tools → Extension Wizard**.
3. Click **"Select Extension"** and choose the `FinaPathPlanner` folder.
4. Build and load the module into Slicer.

### Loading Data
1. Load your **brain parcellation dataset** into the scene.
2. Set **all volumes** to type `LabelMap` **except** for `fakeBrain`, which should remain a scalar volume.
   - Right-click each volume in the Data module and choose "Convert to LabelMap" if needed.

### Running the Planner
1. Switch to the **FinaPathPlanner** module.
2. In the GUI:
   - Select the **target label map** (e.g. hippocampus)
   - Select the **critical structure** (e.g. ventricles)
   - Place **entry and target fiducials** in the Markups module
   - Set the **maximum trajectory length**
3. Click **Apply** to compute and visualise the safest valid trajectory.
4. Click **Run All Tests** to validate the module functionality. Results appear in the Python console.

---

## Validation
The planner applies four core filters:
1. **Length check** – using `vtkMath.Distance2BetweenPoints`
2. **Target inclusion** – verified via `vtkMatrix4x4` and voxel lookup
3. **Collision detection** – using `vtkImageData.GetScalarComponentAsDouble`
4. **Trajectory scoring** – based on `vtkImageEuclideanDistance` maps

---

## Folder Contents
- `FinalPathPlanner/` – Logic scripts and helper classes
- `FinalPathPlanner.ui` – Qt-based UI layout
- `CMakeLists.txt` – Module build config for 3D Slicer
- `FinaPathPlanner.png` – Module icon

---

## Notes
- Built using the `parameterNodeWrapper` framework.
- Designed for **3D Slicer v5.8.1**.
- Part of an end-to-end image-guided robotic navigation pipeline (see root README).

