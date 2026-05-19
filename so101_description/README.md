# SO-101 Description Package

URDF, meshes, and visualization assets for the SO-101 robot arm.

## Overview

This package provides:

- URDF robot description
- Visual and collision meshes
- RViz-friendly assets used by the rest of the stack
- Resources consumed by the MoveIt configuration

## Source

Original model source:
https://github.com/TheRobotStudio/SO-ARM100/tree/main/Simulation/SO101

## Build Instructions

This package lives inside the multi-package `so101_ws` repository. Clone the repository into the `src/` directory of your ROS 2 workspace, then build from the workspace root:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/leochien1110/so101_ws.git

cd ~/ros2_ws
colcon build --packages-select so101_description
source install/setup.bash
```
