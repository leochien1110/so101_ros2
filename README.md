# SO-101 ROS 2 Packages

This repository contains the ROS 2 packages for controlling the SO-101 robot arm with Feetech servos, MoveIt 2, and `ros2_control`.

This repository is not a ROS 2 workspace root. Clone it into the `src/` directory of your own ROS 2 workspace, then build from the workspace root.

## Packages

- `so101_description`: URDF description, meshes, and visualization assets for the SO-101 robot.
- `so101_hardware`: C++ `ros2_control` hardware interface for Feetech servos with LeRobot calibration support.
- `so101_moveit`: MoveIt 2 configuration, controllers, and launch files.

## Prerequisites

- A working ROS 2 installation. ROS 2 Humble is the main target for this repository.
- `colcon` and `rosdep`
- `git`

If this is your first ROS 2 workspace on the machine, initialize `rosdep` once:

```bash
sudo rosdep init
rosdep update
```

## Workspace Setup

Create a workspace, clone this repository into `src/`, then build from the workspace root:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/leochien1110/so101_ws.git

cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

If you prefer to build only these packages while iterating locally:

```bash
colcon build --packages-up-to so101_moveit --symlink-install
```

## Launch

From the workspace root or any shell where `install/setup.bash` has been sourced:

```bash
ros2 launch so101_moveit demo.launch.py \
  use_fake_hardware:=false \
  port:=/dev/ttyACM1
```

Ensure your user has permission to access the robot serial device.

## Key Configuration Files

### Inverse Kinematics

- Solver: `lma_kinematics_plugin/LMAKinematicsPlugin`
- Config: `so101_moveit/config/kinematics.yaml`
- Note: the URDF includes a dummy `world` link to avoid root-link inertia issues in the solver
- Approximate IK is enabled for easier interaction with the marker on this 5-DOF arm, which can cause unexpected wrist rotation for some target poses

### Calibration

- Default calibration file: `so101_moveit/config/calibration_default.json`
- Override it with your own LeRobot follower calibration file, for example `~/.cache/huggingface/lerobot/calibration/robots/so101_follower/my_arm.json`
- The driver maps raw motor values to URDF joint limits using each joint's `range_min` and `range_max`

## Platform Notes

### macOS and RoboStack

ROS 2 is not officially supported on macOS, so the practical path there is RoboStack with micromamba.

```bash
"${SHELL}" <(curl -L micro.mamba.pm/install.sh)

micromamba create -n ros_humble_env \
  -c robostack-humble -c conda-forge \
  --override-channels --strict-channel-priority \
  ros-humble-desktop ros-humble-moveit-setup-assistant rosdep colcon-common-extensions -y

micromamba activate ros_humble_env
rosdep update
```

On macOS, `warehouse_ros_sqlite` is used instead of `warehouse_ros_mongo`.

## TODO

### Servo Motion Smoothing

The SO-101 uses Feetech STS3215 servos, which can move abruptly compared with higher-end actuators. The main follow-up areas are:

- Lower `ACC` from `50` toward `10-20` for softer acceleration and deceleration
- Reduce speed from `2400` toward `800-1000`
- Expose `ACC` and speed as ROS parameters instead of hardcoded values
- Try `GOAL_TIME` mode so joints arrive together
- Add host-side interpolation such as cubic spline or S-curve waypoints

Useful SDK registers:

| Parameter | Register | Function |
|-----------|----------|----------|
| ACC | 0x29 (41) | Acceleration value for soft start and stop |
| GOAL_POSITION | 0x2A-0x2B | Target position |
| GOAL_TIME | 0x2C-0x2D | Target time to reach position |
| GOAL_SPEED | 0x2E-0x2F | Maximum movement speed |

Known limitations:

- Plastic gears introduce backlash near direction changes
- 12-bit encoders limit resolution to about 0.088 degrees
- The servo firmware uses trapezoidal profiles, not true S-curves
- Very low `ACC` values can increase heat during continuous motion

## License

This project follows the LeRobot license, Apache 2.0.

## Acknowledgments

- LeRobot
- LeRobot-ROS PR #8
- MoveIt 2
- LycheeAI Hub's SO-ARM101 Isaac Sim article
- SO-ARM101_MoveIt_IsaacSim
- SCServo_Linux
