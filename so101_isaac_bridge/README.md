# so101_isaac_bridge

Bridge package that lets MoveIt 2 execute SO-101 trajectories in Isaac Sim by translating controller actions into `sensor_msgs/JointState` commands.

## What this package provides

- Node: `so101_isaac_joint_command_bridge`
- Launch file: `so101_isaac_moveit.launch.py`
- Action bridge endpoints:
  - `arm_controller/follow_joint_trajectory`
  - `gripper_controller/gripper_cmd`
- Topic bridge:
  - Subscribes: `joint_state_topic` (default `/joint_states`)
  - Publishes: `joint_command_topic` (default `/joint_command`)

## Build and source

From workspace root:

```bash
colcon build --symlink-install --packages-up-to so101_isaac_bridge
source install/setup.bash
```

## Usage

Start Isaac Sim with the SO-101 model and ensure its ROS graph publishes `/joint_states` and subscribes `/joint_command`, then run:

```bash
ros2 launch so101_isaac_bridge so101_isaac_moveit.launch.py
```

This starts:

- `robot_state_publisher`
- `move_group`
- `rviz2` (if enabled)
- `so101_isaac_joint_command_bridge`

## Launch arguments

- `use_rviz` (default: `true`)
- `use_sim_time` (default: `false`)
- `start_state_max_bounds_error` (default: `0.001`)
- `joint_state_topic` (default: `/joint_states`)
- `joint_command_topic` (default: `/joint_command`)
- `publish_full_joint_command` (default: `true`)

## Quick diagnostics

```bash
ros2 topic echo --once /joint_states
ros2 topic info /joint_command
ros2 node list | grep so101_isaac_joint_command_bridge
ros2 action list | grep -E "arm_controller|gripper_controller"
```