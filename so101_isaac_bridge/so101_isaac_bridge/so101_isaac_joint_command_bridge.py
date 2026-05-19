import threading
import time
from typing import Dict, List, Optional

import rclpy
from control_msgs.action import FollowJointTrajectory, GripperCommand
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class SO101IsaacJointCommandBridge(Node):
    def __init__(self) -> None:
        super().__init__('so101_isaac_joint_command_bridge')

        self.declare_parameter('joint_state_topic', '/joint_states')
        self.declare_parameter('joint_command_topic', '/joint_command')
        self.declare_parameter(
            'controlled_joints',
            [
                'shoulder_pan',
                'shoulder_lift',
                'elbow_flex',
                'wrist_flex',
                'wrist_roll',
                'gripper',
            ],
        )
        self.declare_parameter('publish_full_joint_command', True)

        self._joint_state_topic = self.get_parameter('joint_state_topic').value
        self._joint_command_topic = self.get_parameter('joint_command_topic').value
        self._controlled_joints = list(self.get_parameter('controlled_joints').value)
        self._publish_full_joint_command = bool(
            self.get_parameter('publish_full_joint_command').value
        )

        self._cb_group = ReentrantCallbackGroup()

        self._joint_command_pub = self.create_publisher(
            JointState, self._joint_command_topic, 10
        )
        self._joint_state_sub = self.create_subscription(
            JointState,
            self._joint_state_topic,
            self._on_joint_state,
            10,
            callback_group=self._cb_group,
        )

        self._latest_joint_state_map: Dict[str, float] = {}
        self._state_lock = threading.Lock()

        self._active_goal_lock = threading.Lock()
        self._active_arm_goal: Optional[ServerGoalHandle] = None

        self._arm_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'arm_controller/follow_joint_trajectory',
            execute_callback=self._execute_arm_trajectory,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=self._cb_group,
        )

        self._gripper_action_server = ActionServer(
            self,
            GripperCommand,
            'gripper_controller/gripper_cmd',
            execute_callback=self._execute_gripper,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=self._cb_group,
        )

        self.get_logger().info(
            'SO-101 Isaac bridge ready. joint_state_topic=%s joint_command_topic=%s'
            % (self._joint_state_topic, self._joint_command_topic)
        )

    def _on_goal(self, _goal_request):
        return GoalResponse.ACCEPT

    def _on_cancel(self, _goal_handle):
        return CancelResponse.ACCEPT

    def _on_joint_state(self, msg: JointState) -> None:
        with self._state_lock:
            self._latest_joint_state_map = {
                name: msg.position[index]
                for index, name in enumerate(msg.name)
                if index < len(msg.position)
            }

    def _get_start_positions(self, joint_names: List[str]) -> Dict[str, float]:
        with self._state_lock:
            return {name: self._latest_joint_state_map.get(name, 0.0) for name in joint_names}

    def _make_joint_command(self, joint_names: List[str], positions: List[float]) -> JointState:
        command_map = dict(zip(joint_names, positions))

        if self._publish_full_joint_command:
            with self._state_lock:
                full_command_map = {
                    name: self._latest_joint_state_map.get(name, 0.0)
                    for name in self._controlled_joints
                }
            full_command_map.update(command_map)
            command_names = list(self._controlled_joints)
            command_positions = [full_command_map[name] for name in command_names]
        else:
            command_names = joint_names
            command_positions = positions

        cmd = JointState()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.name = command_names
        cmd.position = command_positions
        cmd.velocity = []
        cmd.effort = []
        return cmd

    @staticmethod
    def _to_sec(point: JointTrajectoryPoint) -> float:
        return float(point.time_from_start.sec) + float(point.time_from_start.nanosec) * 1e-9

    def _execute_arm_trajectory(
        self,
        goal_handle: ServerGoalHandle,
    ) -> FollowJointTrajectory.Result:
        trajectory: JointTrajectory = goal_handle.request.trajectory

        if not trajectory.joint_names:
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
            result.error_string = 'No joint names in trajectory'
            goal_handle.abort()
            return result

        if not trajectory.points:
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = 'No points in trajectory'
            goal_handle.abort()
            return result

        with self._active_goal_lock:
            if self._active_arm_goal is not None and self._active_arm_goal.is_active:
                self._active_arm_goal.abort()
            self._active_arm_goal = goal_handle

        start_positions = self._get_start_positions(list(trajectory.joint_names))
        last_t = 0.0
        start_wall = time.monotonic()

        for index, point in enumerate(trajectory.points):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                result.error_string = 'Goal canceled'
                return result

            if len(point.positions) != len(trajectory.joint_names):
                goal_handle.abort()
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = f'Point {index} size mismatch with joint_names'
                return result

            target_t = self._to_sec(point)
            if target_t < last_t:
                target_t = last_t

            while (time.monotonic() - start_wall) < target_t:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result = FollowJointTrajectory.Result()
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    result.error_string = 'Goal canceled'
                    return result
                time.sleep(0.001)

            cmd_msg = self._make_joint_command(
                list(trajectory.joint_names), list(point.positions)
            )
            self._joint_command_pub.publish(cmd_msg)

            feedback = FollowJointTrajectory.Feedback()
            feedback.joint_names = list(trajectory.joint_names)
            feedback.desired = point

            actual = JointTrajectoryPoint()
            actual.time_from_start = point.time_from_start
            with self._state_lock:
                actual.positions = [
                    self._latest_joint_state_map.get(
                        name, start_positions.get(name, 0.0)
                    )
                    for name in trajectory.joint_names
                ]
            feedback.actual = actual
            feedback.error = JointTrajectoryPoint()
            goal_handle.publish_feedback(feedback)

            last_t = target_t

        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = 'Executed trajectory by publishing JointState commands'
        return result

    def _execute_gripper(self, goal_handle: ServerGoalHandle) -> GripperCommand.Result:
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return GripperCommand.Result()

        target = float(goal_handle.request.command.position)
        cmd_msg = self._make_joint_command(['gripper'], [target])
        self._joint_command_pub.publish(cmd_msg)

        goal_handle.succeed()
        result = GripperCommand.Result()
        result.position = target
        result.effort = 0.0
        result.stalled = False
        result.reached_goal = True
        return result


def main() -> None:
    rclpy.init()
    node = SO101IsaacJointCommandBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
