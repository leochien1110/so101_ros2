from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_isaac_sim = LaunchConfiguration('use_isaac_sim')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_state_max_bounds_error = LaunchConfiguration('start_state_max_bounds_error')
    joint_state_topic = LaunchConfiguration('joint_state_topic')
    joint_command_topic = LaunchConfiguration('joint_command_topic')
    publish_full_joint_command = LaunchConfiguration('publish_full_joint_command')

    moveit_config = (
        MoveItConfigsBuilder('so101_new_calib', package_name='so101_moveit')
        .robot_description(mappings={'use_fake_hardware': 'true'})
        .to_moveit_configs()
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        condition=IfCondition(use_isaac_sim),
        parameters=[
            moveit_config.robot_description,
            {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
        ],
    )

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        condition=IfCondition(use_isaac_sim),
        parameters=[
            moveit_config.to_dict(),
            {
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'start_state_max_bounds_error': ParameterValue(
                    start_state_max_bounds_error, value_type=float
                ),
            },
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=[
            '-d',
            PathJoinSubstitution(
                [FindPackageShare('so101_moveit'), 'config', 'moveit.rviz']
            ),
        ],
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
        ],
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    use_isaac_sim,
                    "'.lower() == 'true' and '",
                    use_rviz,
                    "'.lower() == 'true'",
                ]
            )
        ),
    )

    bridge_node = Node(
        package='so101_isaac_bridge',
        executable='so101_isaac_joint_command_bridge',
        output='screen',
        condition=IfCondition(use_isaac_sim),
        parameters=[
            {
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'joint_state_topic': joint_state_topic,
                'joint_command_topic': joint_command_topic,
                'publish_full_joint_command': ParameterValue(
                    publish_full_joint_command, value_type=bool
                ),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_isaac_sim',
                default_value='true',
                choices=['true', 'false'],
            ),
            DeclareLaunchArgument('use_rviz', default_value='true'),
            DeclareLaunchArgument('use_sim_time', default_value='false'),
            DeclareLaunchArgument('start_state_max_bounds_error', default_value='0.001'),
            DeclareLaunchArgument('joint_state_topic', default_value='/joint_states'),
            DeclareLaunchArgument('joint_command_topic', default_value='/joint_command'),
            DeclareLaunchArgument('publish_full_joint_command', default_value='true'),
            robot_state_publisher_node,
            move_group_node,
            rviz_node,
            bridge_node,
        ]
    )