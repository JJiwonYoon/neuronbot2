import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    # xacro 파일 경로 지정
    xacro_path = '/home/oskar/nav2_ws/src/neuronbot2/neuronbot2_description/urdf/husky_bot.urdf.xacro'

    # xacro → urdf 변환
    robot_description_config = xacro.process_file(xacro_path)
    robot_desc = robot_description_config.toxml()

    # Launch arguments
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='x position of the robot')
    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='y position of the robot')

    # robot_state_publisher 노드 (필수!)
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # Gazebo에 로봇 스폰
    spawn_exploration_bot_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'exploration_bot',
            '-topic', 'robot_description',
            '-timeout', '300',
            '-x', x_pose,
            '-y', y_pose,
            '-z', '1.0',
        ],
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_exploration_bot_cmd)

    return ld
