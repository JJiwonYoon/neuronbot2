import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. xacro 파일 경로 지정
    xacro_path = os.path.join(
        get_package_share_directory('neuronbot2_description'),
        'urdf',
        'husky_bot.urdf.xacro'
    )

    # 2. xacro → urdf 변환
    robot_description_config = xacro.process_file(xacro_path)
    robot_desc = robot_description_config.toxml()

    # 3. robot_state_publisher 노드 생성
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
    )

    # 4. (선택) joint_state_publisher_gui 노드도 추가할 수 있음
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
    )

    # 5. LaunchDescription 반환
    return LaunchDescription([
        joint_state_publisher_node,
        robot_state_publisher_node,
        # 필요시 추가 노드 (예: RViz, Gazebo 등)
    ])
