# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # urdf 파일의 절대경로를 직접 지정
    urdf_path = '/home/oskar/nav2_ws/src/neuronbot2/neuronbot2_description/urdf/test.urdf'

    # Launch arguments
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='x position of the robot')
    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='y position of the robot')

    # exploration_bot 스폰 노드
    spawn_exploration_bot_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'exploration_bot',
            '-file', urdf_path,
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
    ld.add_action(spawn_exploration_bot_cmd)

    return ld
