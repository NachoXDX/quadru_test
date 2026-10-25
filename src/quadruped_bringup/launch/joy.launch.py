# Copyright 2026 Nacho
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

"""Gamepad teleop: joy -> teleop_twist_joy -> /cmd_vel, plus buttons for stand/lie."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = LaunchConfiguration('config')
    use_sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time')}

    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=PathJoinSubstitution(
                [FindPackageShare('quadruped_bringup'), 'config', 'teleop_ps5.yaml']),
            description='Gamepad mapping (joy, teleop_twist_joy, joy_actions_node params)'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='joy',
            executable='game_controller_node',
            parameters=[config, use_sim_time],
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[config, use_sim_time],
        ),
        Node(
            package='quadruped_control',
            executable='joy_actions_node',
            parameters=[config, use_sim_time],
        ),
    ])
