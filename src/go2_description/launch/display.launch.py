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

"""Show the Go2 in RViz with sliders for every joint."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('go2_description')
    gui = LaunchConfiguration('gui')
    # Start the sliders at the Go2 standing pose instead of zeros.
    stand_pose = {'zeros': {
        f'{leg}_{joint}_joint': angle
        for leg in ('FL', 'FR', 'RL', 'RR')
        for joint, angle in (('hip', 0.0), ('thigh', 0.8), ('calf', -1.5))
    }}

    robot_description = ParameterValue(
        Command(['xacro ', PathJoinSubstitution([pkg_share, 'urdf', 'go2.urdf.xacro'])]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='Use joint_state_publisher_gui sliders'),
        # RViz's Ogre window misbehaves under Qt HiDPI scaling (see CLAUDE.md).
        SetEnvironmentVariable('QT_ENABLE_HIGHDPI_SCALING', '0'),
        SetEnvironmentVariable('QT_FONT_DPI', '192'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            parameters=[stand_pose],
            condition=IfCondition(gui),
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            parameters=[stand_pose],
            condition=UnlessCondition(gui),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', PathJoinSubstitution([pkg_share, 'rviz', 'display.rviz'])],
        ),
    ])
