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

"""Spawn the Go2 in Gazebo with ros2_control and the posture node."""

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            RegisterEventHandler, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (Command, LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare('quadruped_bringup')
    description_share = FindPackageShare('go2_description')
    controllers_file = PathJoinSubstitution([bringup_share, 'config', 'controllers.yaml'])

    world = LaunchConfiguration('world')
    rviz = LaunchConfiguration('rviz')
    cheat = LaunchConfiguration('cheat')
    # -s runs the Gazebo server only (no GUI window), e.g. for testing.
    server_only = PythonExpression(
        ["'-s ' if '", LaunchConfiguration('headless'), "' == 'true' else ''"])
    sim_time = {'use_sim_time': True}

    robot_description = ParameterValue(
        Command([
            'xacro ', PathJoinSubstitution([description_share, 'urdf', 'go2.urdf.xacro']),
            ' sim:=true controllers_file:=', controllers_file, ' cheat:=', cheat,
        ]),
        value_type=str,
    )

    gazebo = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']),
        launch_arguments={
            'gz_args': ['-r -v 3 ', server_only, world],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # Spawned lying down (joints start at the lying pose from go2.ros2_control.xacro).
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'go2', '-z', '0.15'],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': PathJoinSubstitution(
            [bringup_share, 'config', 'bridge.yaml'])}, sim_time],
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--param-file', controllers_file],
        parameters=[sim_time],
    )
    position_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['forward_position_controller', '--param-file', controllers_file],
        parameters=[sim_time],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value=PathJoinSubstitution([bringup_share, 'worlds', 'empty.sdf'])),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('cheat', default_value='true',
                              description='Move the body with the PlanarBodyDriver push'),
        DeclareLaunchArgument('joy', default_value='false',
                              description='Start gamepad teleop (joy.launch.py)'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='Run Gazebo without its GUI'),
        # RViz's Ogre window misbehaves under Qt HiDPI scaling (see CLAUDE.md).
        SetEnvironmentVariable('QT_ENABLE_HIGHDPI_SCALING', '0'),
        SetEnvironmentVariable('QT_FONT_DPI', '192'),
        gazebo,
        bridge,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}, sim_time],
        ),
        spawn,
        # Controllers can only be loaded once the robot (and its controller_manager) exists.
        RegisterEventHandler(OnProcessExit(
            target_action=spawn, on_exit=[joint_state_broadcaster])),
        RegisterEventHandler(OnProcessExit(
            target_action=joint_state_broadcaster, on_exit=[position_controller])),
        Node(
            package='quadruped_control',
            executable='posture_node',
            parameters=[sim_time],
            output='screen',
        ),
        Node(
            package='quadruped_control',
            executable='noisy_odom_node',
            parameters=[sim_time],
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution([bringup_share, 'launch', 'joy.launch.py']),
            condition=IfCondition(LaunchConfiguration('joy')),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', PathJoinSubstitution([bringup_share, 'rviz', 'sim.rviz'])],
            parameters=[sim_time],
            condition=IfCondition(rviz),
        ),
    ])
