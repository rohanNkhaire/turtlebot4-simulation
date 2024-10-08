# Copyright 2021 Clearpath Robotics, Inc.
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
#
# @author Roni Kreinin (rkreinin@clearpathrobotics.com)

import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable,
                            TimerAction, GroupAction, LogInfo, RegisterEventHandler, EmitEvent)
from launch.substitutions import Command, PathJoinSubstitution, IfElseSubstitution
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import (OnProcessExit)

from launch_ros.actions import Node


ARGUMENTS = [
    DeclareLaunchArgument('model', default_value='standard',
                          choices=['standard', 'lite'],
                          description='Turtlebot4 Model'),
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='use_sim_time'),
    DeclareLaunchArgument('robot_name', default_value='turtlebot4',
                          description='Robot name'),
    DeclareLaunchArgument("gazebo_gui", default_value="true", 
                          description="Start gazebo with GUI?"),
    DeclareLaunchArgument("world_file", default_value=PathJoinSubstitution([FindPackageShare("turtlebot4_description"), 
                                                                            "world", "depot.sdf"]),
                          description="Gazebo world file (absolute path or filename from the gazebosim worlds collection) \
                          containing a custom world."),                
] 

def generate_launch_description():
    pkg_turtlebot4_description = get_package_share_directory('turtlebot4_description')

    xacro_file = PathJoinSubstitution([pkg_turtlebot4_description,
                                       'urdf',
                                       LaunchConfiguration('model'),
                                       'turtlebot4.urdf.xacro'])

    spawn_robot_group_action = GroupAction([
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={
            "gz_args": IfElseSubstitution(
                LaunchConfiguration('gazebo_gui'),
                if_value=[" -r ", LaunchConfiguration('world_file')],
                else_value=[" -s -r -v 4 ", LaunchConfiguration('world_file')],
            )
        }.items(),
    ),
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')}, 
                {'robot_description': Command([
                    'xacro', ' ', xacro_file, ' ',
                    'gazebo:=ignition'])}, 
            ]
        ),
        # Joint State Publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static')
            ]
        ),
    ])

    # Spawn turtlebot
    spawn_turtlebot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'turtlebot4',
                   '-topic', 'robot_description'],
        output='screen'
    )

    # Launch the gz ros bridge
    turtlebot_ros_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("turtlebot4_description"), "/launch/turtlebot_gz_bridge.launch.py"]
        ))

    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    # Add nodes to LaunchDescription
    ld.add_action(spawn_robot_group_action)

    # Delay spawning turtlebot in Gazebo
    ld.add_action(TimerAction(
                period=5.0,
                actions=[spawn_turtlebot],
            ))
    # Launch the gz ros bridge
    ld.add_action(RegisterEventHandler(OnProcessExit(
            target_action=spawn_turtlebot,
            on_exit=[
                LogInfo(msg='Spawn finished'),
                turtlebot_ros_bridge
                ])))
    return ld