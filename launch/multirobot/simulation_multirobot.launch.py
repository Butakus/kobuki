# Copyright (c) 2024 Intelligent Robotics Lab (URJC)
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

# Modified by Juan Carlos Manzanares Serrano

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# List of robots to be spawned. This list contains the dictionaries
# of launch arguments that are passed to the kobuki_description spawn launcher
# TODO: Move this config to a YAML file
robots = [
    {
        'name': 'kobuki_1',  # Gazebo model name
        'namespace': 'r1',  # Namespace for topics and TF frames
    },
    {
        'name': 'kobuki_2',
        'namespace': 'r2',
        'x': '2.0',
        'y': '1.0',
        'Y': '0.0',
    }
]


def start_gz_sim(context, *args, **kwargs):

    world = LaunchConfiguration('world').perform(context)

    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch',
                         'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r -s ', world]}.items()
    )

    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch',
                         'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': [' -g ']}.items(),
    )

    return [start_gazebo_server_cmd, start_gazebo_client_cmd]


def generate_launch_description():

    declare_world_cmd = DeclareLaunchArgument(
        'world', default_value=os.path.join(
            get_package_share_directory('aws_robomaker_small_house_world'),
            'worlds',
            'small_house.world'))

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch',
                         'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r -s ', LaunchConfiguration('world')]}.items()
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch',
                         'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': [' -g ']}.items(),
    )

    spawn_robot_list = []
    for robot_args in robots:
        spawn_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('kobuki_description'),
                'launch/'), 'spawn.launch.py']),
            launch_arguments=robot_args.items()
        )
        spawn_robot_list.append(spawn_robot)

    ld = LaunchDescription()
    ld.add_action(declare_world_cmd)
    ld.add_action(gazebo_server)
    ld.add_action(gazebo_client)
    for spawn_action in spawn_robot_list:
        ld.add_action(spawn_action)

    return ld
