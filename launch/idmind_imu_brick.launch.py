# Copyright 2024 IDMind
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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # Robot name
    default_robot_name = ""
    robot_name = DeclareLaunchArgument(name="robot_name", default_value=default_robot_name, description="Name of this robot, used for namespacing")    
    ld.add_action(robot_name)    

    # Simulation
    simulation = DeclareLaunchArgument(name="simulation", default_value="False", description="Is the robot in simulation?")    
    ld.add_action(simulation)    

    def_imu_cfg = os.path.join(get_package_share_directory('idmind_imu'), 'config', 'idmind_imu.yaml')
    imu_cfg = DeclareLaunchArgument(name='imu_cfg', default_value=def_imu_cfg, description='Configuration file for IMU')
    ld.add_action(imu_cfg)

    imu_node = Node(
        package='idmind_imu',
        executable='imu_brick_node_v2',
        name='idmind_imu',
        namespace=LaunchConfiguration('robot_name'),
        output='screen',
        emulate_tty=True,
        parameters=[LaunchConfiguration('imu_cfg')],
        remappings=[],
    )
    ld.add_action(imu_node)

    return ld
