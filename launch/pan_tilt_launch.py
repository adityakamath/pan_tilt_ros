# Copyright (c) 2024 Aditya Kamath
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pan_tilt_cmd_config_path = PathJoinSubstitution(
        [FindPackageShare("pan_tilt_ros"), "config", "cmd_config.yaml"])

    pan_tilt_ctrl_config_path = PathJoinSubstitution(
        [FindPackageShare("pan_tilt_ros"), "config", "ctrl_config.yaml"])

    return LaunchDescription([
        Node(
            package='pan_tilt_ros',
            executable='pan_tilt_cmd',
            name='pan_tilt_cmd_node',
            parameters=[pan_tilt_cmd_config_path]),

        Node(
            package='pan_tilt_ros',
            executable='pan_tilt_ctrl',
            name='pan_tilt_ctrl_node',
            parameters=[pan_tilt_ctrl_config_path]),
    ])