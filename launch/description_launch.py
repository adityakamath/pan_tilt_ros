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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import UnlessCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_path

def generate_launch_description():
    pan_tilt_description = ParameterValue(
        Command(['xacro ', str(get_package_share_path('pan_tilt_ros') / 'urdf/pan_tilt_urdf.xacro')]),
        value_type=str)

    return LaunchDescription([ 

        DeclareLaunchArgument(
            name='js_ext',
            default_value='False',
            description='Enable Joint States from external nodes (like the micro-ROS node). If False, enable Joint States from the Joint State Publisher'),

        DeclareLaunchArgument(
            name='js_topic',
            default_value='joint_states',
            description='Switch between measured (joint_states) and required (joint_commands) topics'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': pan_tilt_description}],
            remappings=[
                ('/joint_states', ['/', LaunchConfiguration('js_topic')])
            ]),
    ])