# Copyright 2019 Canonical, Ltd.
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
from re import template

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch.actions
import launch_ros.actions


# Please note that this is only an example!
# It is not guaranteed to work with your setup but can be used as a starting point.

def generate_launch_description():
    
    # joy_dev = launch.substitutions.LaunchConfiguration('joy_dev')
    controller_yaml = os.path.join(get_package_share_directory('moveit_servo'),
        'config', 'xbox_teleop.yaml')
    
    ld = LaunchDescription([
        # launch.actions.DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0'),
        launch.actions.DeclareLaunchArgument('teleop_config', default_value=controller_yaml),
    ])

    # ld.add_action(launch_ros.actions.Node(
    #         package='joy', executable='joy_node',
    #         parameters=[{
    #             'dev': joy_dev,
    #             'autorepeat_rate': 20.0,
    #         }]))

    ld.add_action(launch_ros.actions.Node(
            package='joy_teleop', executable='joy_teleop',
            parameters=[launch.substitutions.LaunchConfiguration('teleop_config')]))

    return ld