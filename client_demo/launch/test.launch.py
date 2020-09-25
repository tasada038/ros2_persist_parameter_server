# Copyright 2020 Sony Corporation
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

"""Launch server && client"""

from launch import LaunchDescription
from launch.substitutions import EnvironmentVariable
import launch_ros.actions
import launch
import os
import sys 
import pathlib

respawn_delay = 0.0

def generate_launch_description():
    return LaunchDescription([
        launch.actions.ExecuteProcess(
            cmd = ['ros2', 'run', 'parameter_server', 'server', '--file-path', '/tmp/parameter_server.yaml'],
            respawn=True, respawn_delay=respawn_delay
        ),
        launch.actions.ExecuteProcess(
            cmd = ['ros2', 'run', 'client_demo', 'client']
        )
    ])
