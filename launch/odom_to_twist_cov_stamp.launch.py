# Copyright 2023 nacky823
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
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from launch_ros.actions import SetParameter


def launch_setup(context, *args, **kwargs):
    set_use_sim_time = SetParameter(name="use_sim_time", value="true")
    return [set_use_sim_time]

def generate_launch_description():

    launch_setup_func = OpaqueFunction(function=launch_setup)

    pkg = "ros2_odometry_twist_converter"

    params_file = os.path.join(
        get_package_share_directory(pkg),
        "config",
        "odom_to_twist_cov_stamp.param.yaml",
    )

    ndt_node = Node(
        package = pkg,
        executable = "odometry_twist_converter",
        name = "odom_to_twist_cov_stamp",
        output = "screen",
        remappings = [
            ("mugimaru_odom", "odom"),
        ],
        parameters = [params_file],
    )

    ld = LaunchDescription()

    ld.add_action(launch_setup_func)
    ld.add_action(ndt_node)

    return ld
