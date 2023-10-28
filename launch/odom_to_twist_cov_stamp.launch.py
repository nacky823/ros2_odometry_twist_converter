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
            ("odom", "mugimaru_odom"),
        ],
        parameters = [params_file],
    )

    ld = LaunchDescription()

    ld.add_action(launch_setup_func)
    ld.add_action(ndt_node)

    return ld