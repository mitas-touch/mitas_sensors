from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("mitas_sensors"),
        "config",
        "sensors.yaml",
    )

    return LaunchDescription([
        Node(
            package="mitas_sensors",
            executable="evetac_node",
            name="evetac_node",
            namespace="mitas_sensors",
            output="screen",
            parameters=[config],
        ),
    ])
