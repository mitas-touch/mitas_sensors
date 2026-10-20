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

        # ── Realsense Wrist Camera ──────────────────────────────────────────
        Node(
            package="mitas_sensors",
            executable="realsense_node",
            name="realsense_node",
            namespace="mitas_sensors",
            output="screen",
            parameters=[config],
        ),

        # ── GelSight Mini ─────────────────────────────────────────────────────
        Node(
            package="mitas_sensors",
            executable="gelsight_mini_node",
            name="gelsight_mini_node",
            namespace="mitas_sensors",
            output="screen",
            parameters=[config],
        ),

        # ── Evetac (DV event camera) ──────────────────────────────────────────
        Node(
            package="mitas_sensors",
            executable="evetac_node",
            name="evetac_node",
            namespace="mitas_sensors",
            output="screen",
            parameters=[config],
        ),

    ])
