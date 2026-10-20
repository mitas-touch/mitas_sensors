from setuptools import find_packages, setup
import os
from glob import glob

package_name = "mitas_sensors"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Rickmer Krohn",
    maintainer_email="rickmer.krohn@tu-darmstadt.de",
    description="ROS 2 drivers for the MiTaS sensor suite: GelSight Mini, Evetac, and a RealSense wrist camera.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "gelsight_mini_node = mitas_sensors.gelsight_mini_node:main",
            "evetac_node        = mitas_sensors.evetac_node:main",
            "realsense_node     = mitas_sensors.realsense_node:main",
        ],
    },
)
