from glob import glob
import os

from setuptools import find_packages, setup


package_name = "pylon_demo_position_estimator"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="PyLoN",
    maintainer_email="user@example.com",
    description="6DoF scan-to-scan position estimator demo from 3D LiDAR for KSP ROS2.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "pylon_demo_position_estimator_node = pylon_demo_position_estimator.node:main",
        ]
    },
)
