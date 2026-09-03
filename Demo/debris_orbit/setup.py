from glob import glob
import os

from setuptools import find_packages, setup


package_name = "debris_orbit"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Kerbal LiDAR Lab",
    maintainer_email="user@example.com",
    description="LiDAR-guided debris orbit and 30-degree image capture demo for KSP ROS2.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "debris_orbit_node = debris_orbit.node:main",
        ]
    },
)
