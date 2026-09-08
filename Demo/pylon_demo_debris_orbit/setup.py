from glob import glob
import os

from setuptools import find_packages, setup


package_name = "pylon_demo_debris_orbit"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools", "numpy", "scipy"],
    zip_safe=True,
    maintainer="PyLoN",
    maintainer_email="user@example.com",
    description="LiDAR/IMU debris orbit without Ground Truth and 36-degree image capture demo for KSP ROS2.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "pylon_demo_debris_orbit_node = pylon_demo_debris_orbit.node:main",
            "target_estimator = pylon_demo_debris_orbit.target_node:main",
        ]
    },
)
