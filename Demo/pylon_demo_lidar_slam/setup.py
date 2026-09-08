from glob import glob
import os

from setuptools import find_packages, setup


package_name = "pylon_demo_lidar_slam"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "params"), glob("params/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="PyLoN",
    maintainer_email="user@example.com",
    description="Isolated Nav2 and SLAM Toolbox integration for the KSP ROS2 bridge.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "laser_scan_odometry = pylon_demo_lidar_slam.laser_scan_odometry:main",
            "planar_wrench_controller = pylon_demo_lidar_slam.planar_wrench_controller:main",
        ],
    },
)
