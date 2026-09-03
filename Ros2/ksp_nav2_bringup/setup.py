from glob import glob
import os

from setuptools import find_packages, setup


package_name = "ksp_nav2_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "params"), glob("params/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Kerbal LiDAR Lab",
    maintainer_email="user@example.com",
    description="Isolated Nav2 and SLAM Toolbox integration for the KSP ROS2 bridge.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "laser_scan_odometry = ksp_nav2_bringup.laser_scan_odometry:main",
            "planar_wrench_controller = ksp_nav2_bringup.planar_wrench_controller:main",
        ],
    },
)
