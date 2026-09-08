from setuptools import find_packages, setup

package_name = "ksp_lidar_bridge"

setup(
    name=package_name,
    version="0.3.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="Kerbal LiDAR Lab",
    maintainer_email="user@example.com",
    description="Bridge KerbalLiDAR sensors, motors, propulsion, and runtime URDF proxies to ROS2.",
    license="MIT",
    test_suite="test.test_packet_conversion",
    entry_points={
        "console_scripts": [
            "udp_bridge = ksp_lidar_bridge.udp_bridge:main",
        ],
    },
)
