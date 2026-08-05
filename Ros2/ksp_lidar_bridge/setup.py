from setuptools import setup

package_name = "ksp_lidar_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Kerbal LiDAR Lab",
    maintainer_email="user@example.com",
    description="Bridge KerbalLiDAR UDP JSON packets to ROS2 sensor topics.",
    license="MIT",
    test_suite="test.test_packet_conversion",
    entry_points={
        "console_scripts": [
            "udp_bridge = ksp_lidar_bridge.udp_bridge:main",
        ],
    },
)
