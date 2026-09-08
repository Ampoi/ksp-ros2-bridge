from setuptools import find_packages, setup


package_name = "pylon_vehicle_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="PyLoN",
    maintainer_email="user@example.com",
    description="Lease-aware reusable pose and velocity control for KSP ROS2 vessels.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "setpoint_controller = pylon_vehicle_control.adapters.ros2.setpoint_controller:main",
        ]
    },
)
