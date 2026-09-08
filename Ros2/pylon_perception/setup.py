from setuptools import setup, find_packages
setup(name="pylon_perception", version="1.0.0", packages=find_packages(), data_files=[("share/ament_index/resource_index/packages", ["resource/pylon_perception"]), ("share/pylon_perception", ["package.xml"])], install_requires=["setuptools"], license="MIT")
