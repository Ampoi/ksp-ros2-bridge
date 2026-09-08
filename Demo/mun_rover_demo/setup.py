from glob import glob
from os.path import isfile
from setuptools import setup, find_packages
name='mun_rover_demo'
setup(name=name,version='0.1.0',packages=find_packages(exclude=['test']),
    data_files=[('share/ament_index/resource_index/packages',['resource/'+name]),('share/'+name,['package.xml'])]+
        [('share/'+name+'/'+folder,[p for p in glob(folder+'/*') if isfile(p)]) for folder in ('launch','config','rviz')],
    install_requires=['setuptools','numpy','scipy'],zip_safe=True,
    maintainer='Kerbal LiDAR Lab',maintainer_email='user@example.com',license='MIT',
    description='Sensor-only Mun rover navigation with Nav2 and guarded wheel control',
    entry_points={'console_scripts':['rover_node=mun_rover_demo.node:main','evaluate=mun_rover_demo.evaluation:main']})
