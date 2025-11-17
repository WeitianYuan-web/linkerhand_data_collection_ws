from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'linkerhand_data_collection_srv'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'srv'), glob('srv/*.srv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='linkerhand',
    maintainer_email='linkerhand@todo.todo',
    description='LinkerHand Data Collection Service Package for ROS2',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'linkerhand_data_collection_node = scripts.linkerhand_data_collection:main',
            'linkerhand_data_collection.py = scripts.linkerhand_data_collection:main',  # 别名，兼容旧命令（带.py）
            'mqtt_node = scripts.mqtt:main',
        ],
    },
)
