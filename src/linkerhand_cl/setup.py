from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'linkerhand_cl'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ywt',
    maintainer_email='ywt@todo.todo',
    description='LinkerHand控制节点，用于订阅EXHand映射数据话题来控制灵巧手',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'linkerhand_control_node = linkerhand_cl.linkerhand_control_node:main',
        ],
    },
)

