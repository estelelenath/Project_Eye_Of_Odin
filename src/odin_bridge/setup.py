from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'odin_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='Bridge node for Project Odin',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bridge_node = odin_bridge.bridge_node:main',
        ],
    },
)

