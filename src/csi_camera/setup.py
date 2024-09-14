from setuptools import setup
import os
from glob import glob

package_name = 'csi_camera'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='CSI 카메라 처리를 위한 ROS2 패키지',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'csi_camera_node = csi_camera.csi_camera_node:main',
        ],
    },
    python_requires='>=3.6',
)

