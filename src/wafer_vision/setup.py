from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'wafer_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='WaferFlow Developer',
    maintainer_email='you@example.com',
    description='Vision package: overhead camera + wafer disc detection + AlignWafer service',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_sim_node = wafer_vision.camera_sim_node:main',
            'alignment_service = wafer_vision.alignment_service:main',
        ],
    },
)
