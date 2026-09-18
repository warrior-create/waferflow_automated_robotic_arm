from setuptools import setup, find_packages

package_name = 'wafer_trajectory'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy', 'scipy'],
    zip_safe=True,
    maintainer='WaferFlow Developer',
    maintainer_email='you@example.com',
    description='THE DIFFERENTIATOR: Jerk-limited S-curve trajectory time parameterization '
                'as a ROS2 action server. Replaces MoveIt default TOTG.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trajectory_action_server = wafer_trajectory.trajectory_action_server:main',
        ],
    },
)
