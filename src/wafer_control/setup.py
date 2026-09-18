from setuptools import setup, find_packages

package_name = 'wafer_control'

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
    description='Main state machine for WaferFlow pick-place cycles',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pick_place_fsm = wafer_control.pick_place_fsm:main',
        ],
    },
)
