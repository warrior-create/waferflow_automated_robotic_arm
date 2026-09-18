from setuptools import setup, find_packages

package_name = 'wafer_benchmark'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pandas', 'matplotlib'],
    zip_safe=True,
    maintainer='WaferFlow Developer',
    maintainer_email='you@example.com',
    description='Automated cycle time sweep, logging, and plotting for WaferFlow',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sweep_runner = wafer_benchmark.sweep_runner:main',
            'metrics_logger = wafer_benchmark.metrics_logger:main',
            'plot_results = wafer_benchmark.plot_results:main',
        ],
    },
)
