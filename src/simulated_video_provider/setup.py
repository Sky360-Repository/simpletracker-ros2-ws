import os
from glob import glob
from setuptools import setup

package_name = 'simulated_video_provider'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='michael.groenewald@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simulated_video_provider = simulated_video_provider.simulated_video_provider_node:main',
            'simulation_overlay_provider = simulated_video_provider.simulation_overlay_provider_node:main',
        ],
    },
)
