import os
from glob import glob
from setuptools import setup

package_name = 'simple_tracker_launch'

setup(
    #name=package_name,
    #version='0.0.0',
    #packages=[package_name],
    data_files=[
        # ... Other data files
        # Include all launch files.
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    #install_requires=['setuptools'],
    #zip_safe=True,
    #maintainer='ros',
    #maintainer_email='michael.groenewald@gmail.com',
    #description='TODO: Package description',
    #license='TODO: License declaration',
    #tests_require=['pytest'],
    #entry_points={
    #    'console_scripts': [
    #        'launch = simple_tracker_launch.simple_tracker_launch.py',
    #    ],
    #},
)
