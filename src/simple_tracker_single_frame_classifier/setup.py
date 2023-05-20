from setuptools import setup

package_name = 'simple_tracker_single_frame_classifier'

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
    maintainer='',
    maintainer_email='',
    description='ROS2 Python package to classify objects in a single frame',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'single_frame_classifier = simple_tracker_single_frame_classifier.simple_tracker_single_frame_classifier_node:main',
        ],
    },
)