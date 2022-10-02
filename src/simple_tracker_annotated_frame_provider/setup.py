from setuptools import setup

package_name = 'simple_tracker_annotated_frame_provider'

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
            'annotated_frame_provider = simple_tracker_annotated_frame_provider.annotated_frame_provider_node:main',
            'rx_annotated_frame_provider = simple_tracker_annotated_frame_provider.rx_annotated_frame_provider_node:main',
        ],
    },
)
