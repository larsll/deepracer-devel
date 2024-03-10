import os
from setuptools import setup

package_name = 'debug_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name), ["launch/debug_pkg_launch.py"]),
    ],
    install_requires=['setuptools', 'transforms3d', 'rosbags'],
    zip_safe=True,
    maintainer='ludvigse',
    maintainer_email='larsll@outlook.com',
    description='Debugging tools for DeepRacer',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu = debug_pkg.debug_imu_node:main',
            'camera_replay_node = debug_pkg.camera_replay_node:main',
            'single_picture_injection_node = debug_pkg.single_picture_injection_node:main',
            'inference_comparison_node = debug_pkg.inference_comparison_node:main'
        ],
    },
)
