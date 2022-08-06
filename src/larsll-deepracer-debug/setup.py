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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ludvigse',
    maintainer_email='larsll@outlook.com',
    description='Debugging tools for DeepRacer',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test = debug_pkg.debug_imu_node:main'
        ],
    },
)
