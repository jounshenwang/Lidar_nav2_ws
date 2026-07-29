from setuptools import find_packages, setup

package_name = 'serial_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/serial_bridge_launch.py',
            'launch/real_robot_navigation_launch.py',
            'launch/real_robot_mapping_launch.py',
        ]),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.com',
    description='ROS2 serial bridge to ESP32-S3 motor controller',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_bridge = serial_bridge.serial_bridge:main',
        ],
    },
)
