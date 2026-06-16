from glob import glob

from setuptools import find_packages, setup

package_name = 'dynamixel_demo'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='DingoOz',
    maintainer_email='nigel.hungerfordsymes@gmail.com',
    description='Self-contained single-Dynamixel control demos (raw SDK + ROS 2).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'single_servo_demo = dynamixel_demo.single_servo_demo:main',
            'servo_ros_node = dynamixel_demo.servo_ros_node:main',
        ],
    },
)
