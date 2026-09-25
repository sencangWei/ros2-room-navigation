from glob import glob
from setuptools import find_packages, setup

package_name = 'room_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@example.com',
    description='A small ROS 2 room navigation exercise.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'simulator = room_nav.simulator:main',
            'navigator = room_nav.navigator:main',
        ],
    },
)
