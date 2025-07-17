from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'last_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/**/*', recursive=True)),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/**/*', recursive=True)),
        (os.path.join('share', package_name, 'config'), glob('config/*'))
    ],
    zip_safe=True,
    maintainer='author',
    maintainer_email='todo@todo.com',
    description='The ' + package_name + ' package',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'cmd_vel_publisher = last_description.controller:main',
            'cmd_vel_subscriber = last_description.subscriber:main',
            'joint_state_publisher = last_description.joint_state_publisher:main',
            'robot_state_publisher = last_description.robot_state_publisher:main',
            'rviz2 = last_description.rviz2:main',
            'controller = last_description.controller:main',

        ],
    },
)
