from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'last_smooth_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.*'))),
        (os.path.join('share', package_name, 'maps'), glob(os.path.join('maps', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gunes',
    maintainer_email='gunes@todo.todo',
    description='6 tekerlekli robot için pürüzsüz hareket controller paketi. PID kontrol, velocity filtering ve motion planning özellikleri içerir.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'set_initial_pose = last_smooth_controller.scripts.set_initial_pose:main',
        ],
    },
)
