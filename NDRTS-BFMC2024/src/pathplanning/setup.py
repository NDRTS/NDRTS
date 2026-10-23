from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'pathplanning'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.py')),
        (os.path.join('share', package_name, 'maps'),
         glob('maps/*.graphml')),
        (os.path.join('share', package_name, 'maps'),
         glob('maps/*.graphml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ndrts',
    maintainer_email='ndrts@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_publisher_node = pathplanning.path_publisher_node:main',
            'gps_position_node = pathplanning.gps_position_node:main',
            'pure_pursuit_node = pathplanning.pure_pursuit_node:main',
        ],
    },
)
