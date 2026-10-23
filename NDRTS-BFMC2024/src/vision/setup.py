from glob import glob  # add this import
from setuptools import setup, find_packages
import os

package_name = 'vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'models'), [
            'vision/models/best.pt',
            # 'vision/models/yolo11n.onnx',
            # 'vision/models/best.engine',
            # 'vision/models/mb2-ssd-lite_op13.onnx',
            'vision/models/voc-model-labels.txt',
        ]),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.py')),
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
            'camera_node = vision.camera_node:main',
            'custom_detect_node = vision.custom_detect_node:main',
            'inference_node = vision.inference_node:main',
        ],
    },
)
