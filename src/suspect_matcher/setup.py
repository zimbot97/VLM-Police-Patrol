import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'suspect_matcher'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Brian',
    maintainer_email='brian@example.com',
    description='Suspect appearance-matching node using the hobot_llamacpp VLM.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'attribute_compare = '
            'suspect_matcher.attribute_compare_from_files_node:main',
            'yoloworld_detect = '
            'suspect_matcher.yoloworld_detect_node:main',
            'yolo_detect = '
            'suspect_matcher.yolo_detect_node:main',
        ],
    },
)
