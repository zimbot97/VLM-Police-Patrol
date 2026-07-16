from setuptools import setup
import os
from glob import glob

package_name = 'oled_status'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Brian',
    maintainer_email='brian@braintree.local',
    description='SSD1306 0.96" status HUD for the VLM-Police-Patrol robot.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'oled_status = oled_status.oled_status_node:main',
        ],
    },
)
