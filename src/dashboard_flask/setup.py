import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'dashboard_flask'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'templates'),
            glob('dashboard_flask/templates/*.html')),
        (os.path.join('share', package_name, 'static'),
            glob('dashboard_flask/static/*')),
    ],
    install_requires=['setuptools', 'flask', 'flask-socketio'],
    zip_safe=True,
    maintainer='Brian',
    maintainer_email='brian@example.com',
    description='Flask web dashboard for ROS2: camera, map, AMCL pose, cmd_vel joystick',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'flask_node = dashboard_flask.flask_node:main',
        ],
    },
)
