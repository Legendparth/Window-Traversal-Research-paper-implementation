from setuptools import setup
import os
from glob import glob

package_name = 'imav_indoor_2026'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # This line installs the package index marker
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # This line installs the package.xml file
        (os.path.join('share', package_name), ['package.xml']),
        # This line installs all launch files. Adjust the path as necessary
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='youremail@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drone_joystick_cam = imav_indoor_2026.drone_joystick_cam:main',
        ],
    },
)