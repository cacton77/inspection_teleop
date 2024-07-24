from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'inspection_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='macs',
    maintainer_email='brianjhc@uw.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'joy_to_twist = inspection_teleop.joy_to_twist:main',
                'twist_to_twist_stamped = inspection_teleop.twist_to_twist_stamped:main'
        ],
    },
)