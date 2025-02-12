from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mega_con'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob(os.path.join('launch', '*.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kmw',
    maintainer_email='werkm1214@hanyang.ac.kr',
    description='ROS2 package for joystick and serial control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tel = mega_con.tel_motor:main',
            'con = mega_con.motor_con:main',
            'joy_con = mega_con.joy_con:main',
        ],
    },
)

