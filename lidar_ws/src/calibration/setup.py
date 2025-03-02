from setuptools import find_packages, setup

package_name = 'calibration'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kmw',
    maintainer_email='werkm1214@hanyang.ac.kr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'lidar_point = calibration.lidar_point:main',
        'lidar_to_camera = calibration.lidar_to_camera:main',
        'camera = calibration.camera_node:main',
        'projection = calibration.projection:main',
        ],
    },
)
