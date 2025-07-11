from setuptools import find_packages, setup

package_name = 'gps_action_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'utm'],
    zip_safe=True,
    maintainer='m2nyok2',
    maintainer_email='m2nyok2@hanyang.ac.kr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_action_node = gps_action_planner.gps_action_node:main',
        ],
    },
)
