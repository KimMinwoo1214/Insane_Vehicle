from setuptools import setup

package_name = 'lidar_only_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='antel',
    maintainer_email='your@email.com',
    description='Lidar only path planner',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_only_planner = lidar_only_planner.lidar_only_planner:main'
        ],
    },
)

