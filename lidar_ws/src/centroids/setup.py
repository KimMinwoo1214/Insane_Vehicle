from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'centroids'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob(os.path.join('launch', '*.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kmw',
    maintainer_email='werkm1214@hanyang.ac.kr',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
        'centroids_visualize = centroids.centroids_visualize:main',
        'lr_dis = centroids.lr_dis:main',
        ],
    },
)
