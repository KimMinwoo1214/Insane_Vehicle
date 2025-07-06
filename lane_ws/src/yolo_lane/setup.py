from setuptools import setup, find_packages

package_name = 'yolo_lane'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='antel',
    maintainer_email='ati0423@hanyang.ac.kr',
    description='YOLO Segmentation ROS 2 Node',
    license='MIT',
    entry_points={
        'console_scripts': [
            'yolo_lane = yolo_lane.yolo_lane:main',
            'kmw_lane = yolo_lane.kmw_lane:main',
            'engine_lane = yolo_lane.engine_lane:main'
        ],
    },
)

