from setuptools import find_packages, setup

package_name = 'yolo_lane'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='antel',
    maintainer_email='ati0423@hanyang.ac.kr',
    description='YOLO Lane Detection Angle Estimator',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'antel_lane = yolo_lane.antel_lane:main',
            'parkm_lane = yolo_lane.parkm_lane:main',
            'alsgur_lane = yolo_lane.antel_lane_editted:main',
            'mix_lane = yolo_lane.antel_lane_blended:main'# ✅ 실행 엔트리포인트 등록
        ],
    },
)

