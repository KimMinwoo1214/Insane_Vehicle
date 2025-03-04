from setuptools import setup, find_packages

package_name = 'CalibJ'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['CalibJ', 'CalibJ.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='f1tenth',
    maintainer_email='f1tenth@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'calib = CalibJ.calib:main',
            'camera_calib = CalibJ.camera_calibration:main',
            'viscam = CalibJ.viscam:main'
        ],
    },
)
