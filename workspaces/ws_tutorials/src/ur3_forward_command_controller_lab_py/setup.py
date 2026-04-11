from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ur3_forward_command_controller_lab_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='minzi',
    maintainer_email='chenmj75@mail2.sysu.edu.cn',
    description='Task 4G: forward_command_controller vs joint_trajectory_controller comparison',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'forward_cmd_publisher = ur3_forward_command_controller_lab_py.forward_cmd_publisher:main',
        ],
    },
)
