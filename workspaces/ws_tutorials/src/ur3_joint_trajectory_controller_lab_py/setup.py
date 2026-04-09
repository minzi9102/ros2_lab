import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ur3_joint_trajectory_controller_lab_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.xacro'))),
        (os.path.join('share', package_name, 'scripts'), glob(os.path.join('scripts', '*.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='minzi',
    maintainer_email='chenmj75@mail2.sysu.edu.cn',
    description='Task 4F: joint_trajectory_controller integration and testing',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'test_trajectory_client = ur3_joint_trajectory_controller_lab_py.test_trajectory_client:main',
        ],
    },
)
