from glob import glob

from setuptools import find_packages, setup

package_name = 'ur3_mode_service_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='minzi',
    maintainer_email='chenmj75@mail2.sysu.edu.cn',
    description='Minimal UR3 mode service demo in Python for ROS 2 learning.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mode_service_server = ur3_mode_service_py.mode_service_server:main',
            'mode_service_client = ur3_mode_service_py.mode_service_client:main',
        ],
    },
)
