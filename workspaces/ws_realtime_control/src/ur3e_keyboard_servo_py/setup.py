from glob import glob

from setuptools import find_packages, setup

package_name = 'ur3e_keyboard_servo_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='minzi',
    maintainer_email='chenmj75@mail2.sysu.edu.cn',
    description='Keyboard Twist command publisher for UR3e MoveIt Servo experiments.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'keyboard_servo_node = ur3e_keyboard_servo_py.keyboard_servo_node:main',
        ],
    },
)
