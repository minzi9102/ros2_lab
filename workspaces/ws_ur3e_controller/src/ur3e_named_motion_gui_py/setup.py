from glob import glob

from setuptools import find_packages, setup

package_name = 'ur3e_named_motion_gui_py'

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
    description='Qt GUI for UR3e named motion service simulation workflows.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'named_motion_gui = ur3e_named_motion_gui_py.main:main',
        ],
    },
)
