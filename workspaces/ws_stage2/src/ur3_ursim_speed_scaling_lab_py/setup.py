import os
from glob import glob

from setuptools import find_packages, setup

package_name = "ur3_ursim_speed_scaling_lab_py"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob(os.path.join("launch", "*.py"))),
        (os.path.join("share", package_name, "config"), glob(os.path.join("config", "*.yaml"))),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="minzi",
    maintainer_email="chenmj75@mail2.sysu.edu.cn",
    description="Task 6B URSim speed scaling practice package for UR3 stage 2 learning.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "speed_scaling_monitor = ur3_ursim_speed_scaling_lab_py.speed_scaling_monitor:main",
            "scaled_trajectory_runner = ur3_ursim_speed_scaling_lab_py.scaled_trajectory_runner:main",
        ],
    },
)
