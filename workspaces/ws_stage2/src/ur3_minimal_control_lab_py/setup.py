import os
from glob import glob

from setuptools import find_packages, setup

package_name = "ur3_minimal_control_lab_py"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob(os.path.join("launch", "*.py"))),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="minzi",
    maintainer_email="chenmj75@mail2.sysu.edu.cn",
    description="Task 5C minimal control loop package for UR3 stage 2 practice.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "joint_state_observer = ur3_minimal_control_lab_py.joint_state_observer:main",
            "joint_trajectory_sender = ur3_minimal_control_lab_py.joint_trajectory_sender:main",
        ],
    },
)
