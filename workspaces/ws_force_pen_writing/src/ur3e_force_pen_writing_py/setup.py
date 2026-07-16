from glob import glob

from setuptools import find_packages, setup


package_name = "ur3e_force_pen_writing_py"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="minzi",
    maintainer_email="chenmj75@mail2.sysu.edu.cn",
    description="Force-controlled paper detection and pen writing for UR3e.",
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "force_mode_validation_node = "
            "ur3e_force_pen_writing_py.force_mode_validation_node:main",
            "paper_seek_servo_node = "
            "ur3e_force_pen_writing_py.paper_seek_servo_node:main",
            "z_compliance_validation_node = "
            "ur3e_force_pen_writing_py.z_compliance_validation_node:main",
        ]
    },
)
