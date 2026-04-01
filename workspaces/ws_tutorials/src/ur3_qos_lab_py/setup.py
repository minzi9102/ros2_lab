from glob import glob

from setuptools import find_packages, setup

package_name = "ur3_qos_lab_py"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="minzi",
    maintainer_email="chenmj75@mail2.sysu.edu.cn",
    description="Minimal QoS lab package for ROS 2 UR3 learning.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "qos_publisher_node = ur3_qos_lab_py.qos_publisher_node:main",
            "qos_subscriber_node = ur3_qos_lab_py.qos_subscriber_node:main",
        ],
    },
)
