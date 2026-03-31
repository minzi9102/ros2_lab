from setuptools import find_packages, setup

package_name = "ur3_follow_joint_trajectory_client_py"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="minzi",
    maintainer_email="chenmj75@mail2.sysu.edu.cn",
    description="Minimal FollowJointTrajectory action client in Python for UR3 learning.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "follow_joint_trajectory_client_node = ur3_follow_joint_trajectory_client_py.follow_joint_trajectory_client_node:main",
        ],
    },
)
