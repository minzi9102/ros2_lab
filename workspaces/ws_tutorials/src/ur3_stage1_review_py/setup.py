from glob import glob

from setuptools import find_packages, setup

package_name = "ur3_stage1_review_py"

setup(
    name=package_name,
    version="0.0.0",
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
    description="Stage 1 review package: demonstrates package creation, Action rationale, and UR3 description parsing.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "demo_package_build = ur3_stage1_review_py.demo_package_build:main",
            "demo_action_rationale = ur3_stage1_review_py.demo_action_rationale:main",
            "demo_description_reader = ur3_stage1_review_py.demo_description_reader:main",
        ],
    },
)
