from glob import glob

from setuptools import find_packages, setup

package_name = "ur3e_pen_writing_control_py"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="minzi",
    maintainer_email="chenmj75@mail2.sysu.edu.cn",
    description="Stage 1 virtual pen writing visualization for UR3e practice.",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "pen_writing_visualizer_node = "
            "ur3e_pen_writing_control_py.pen_writing_visualizer_node:main",
            "pen_fakehardware_servo_node = "
            "ur3e_pen_writing_control_py.pen_fakehardware_servo_node:main",
            "pen_tracking_benchmark_node = "
            "ur3e_pen_writing_control_py.pen_tracking_benchmark_node:main",
            "pen_real_tracking_benchmark_node = "
            "ur3e_pen_writing_control_py.pen_real_tracking_benchmark_node:main",
            "command_latency_report = "
            "ur3e_pen_writing_control_py.command_latency_report:main",
            "chain_split_fk_report = "
            "ur3e_pen_writing_control_py.chain_split_fk_report:main",
            "controller_switch_once_node = "
            "ur3e_pen_writing_control_py.controller_switch_once_node:main",
            "constant_twist_diagnostic_node = "
            "ur3e_pen_writing_control_py.constant_twist_diagnostic_node:main",
            "speedl_benchmark_node = "
            "ur3e_pen_writing_control_py.speedl_benchmark_node:main",
            "pen_tip_plane_monitor_node = "
            "ur3e_pen_writing_control_py.pen_tip_plane_monitor_node:main",
            "force_mode_validation_node = "
            "ur3e_pen_writing_control_py.force_mode_validation_node:main",
            "z_compliance_validation_node = "
            "ur3e_pen_writing_control_py.z_compliance_validation_node:main",
        ],
    },
)
