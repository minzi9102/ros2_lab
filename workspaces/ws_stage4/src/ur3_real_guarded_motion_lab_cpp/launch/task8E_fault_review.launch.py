from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    dry_run_guarded_motion = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ur3_real_guarded_motion_lab_cpp"),
                    "launch",
                    "task8D_guarded_home_ready.launch.py",
                ]
            )
        ),
        launch_arguments={
            "execute": "false",
            "target_name": "out_of_range_test",
        }.items(),
    )

    return LaunchDescription([dry_run_guarded_motion])
