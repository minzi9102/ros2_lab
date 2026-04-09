import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/minzi/ros2_lab/install/ur3_joint_trajectory_controller_lab_py'
