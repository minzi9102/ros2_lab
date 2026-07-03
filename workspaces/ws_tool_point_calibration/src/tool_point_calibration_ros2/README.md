# tool_point_calibration_ros2

ROS 2 Jazzy console tool for UR3e TCP touch-point calibration.

This package only samples TF and computes the TCP offset. Start the existing
UR3e driver or robot state publisher first.

Check the frame names before calibration:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run tf2_ros tf2_echo base tool0
```

Run calibration:

```bash
cd /home/minzi/ros2_lab/workspaces/ws_tool_point_calibration
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch tool_point_calibration_ros2 console_calibration.launch.py \
  base_frame:=base \
  tool0_frame:=tool0 \
  num_samples:=4
```

Move the tool tip to the same fixed point from different orientations. Press
Enter once per pose. The result prints the TCP offset in `tool0_frame` and the
shared touch point in `base_frame`, both in meters.
