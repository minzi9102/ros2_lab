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

Verify the calibrated TCP:

```bash
ros2 launch tool_point_calibration_ros2 tcp_verification.launch.py
```

In another terminal, keep the tool tip touching the same fixed point while
changing robot posture:

```bash
source /opt/ros/jazzy/setup.bash
source /home/minzi/ros2_lab/workspaces/ws_tool_point_calibration/install/setup.bash
ros2 run tool_point_calibration_ros2 watch_calibrated_tcp.py
```

The watcher prints `base -> calibrated_tcp` and drift from the first sample.
Less than 1 mm is good, 1-2 mm is usable, and more than 3 mm usually means the
touch point or sampled TCP needs another pass.

Calibrate the paper plane after publishing the calibrated TCP:

```bash
ros2 launch tool_point_calibration_ros2 console_paper_calibration.launch.py \
  base_frame:=base \
  tcp_frame:=calibrated_tcp \
  num_samples:=4
```

Touch different points on the paper with the calibrated TCP and press Enter
once per point. The result prints the paper center in `base_frame`, a unit
normal pointing toward the `+Z` half-space, the plane equation
`normal.dot(p - center) = 0`, and average/max point-to-plane residuals in
meters.
