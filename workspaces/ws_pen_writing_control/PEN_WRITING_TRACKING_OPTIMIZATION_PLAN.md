# ws_pen_writing_control 跟随误差降低改造计划

下面按“先减少目标跑远，再提升伺服跟踪，再验证底层链路”的顺序推进。本计划默认只用于 `fake hardware` 和 RViz，不进入真机接触阶段。

依据：当前策略已经是“手柄推动虚拟笔目标，节点反解 `target tool0`，再通过 MoveIt Servo 追踪；绿色是目标，青色是当前实际状态”，并且已有“笔尖误差超过 12 mm 或轴线误差超过 8 deg 时暂停目标积分”的门控。MoveIt Servo 支持末端期望位姿控制，也会在接近奇异位形、碰撞、关节限位时缩放速度；这意味着跟随误差变大时，既可能是上游目标生成太激进，也可能是 Servo 或底层控制器主动降速。

## 0. 改造目标

当前问题：

```text
target tool0 按虚拟笔目标持续前进
current tool0 通过 Servo 追踪
动态运动时绿色目标和青色实际出现明显跟随误差
```

改造目标：

```text
正常直线运动：
  tip_error < 3-5 mm
  axis_error < 2-4 deg

方向变化或掉头：
  允许短暂误差增大
  但 target 不继续远离 current

停止摇杆：
  target 不跳变
  current 能平滑追上
```

核心原则：

```text
不是让 Servo 硬追一个越跑越远的目标
而是让虚拟目标根据 current tool0 的实际跟随能力动态放慢
```

## 1. 第一步：增加跟随误差观测与日志基线

### 1.1 目标

先确认误差来自哪里：

```text
A. 虚拟目标生成太快
B. Servo 参数限速
C. Servo 因奇异位形/碰撞/关节限位主动降速
D. 底层 ros2_control 控制器更新慢或接口不合适
```

`ros2_control` 的关节轨迹控制器至少需要关节位置反馈，速度和加速度反馈是可选项；轨迹点中是否带位置、速度、加速度会影响插值和控制表现。

### 1.2 新增记录量

在现有节点中新增调试发布或日志记录：

```text
/pen_writing/debug/follow_error
  tail_error_m
  tip_error_m
  axis_error_deg
  target_current_tool0_pos_error_m
  target_current_tool0_rot_error_deg

/pen_writing/debug/target_state
  target_pen_tip
  target_pen_axis
  target_yaw
  target_tilt
  virtual_planar_speed

/pen_writing/debug/current_state
  current_tool0
  current_pen_tip
  current_pen_axis

/pen_writing/debug/gates
  follow_scale
  lead_clamp_active
  yaw_rate_limit_active
  tilt_rate_limit_active
  dynamic_pause_active
```

同时记录：

```text
/servo_node/status
/servo_node/pose_target_cmds
Servo 输出的 joint trajectory
/joint_states
```

### 1.3 验收

先不改控制逻辑，只运行 3 组动作：

```text
1. 慢速 +X 直线 5 秒
2. 慢速圆弧 5 秒
3. 左右来回掉头 5 次
```

记录每组：

```text
tip_error 最大值 / 平均值
axis_error 最大值 / 平均值
target 是否明显跑远
Servo status 是否出现降速、奇异位形、碰撞或关节限位相关状态
```

进入下一步的条件：

```text
能稳定看到误差曲线
能确认误差峰值出现在直线、圆弧、还是掉头
```

## 2. 第二步：把“硬暂停”改成“连续减速”

### 2.1 现状

当前是：

```text
tip_error > 12 mm
或 axis_error > 8 deg
-> 暂停虚拟笔继续积分
```

这属于“目标跑远后再刹车”。

### 2.2 改造目标

改成：

```text
误差刚变大时就开始降低虚拟笔速度
误差很大时才停止积分
误差恢复后平滑恢复速度
```

### 2.3 新增参数

```yaml
follow_error_gate:
  tip_soft_m: 0.004          # 4 mm 开始降速
  tip_hard_m: 0.010          # 10 mm 停止积分
  axis_soft_deg: 3.0         # 3 deg 开始降速
  axis_hard_deg: 8.0         # 8 deg 停止积分
  scale_rise_rate: 1.5       # 每秒最多恢复多少
  scale_fall_rate: 4.0       # 每秒最多下降多少
```

### 2.4 控制逻辑

```text
tip_scale:
  tip_error <= 4 mm       -> 1.0
  4 mm 到 10 mm           -> 从 1.0 线性降到 0.0
  tip_error >= 10 mm      -> 0.0

axis_scale:
  axis_error <= 3 deg     -> 1.0
  3 deg 到 8 deg          -> 从 1.0 线性降到 0.0
  axis_error >= 8 deg     -> 0.0

follow_scale = min(tip_scale, axis_scale)
virtual_velocity = joystick_velocity * follow_scale
```

### 2.5 注意点

`follow_scale` 不要瞬间跳变，否则会出现绿色目标突然卡一下：

```text
follow_scale_raw -> rate_limit -> follow_scale
```

即：

```text
下降可以快一点
恢复必须慢一点
```

### 2.6 验收

同样做 3 组动作：

```text
慢速直线
圆弧
左右掉头
```

通过标准：

```text
target 不再明显越跑越远
dynamic_pause_active 出现次数减少
tip_error 峰值低于原来
绿色目标停顿变少，改成平滑变慢
```

## 3. 第三步：降低阶段 1 初始速度、加速度和姿态角速度

### 3.1 现状风险

当前最大平面速度是：

```text
0.08 m/s
```

但笔尖到 `tool0` 有 0.14 m 偏移。笔杆姿态变化时，`tool0` 不是只做平移，还会因为笔长产生额外摆动：

```text
tool0 负担 ≈ 笔尖平移 + 笔长引起的姿态摆动
```

### 3.2 建议先改成保守参数

```yaml
virtual_pen_motion:
  max_planar_speed_mps: 0.03
  max_planar_accel_mps2: 0.04
  max_planar_decel_mps2: 0.08

pen_orientation:
  max_yaw_rate_radps: 0.4
  max_yaw_accel_radps2: 0.8
  max_tilt_rate_radps: 0.3
  max_tilt_accel_radps2: 0.6
```

偏航角是笔身绕纸面法线旋转的角度；倾角是笔身相对纸面法线的倾斜角。

### 3.3 Servo 配置同步检查

不要让上游目标速度大于 Servo 允许速度。建议先检查：

```yaml
servo:
  publish_period: 0.01
  scale:
    linear: 0.10 - 0.20
    rotational: 0.30 - 0.50
  incoming_command_timeout: 0.1
  use_smoothing: true
```

Servo 默认启用轨迹平滑插件。默认平滑插件是 Butterworth 滤波器；Butterworth 滤波器是一种低通滤波器，用来削弱高频抖动，但平滑越强也可能引入更多滞后。

### 3.4 验收

通过标准：

```text
慢速直线 tip_error < 3 mm
慢速圆弧 axis_error < 3 deg
掉头时 target 不甩开 current
Servo status 不频繁出现安全降速
```

如果误差仍大，不要继续升速，进入下一步。

## 4. 第四步：增加 target-current 最大领先距离限制

### 4.1 目标

让目标位姿永远不要离当前实际状态太远。

现状可能是：

```text
target_tip = 上一帧 target_tip + virtual_velocity * dt
```

建议改成：

```text
candidate_target_tip = 上一帧 target_tip + virtual_velocity * dt
target_tip = 限制在 current_tip 附近
```

### 4.2 新增参数

```yaml
target_lead_limit:
  max_tip_lead_m: 0.005          # target pen_tip 不超过 current pen_tip 5 mm
  max_tail_lead_m: 0.008         # target tool0 或笔尾不超过 current 8 mm
  max_axis_lead_deg: 3.0         # target pen_axis 不超过 current pen_axis 3 deg
```

### 4.3 位置限制逻辑

```text
lead_vec = candidate_target_tip - current_tip
lead_norm = norm(lead_vec)

if lead_norm > max_tip_lead:
    target_tip = current_tip + lead_vec / lead_norm * max_tip_lead
else:
    target_tip = candidate_target_tip
```

### 4.4 姿态限制逻辑

```text
target_axis_raw = 根据 yaw 和 tilt 计算得到
axis_error_to_current = angle_between(target_axis_raw, current_axis)

if axis_error_to_current > max_axis_lead:
    target_axis = 从 current_axis 朝 target_axis_raw 旋转 max_axis_lead
else:
    target_axis = target_axis_raw
```

这里“从一个轴向量朝另一个轴向量旋转一点”可以先不做复杂球面插值，第一版可只限制 `yaw` 和 `tilt` 的每帧变化量。

### 4.5 验收

通过标准：

```text
即使持续大推摇杆
target_tip 也不会超过 current_tip 5 mm 以上
tip_error 曲线不再持续爬升
硬暂停次数明显减少
```

这个步骤优先级很高，因为它能从结构上避免“目标跑远”。

## 5. 第五步：把手柄输入改成非线性速度曲线

### 5.1 问题

线性摇杆映射通常不适合写字：

```text
小推摇杆：速度仍偏大，容易抖
大推摇杆：目标突然变快，误差变大
```

### 5.2 改造

从：

```text
v = max_speed * u
```

改成：

```text
v = max_speed * sign(u) * abs(u)^3
```

或者稍微温和一点：

```text
v = max_speed * sign(u) * abs(u)^2
```

其中 `u` 是经过死区处理后的摇杆输入，范围是 `-1` 到 `1`。

### 5.3 推荐参数

```yaml
joystick_mapping:
  deadzone: 0.12
  curve_power: 3.0
  min_effective_speed_mps: 0.002
```

### 5.4 验收

通过标准：

```text
小幅推摇杆时，笔尖可以慢速细控
大幅推摇杆时，仍受 follow_scale 和 lead_limit 限制
松手附近不会因为摇杆噪声导致 yaw 抖动
```

## 6. 第六步：姿态方向从“瞬时速度”改成“平滑轨迹切线”

### 6.1 现状风险

如果直接用：

```text
yaw = atan2(vy, vx) + pi
```

那么摇杆抖动会直接变成笔身抖动。`atan2` 是一种带象限判断的反正切函数，用来把二维速度向量转换成方向角。

### 6.2 改造目标

用平滑后的运动方向生成偏航角：

```text
原始 joystick vx/vy
-> 速度低通滤波
-> 方向低通滤波
-> 偏航角连续展开
-> 偏航角角速度限制
```

### 6.3 新增参数

```yaml
direction_filter:
  min_speed_for_yaw_update_mps: 0.005
  direction_lowpass_tau_s: 0.15
  hold_yaw_when_stopped: true
  unwrap_yaw: true
```

`tau_s` 是低通滤波时间常数。数值越大，方向越稳，但响应越慢。

### 6.4 控制逻辑

```text
如果 speed < 0.005 m/s：
  保持 last_yaw，不更新方向

如果 speed >= 0.005 m/s：
  move_dir_raw = normalize([vx, vy])
  move_dir_filtered = 低通滤波(move_dir_raw)
  move_yaw = atan2(move_dir_filtered_y, move_dir_filtered_x)
  target_yaw = move_yaw + pi
  target_yaw = 角度连续展开
  target_yaw = 按 max_yaw_rate 限速
```

### 6.5 验收

通过标准：

```text
停止摇杆后，笔身不来回摆
慢速圆弧时，笔身连续旋转
直角折线时，笔身转向受限，不突然甩动
180° 掉头时，不发生 +179° 到 -179° 的跳变
```

## 7. 第七步：平移和转向分配速度预算

### 7.1 问题

如果同时发生：

```text
笔尖高速移动
笔身快速偏航
倾角快速变化
```

`tool0` 实际负担会明显增加，跟随误差也会变大。

### 7.2 改造策略

根据姿态误差自动降低平移速度：

```text
yaw_error 越大
平面速度越小
```

### 7.3 新增参数

```yaml
orientation_priority:
  yaw_error_slowdown_start_deg: 10.0
  yaw_error_stop_translation_deg: 45.0
  tilt_error_slowdown_start_deg: 5.0
  tilt_error_stop_translation_deg: 20.0
```

### 7.4 控制逻辑

```text
yaw_speed_scale:
  yaw_error < 10 deg       -> 1.0
  10 deg 到 45 deg         -> 从 1.0 降到 0.0
  yaw_error > 45 deg       -> 0.0

final_speed_scale = follow_scale * yaw_speed_scale * tilt_speed_scale
```

### 7.5 掉头规则

第一版建议：

```text
角度变化 < 30 deg：
  正常连续写

30 deg 到 90 deg：
  降低平移速度，优先让笔身方向跟上

超过 90 deg：
  暂停 pen_tip 平移
  原地完成姿态转向
  再恢复平移
```

真机接触阶段再考虑：

```text
抬笔 -> 转向 -> 落笔
```

fake hardware 阶段只需要：

```text
停住笔尖 -> 转笔身 -> 继续
```

### 7.6 验收

通过标准：

```text
快速改变摇杆方向时，target tool0 不突然飞出
笔尖运动速度会自动降低
axis_error 不再持续超过 8 deg
```

## 8. 第八步：检查 Servo 输入输出参数和话题频率

### 8.1 目标

确认 Servo 本身没有因为参数或频率问题造成额外滞后。

### 8.2 检查项

```text
1. /servo_node/pose_target_cmds 发布频率是否稳定
2. 发布频率是否接近 50-100 Hz
3. pose header 时间戳是否正确
4. command frame 是否与预期一致
5. Servo 输出是否真的进入目标 joint trajectory controller
6. /joint_states 是否稳定更新
7. /servo_node/status 是否出现 safety 降速
```

### 8.3 参数建议

先保守：

```yaml
servo:
  publish_period: 0.01
  scale:
    linear: 0.10
    rotational: 0.30
  incoming_command_timeout: 0.1
  use_smoothing: true
```

如果目标已经被前几步限制住，但 current 仍明显滞后，再逐步尝试：

```yaml
servo:
  scale:
    linear: 0.15
    rotational: 0.40
```

不建议一开始直接提高 Servo 速度，因为那可能掩盖上游目标生成问题。

## 9. 第九步：检查底层 ros2_control 控制器

### 9.1 目标

确认 Servo 输出的轨迹被底层控制器按预期执行。

### 9.2 检查项

```text
controller 是否是当前激活状态
Servo command_out_topic 是否连接到正确控制器
控制器更新频率是否足够
joint trajectory 点是否包含 position
是否包含 velocity
控制器硬件接口是 position、velocity，还是两者都有
```

关节轨迹控制器文档说明，轨迹可含位置、速度、加速度，不同输入会对应不同插值行为；如果只给位置，插值和动态响应会与给位置加速度或位置速度时不同。

### 9.3 验收

通过标准：

```text
Servo 输出轨迹频率稳定
joint_states 跟随轨迹没有明显阶跃或卡顿
current tool0 滞后不随时间累积
```

如果底层控制器本身更新慢或只接收位置命令，跟随误差很难只靠上游逻辑完全消除。

## 10. 推荐实施顺序

### 改造批次 1：只改上游目标生成

范围：

```text
连续减速 follow_scale
降低速度/加速度/yaw rate/tilt rate
target-current 最大领先距离
```

不改：

```text
Servo 参数
控制器配置
真机 launch
接触逻辑
```

验收：

```text
直线 tip_error < 5 mm
圆弧 axis_error < 4 deg
硬暂停次数减少 50% 以上
```

### 改造批次 2：改输入和姿态方向

范围：

```text
摇杆三次曲线
方向低通滤波
速度低于阈值时保持 yaw
角度连续展开
掉头时先降速/停笔尖
```

验收：

```text
停止时姿态不抖
圆弧时姿态连续
180° 掉头无角度跳变
```

### 改造批次 3：核对 Servo 和控制器

范围：

```text
Servo 频率
Servo status
scale.linear / scale.rotational
use_smoothing
command_out_topic
joint trajectory controller 接口
```

验收：

```text
误差峰值能解释
没有无来源的突然降速
Servo 输出和 current tool0 之间的延迟稳定
```

## 11. 建议配置初稿

```yaml
pen_writing_tracking:
  control_rate_hz: 100.0

  joystick_mapping:
    deadzone: 0.12
    curve_power: 3.0

  virtual_pen_motion:
    max_planar_speed_mps: 0.03
    max_planar_accel_mps2: 0.04
    max_planar_decel_mps2: 0.08

  pen_orientation:
    max_yaw_rate_radps: 0.4
    max_yaw_accel_radps2: 0.8
    max_tilt_rate_radps: 0.3
    max_tilt_accel_radps2: 0.6
    min_speed_for_yaw_update_mps: 0.005
    direction_lowpass_tau_s: 0.15

  follow_error_gate:
    tip_soft_m: 0.004
    tip_hard_m: 0.010
    axis_soft_deg: 3.0
    axis_hard_deg: 8.0
    scale_rise_rate: 1.5
    scale_fall_rate: 4.0

  target_lead_limit:
    max_tip_lead_m: 0.005
    max_tail_lead_m: 0.008
    max_axis_lead_deg: 3.0

  orientation_priority:
    yaw_error_slowdown_start_deg: 10.0
    yaw_error_stop_translation_deg: 45.0
    tilt_error_slowdown_start_deg: 5.0
    tilt_error_stop_translation_deg: 20.0
```

## 12. 最终验收用例

| 用例 | 操作 | 目标现象 | 通过标准 |
| --- | --- | --- | --- |
| 直线 | 左摇杆稳定推向 +X | 绿色和青色接近，笔杆方向稳定 | tip_error < 5 mm |
| 慢圆 | 左摇杆慢速画圆 | 笔杆连续旋转 | axis_error < 4 deg |
| 松手 | 突然松开摇杆 | 笔尖停住，姿态不跳 | 无明显 yaw 抖动 |
| 直角 | +X 后切到 +Y | 目标减速，笔身转向 | 无绿色甩开 |
| 掉头 | +X 后切到 -X | 笔尖可短暂停，姿态先转向 | 无 +/-pi 跳变 |
| 边界 | 推向纸面边界外 | target 被限幅 | 不越界 |
| 高误差 | 人为降低 Servo 速度 | follow_scale 自动下降 | target 不继续跑远 |

## 13. 计划结论

最推荐的改造主线是：

```text
第一优先级：
  连续减速 follow_scale
  target-current 最大领先距离
  降低速度和姿态角速度

第二优先级：
  摇杆非线性速度曲线
  方向低通滤波
  大角度转向时先降速或停笔尖

第三优先级：
  检查 Servo 参数、status 和底层 controller
```

这条路线比单纯放宽误差阈值更安全。它会把系统从：

```text
虚拟目标自由跑，Servo 被动追
```

改成：

```text
虚拟目标根据 current tool0 的实际能力前进
```

这更适合后续从 fake hardware 过渡到 URSim，再进入真机空中写字验证。

## References

- [Realtime Servo - MoveIt Documentation - PickNik Robotics](https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html)
- [joint_trajectory_controller - ROS2_Control documentation](https://control.ros.org/galactic/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html)
- [MoveIt Servo parameters](https://github.com/ros-planning/moveit2/blob/main/moveit_ros/moveit_servo/config/servo_parameters.yaml)
- [Trajectory Representation - ROS2_Control documentation](https://control.ros.org/rolling/doc/ros2_controllers/joint_trajectory_controller/doc/trajectory.html)
