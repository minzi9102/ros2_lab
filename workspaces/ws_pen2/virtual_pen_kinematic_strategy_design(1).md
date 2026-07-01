# 虚拟笔运动策略技术原理方案

## 1. 文档目的

本文档描述一套新的虚拟笔运动策略，用于 UR3e 写字控制中的目标生成与机械臂速度控制前馈。

当前虚拟笔不是物理仿真模型，没有质量、弹簧、阻尼，也不使用 PID（比例-积分-微分控制）。它本质上是一个运动学参考生成器：根据摇杆输入生成笔尖位置、笔尖速度、笔杆姿态、笔杆角速度，以及可供机械臂控制器使用的 tool0 目标位姿和速度。

本文档的目标是把当前策略改造成一套更自然、更可预测、更适合机械臂实时跟踪的运动学方案。

---

## 2. 当前问题抽象

当前控制链可以概括为：

```text
摇杆输入
→ 虚拟笔目标
→ MoveIt Servo
→ commanded joints
→ URSim / 真机 actual joints
```

其中 MoveIt Servo 是 MoveIt 的实时伺服控制模块，下文简称 Servo。commanded joints 指 Servo 输出的关节命令，actual joints 指 URSim 或真机实际执行到的关节状态。

前期实验已经表明：

```text
target pose → commanded-joint FK 误差较大
commanded joints → actual joints 误差很小
```

这里 FK 是 Forward Kinematics，即前向运动学，表示由关节角计算末端位姿。

因此，主要误差不来自 UR 执行层，而来自：

```text
虚拟笔目标 → Servo → commanded joints
```

进一步实验表明：

- 固定竖直姿态时，平面位置跟踪误差很小；
- 固定笔尖 XY 但动态改变姿态时，位置误差明显增大；
- 姿态误差会通过笔长偏置放大成笔尖位置误差。

因此问题可以抽象为：

```text
带工具偏置的动态姿态任务
→ 造成 tool0 目标位姿快速变化
→ Servo 对完整 6 维位姿目标产生滞后或压缩
→ 笔尖位置误差增大
```

这里 6 维位姿指 3 个位置自由度加 3 个姿态自由度。tool0 指机械臂法兰或工具坐标系。pen tip 指笔尖。

---

## 3. 新策略设计目标

新的虚拟笔策略应满足以下目标。

### 3.1 自然性

自然性指运动连续、过渡柔和、没有明显跳变。

要求：

```text
速度连续
加速度尽量连续
姿态连续
角速度连续
停止和启动没有突变
```

### 3.2 可预测性

可预测性指相同输入得到相同输出，所有状态变化都由明确的运动学规则产生。

要求：

```text
摇杆输入相同 → 虚拟笔轨迹相同
参数固定 → 行为可复现
速度、姿态、状态切换都有明确边界
```

### 3.3 机械臂友好

机械臂友好指输出不仅包含目标位姿，还包含速度前馈，使机械臂不必只通过追逐连续位姿点来推断速度。

要求每帧输出：

```text
笔尖位置
笔尖线速度
笔尖线加速度
笔杆姿态
笔杆角速度
笔杆角加速度
tool0 目标位置
tool0 目标姿态
tool0 线速度
tool0 角速度
```

### 3.4 笔尖位置优先

写字任务的核心是笔尖轨迹，而不是 tool0 的完整 6 维位姿。

因此后续机械臂控制应优先满足：

```text
笔尖位置跟踪
```

其次再满足：

```text
笔轴方向跟踪
tool0 姿态细节
关节运动平滑
```

---

## 4. 总体架构

新策略分为 5 层：

```text
1. 摇杆意图层
2. 平面速度规划层
3. 笔尖状态层
4. 姿态规划层
5. 输出映射层
```

整体数据流为：

```text
摇杆输入
→ 平面意图速度
→ 速度 / 加速度 / 加加速度受限的笔尖速度
→ 笔尖位置积分
→ 方向可信度
→ 连续 yaw 与连续 tilt
→ 目标笔轴方向
→ 连续四元数姿态
→ 角速度与角加速度
→ tool0 位姿与速度
→ 机械臂控制器
```

其中 yaw 指平面航向角，表示笔尾在平面内朝向哪个方向。tilt 指笔杆倾角，表示笔杆偏离竖直方向的角度。

---

## 5. 摇杆意图层

### 5.1 输入

摇杆输入为二维向量：

```text
joy = [jx, jy]
```

### 5.2 死区处理

摇杆中心附近容易有噪声，因此需要死区。

```text
joy_norm = sqrt(jx² + jy²)
```

如果：

```text
joy_norm < deadzone
```

则认为没有运动意图：

```text
intent_strength = 0
intent_direction = previous_direction
```

如果超过死区，则归一化方向：

```text
intent_direction = [jx, jy] / joy_norm
```

### 5.3 意图速度

有两种模式。

#### 模式 A：固定速度模式

该模式保持当前写字策略，即摇杆只控制方向，不控制速度大小。

```text
v_desired = intent_direction × max_speed
```

当没有运动意图时：

```text
v_desired = [0, 0]
```

#### 模式 B：比例速度模式

该模式让摇杆幅值控制速度大小。

```text
v_desired = intent_direction × max_speed × intent_strength
```

当前项目若希望保持虚拟笔动态速度不变，建议先使用模式 A。

---

## 6. 平面速度规划层

当前策略已有速度和加速度限制。新方案建议在此基础上加入加加速度限制。

加加速度指加速度变化的速度。加入该限制后，速度曲线会更平滑，机械臂跟踪更友好。

### 6.1 状态量

平面速度规划层维护：

```text
v_xy: 当前平面速度
acc_xy: 当前平面加速度
```

### 6.2 限制项

```text
max_speed_mps       最大速度
max_accel_mps2      最大加速度
max_decel_mps2      最大减速度
max_jerk_mps3       最大加加速度
```

### 6.3 更新逻辑

先计算期望加速度：

```text
a_desired = (v_desired - v_xy) / dt
```

再根据加速或减速选择上限：

```text
if dot(v_desired - v_xy, v_xy) >= 0:
    a_limit = max_accel_mps2
else:
    a_limit = max_decel_mps2
```

对期望加速度限幅：

```text
a_target = limit_norm(a_desired, a_limit)
```

再限制加速度变化：

```text
acc_xy = move_towards_vector(
    acc_xy,
    a_target,
    max_jerk_mps3 × dt
)
```

最后积分速度：

```text
v_xy = v_xy + acc_xy × dt
v_xy = limit_norm(v_xy, max_speed_mps)
```

输出：

```text
tip_velocity = [vx, vy, 0]
tip_acceleration = [ax, ay, 0]
```

---

## 7. 笔尖状态层

笔尖状态层只负责积分位置，不处理姿态。

```text
tip_position = tip_position + tip_velocity × dt
```

重要约束：

```text
姿态层不能反向修改笔尖位置
```

这样可以保证：

```text
摇杆 → 平面速度 → 笔尖位置
```

这条链路简单、可预测。

---

## 8. 姿态规划层

姿态规划层负责从平面速度生成三维笔杆姿态。

新策略不再使用硬阈值切换，而是使用连续权重。

当前硬阈值逻辑类似：

```text
速度低于阈值：保持 yaw
速度高于阈值：更新 yaw
速度低于阈值：不倾斜
速度高于阈值：开始倾斜
```

新逻辑改成：

```text
速度越低 → 越不相信运动方向
速度越高 → 越相信运动方向
速度越低 → 倾角越接近 0
速度越高 → 倾角越接近最大倾角
```

---

## 9. 方向可信度

### 9.1 定义

方向可信度表示当前平面速度方向是否足够稳定，是否可以用于更新 yaw。

```text
direction_confidence ∈ [0, 1]
```

含义：

```text
0：完全不相信当前速度方向
1：完全相信当前速度方向
```

### 9.2 平滑阶跃函数

使用 smoothstep 函数。smoothstep 是一种从 0 平滑过渡到 1 的函数。

```text
smoothstep(edge0, edge1, x):
    t = clamp((x - edge0) / (edge1 - edge0), 0, 1)
    return t × t × (3 - 2 × t)
```

方向可信度为：

```text
direction_confidence = smoothstep(
    confidence_speed_low_mps,
    confidence_speed_high_mps,
    speed
)
```

其中：

```text
speed = sqrt(vx² + vy²)
```

### 9.3 yaw 目标

如果速度不为零，则原始 yaw 为：

```text
yaw_raw = atan2(vy, vx)
```

然后用方向可信度混合上一帧 yaw 和当前速度方向：

```text
yaw_target = blend_angle(yaw_previous, yaw_raw, direction_confidence)
```

blend_angle 表示考虑角度环绕的角度插值。

例如从 179° 到 -179°，应走 2°，而不是走 358°。

---

## 10. 倾角规划

### 10.1 连续倾角目标

当前策略中，速度超过阈值后才开始倾斜。新策略改为连续映射：

```text
tilt_ratio = smoothstep(
    tilt_speed_low_mps,
    tilt_speed_high_mps,
    speed
)
```

目标倾角：

```text
tilt_target = max_tilt_rad × tilt_ratio
```

这样：

```text
速度接近 0       → tilt_target 接近 0
速度逐渐变大     → tilt_target 平滑增大
速度达到写字速度 → tilt_target 接近 max_tilt
```

### 10.2 倾角速率限制

当前倾角不能瞬间变化，需要限制倾斜和回正速率。

```text
if tilt_target > tilt_current:
    rate_limit = max_tilt_rate_radps
else:
    rate_limit = max_untilt_rate_radps
```

更新：

```text
tilt_current = move_towards_scalar(
    tilt_current,
    tilt_target,
    rate_limit × dt
)
```

### 10.3 倾角加速度限制

为了让姿态速度更平滑，建议进一步引入倾角加速度限制。

维护倾角速度：

```text
tilt_rate_current
```

先计算目标倾角速度：

```text
tilt_rate_target = clamp(
    (tilt_target - tilt_current) / dt,
    -max_untilt_rate_radps,
    max_tilt_rate_radps
)
```

再限制倾角速度变化：

```text
tilt_rate_current = move_towards_scalar(
    tilt_rate_current,
    tilt_rate_target,
    max_tilt_accel_radps2 × dt
)
```

最后积分：

```text
tilt_current = tilt_current + tilt_rate_current × dt
```

---

## 11. 目标笔轴方向

设：

```text
ψ = yaw_current
θ = tilt_current
```

如果笔轴方向定义为从笔尖指向笔尾，则世界坐标系下的目标笔轴方向为：

```text
pen_axis_target = [
    sin(θ) × cos(ψ),
    sin(θ) × sin(ψ),
    cos(θ)
]
```

当：

```text
θ = 0
```

则：

```text
pen_axis_target = [0, 0, 1]
```

表示笔竖直向上。

如果实际代码中使用的是从 tool0 指向 pen tip 的方向，则需要取反：

```text
axis_tool_to_tip = -pen_axis_tip_to_tail
```

---

## 12. 连续姿态生成

### 12.1 四元数

四元数是一种表示三维旋转的数学量，常用于避免欧拉角万向节锁问题。

新策略继续使用四元数作为最终姿态表示。

### 12.2 最小扭转姿态

笔杆主要关心笔轴方向。绕笔轴自身的旋转通常不影响写字位置，因此应使用最小扭转方式生成姿态。

目标是：

```text
让笔轴对准 pen_axis_target
同时尽量减少绕笔轴自身的额外旋转
```

### 12.3 笔轴角速度限制

当前笔轴不能瞬间转到目标方向，需要限制最大角速度。

设：

```text
a_current = 当前笔轴方向
b_target  = 目标笔轴方向
```

夹角为：

```text
angle = acos(clamp(dot(a_current, b_target), -1, 1))
```

每步最大旋转角：

```text
max_step = max_axis_angular_speed_radps × dt
```

实际旋转角：

```text
step_angle = min(angle, max_step)
```

旋转轴：

```text
rot_axis = normalize(cross(a_current, b_target))
```

如果 angle 很小，则保持当前方向。

### 12.4 笔轴角加速度限制

为了进一步平滑，建议引入笔轴角加速度限制。

维护当前笔轴角速度大小：

```text
axis_angular_speed_current
```

目标角速度大小为：

```text
axis_angular_speed_target = min(
    angle / dt,
    max_axis_angular_speed_radps
)
```

限制角速度变化：

```text
axis_angular_speed_current = move_towards_scalar(
    axis_angular_speed_current,
    axis_angular_speed_target,
    max_axis_angular_accel_radps2 × dt
)
```

实际旋转角：

```text
step_angle = axis_angular_speed_current × dt
```

---

## 13. 从姿态差分得到角速度

机械臂速度控制需要角速度。建议从最终四元数差分得到角速度，而不是分别微分 yaw 和 tilt。

设：

```text
q_prev = 上一帧姿态
q_curr = 当前帧姿态
```

相对旋转：

```text
dq = q_curr × inverse(q_prev)
```

将 dq 转换为旋转轴和旋转角：

```text
angle = 2 × atan2(norm(dq.xyz), dq.w)
axis  = dq.xyz / norm(dq.xyz)
```

角速度为：

```text
omega_world = axis × angle / dt
```

omega_world 表示世界坐标系下的角速度。

如果后端控制器需要 tool0 坐标系下的角速度，则转换为：

```text
omega_tool = R_world_tool^T × omega_world
```

其中 R_world_tool 是 tool0 姿态对应的旋转矩阵。

---

## 14. 输出映射层：从笔尖到 tool0

### 14.1 几何关系

设：

```text
p_tip  = 笔尖位置
p_tool = tool0 位置
r      = 从 tool0 指向 pen tip 的世界坐标向量
```

则：

```text
p_tip = p_tool + r
```

因此：

```text
p_tool = p_tip - r
```

如果 pen_length 是笔长，且 pen_axis 表示从笔尖指向笔尾的单位向量，则：

```text
r = -pen_axis × pen_length
```

因为从 tool0 到笔尖的方向与从笔尖到笔尾方向相反。

### 14.2 速度关系

对位置关系求导：

```text
v_tip = v_tool + omega × r
```

因此：

```text
v_tool = v_tip - omega × r
```

这是动态姿态补偿的关键公式。

含义是：

```text
即使笔尖不动，只要笔杆姿态在转，tool0 也必须移动
```

例如固定笔尖 XY、动态改变姿态时：

```text
v_tip ≈ 0
omega ≠ 0
v_tool = -omega × r
```

这正是当前固定笔尖 XY 测试中容易产生位置误差的原因。

### 14.3 输出 tool0 速度

```text
tool0_linear_velocity = tip_linear_velocity - cross(pen_angular_velocity, r)
tool0_angular_velocity = pen_angular_velocity
```

---

## 15. 运动状态机

新策略建议使用明确状态机。

状态包括：

```text
IDLE       空闲
MOVING     运动中
HOLDING    短暂停笔保持
RETURNING  回正中
```

### 15.1 IDLE：空闲

条件：

```text
长时间无摇杆输入
平面速度接近 0
姿态已回正
```

行为：

```text
笔尖不动
笔杆竖直
角速度为 0
```

### 15.2 MOVING：运动中

条件：

```text
存在摇杆运动意图
或平面速度仍未降到 0
```

行为：

```text
笔尖按规划速度运动
姿态根据方向可信度和速度连续倾斜
```

### 15.3 HOLDING：短暂停笔保持

条件：

```text
摇杆输入消失
平面速度接近 0
但停止时间小于 hold_time_sec
```

行为：

```text
笔尖停止
姿态保持最后写字姿态
不立即回正
```

目的：

```text
避免写字短暂停顿时笔杆反复回正和重新倾斜
```

### 15.4 RETURNING：回正中

条件：

```text
停止时间超过 hold_time_sec
```

行为：

```text
笔尖保持不动
倾角逐渐回到 0
可选择保持 yaw 或慢慢回到默认 yaw
```

### 15.5 状态机原则

状态可以离散切换，但输出不能跳变。

```text
状态可以跳
位置不能跳
速度不能跳
姿态不能跳
角速度不能跳
```

所有状态输出都必须经过连续限速器。

---

## 16. 推荐数据结构

建议定义统一虚拟笔运动学状态。

```python
@dataclass
class VirtualPenKinematicState:
    time_sec: float

    # 笔尖状态
    tip_position_world: np.ndarray
    tip_velocity_world: np.ndarray
    tip_acceleration_world: np.ndarray

    # 姿态状态
    orientation_world: np.ndarray       # quaternion [x, y, z, w]
    angular_velocity_world: np.ndarray
    angular_acceleration_world: np.ndarray

    # 中间语义量
    planar_speed_mps: float
    direction_confidence: float
    yaw_rad: float
    tilt_rad: float
    motion_phase: str

    # tool0 输出
    tool0_position_world: np.ndarray
    tool0_orientation_world: np.ndarray
    tool0_linear_velocity_world: np.ndarray
    tool0_angular_velocity_world: np.ndarray
```

---

## 17. 核心伪代码

```python
def update_virtual_pen(dt, joystick):
    # 1. 摇杆意图
    intent = compute_joystick_intent(joystick)

    # 2. 平面速度规划
    v_desired_xy = compute_desired_planar_velocity(intent)
    v_xy, acc_xy = update_planar_velocity_with_accel_and_jerk_limits(
        v_current_xy,
        acc_current_xy,
        v_desired_xy,
        dt,
    )

    # 3. 笔尖位置积分
    tip_velocity_world = np.array([v_xy[0], v_xy[1], 0.0])
    tip_acceleration_world = np.array([acc_xy[0], acc_xy[1], 0.0])
    tip_position_world += tip_velocity_world * dt

    # 4. 状态机更新
    motion_phase = update_motion_phase(intent, v_xy, dt)

    # 5. 方向可信度
    speed = norm(v_xy)
    direction_confidence = smoothstep(
        confidence_speed_low_mps,
        confidence_speed_high_mps,
        speed,
    )

    # 6. yaw 目标
    if speed > epsilon:
        yaw_raw = atan2(v_xy[1], v_xy[0])
    else:
        yaw_raw = yaw_current

    yaw_target = blend_angle(yaw_current, yaw_raw, direction_confidence)
    yaw_current = update_yaw_with_rate_and_accel_limits(yaw_current, yaw_target, dt)

    # 7. tilt 目标
    if motion_phase in ["MOVING"]:
        tilt_ratio = smoothstep(tilt_speed_low_mps, tilt_speed_high_mps, speed)
        tilt_target = max_tilt_rad * tilt_ratio
    elif motion_phase == "HOLDING":
        tilt_target = tilt_current
    else:
        tilt_target = 0.0

    tilt_current = update_tilt_with_rate_and_accel_limits(
        tilt_current,
        tilt_target,
        dt,
    )

    # 8. 目标笔轴方向
    pen_axis_target = compute_pen_axis_from_yaw_and_tilt(yaw_current, tilt_current)

    # 9. 连续姿态更新
    q_prev = q_current
    q_current = update_orientation_minimum_twist_with_axis_limits(
        q_current,
        pen_axis_target,
        dt,
    )

    # 10. 四元数差分得到角速度
    angular_velocity_world = angular_velocity_from_quaternion_delta(
        q_prev,
        q_current,
        dt,
    )

    angular_acceleration_world = (
        angular_velocity_world - previous_angular_velocity_world
    ) / dt

    # 11. tool0 映射
    r_tool_to_tip_world = compute_tool_to_tip_vector(q_current, pen_length_m)
    tool0_position_world = tip_position_world - r_tool_to_tip_world
    tool0_orientation_world = q_current

    tool0_linear_velocity_world = (
        tip_velocity_world
        - cross(angular_velocity_world, r_tool_to_tip_world)
    )
    tool0_angular_velocity_world = angular_velocity_world

    return VirtualPenKinematicState(...)
```

---

## 18. 推荐参数

建议第一版使用以下参数。

```yaml
virtual_pen:
  update_rate_hz: 125.0

  planar:
    max_speed_mps: 0.03
    max_accel_mps2: 0.08
    max_decel_mps2: 0.16
    max_jerk_mps3: 0.80

  joystick:
    deadzone: 0.08
    mode: fixed_speed

  direction:
    confidence_speed_low_mps: 0.003
    confidence_speed_high_mps: 0.015
    max_yaw_rate_degps: 30.0
    max_yaw_accel_degps2: 120.0

  tilt:
    max_tilt_deg: 20.0
    speed_low_mps: 0.003
    speed_high_mps: 0.020
    max_tilt_rate_degps: 12.0
    max_untilt_rate_degps: 12.0
    max_tilt_accel_degps2: 80.0

  orientation:
    max_axis_angular_speed_degps: 12.0
    max_axis_angular_accel_degps2: 80.0
    minimum_twist: true

  stop:
    hold_time_sec: 0.30
    return_time_sec: 1.00
```

说明：

- `update_rate_hz` 建议从 60 Hz 提高到 125 Hz；
- `max_speed_mps` 保持 0.03，不降低虚拟笔动态速度；
- `confidence_speed_low_mps` 和 `confidence_speed_high_mps` 代替原来的低速保 yaw 硬阈值；
- `tilt.speed_low_mps` 和 `tilt.speed_high_mps` 代替原来的开始倾斜硬阈值；
- `hold_time_sec` 用于短暂停笔保持姿态。

---

## 19. 与当前代码的对应关系

当前代码中的职责可以迁移如下。

### 19.1 `SmoothPlanarVelocity`

当前职责：

```text
摇杆方向 → 固定最大速度 → 加速度/减速度限幅
```

建议改为：

```text
摇杆方向 → 目标速度 → 速度/加速度/加加速度限幅
```

新增输出：

```text
tip_velocity
tip_acceleration
```

### 19.2 `VirtualPenState`

当前职责：

```text
tip += v × dt
低速保 yaw
速度超过阈值后倾斜
停止后回正
```

建议改为：

```text
tip += v × dt
方向可信度生成 yaw
速度连续映射 tilt
状态机管理 MOVING / HOLDING / RETURNING
```

新增输出：

```text
direction_confidence
yaw
tilt
motion_phase
```

### 19.3 `ContinuousPenOrientation`

当前职责：

```text
限制笔轴最大角速度
生成连续四元数
最小扭转
```

建议保留，并扩展：

```text
增加角加速度限制
输出 angular_velocity
输出 angular_acceleration
```

### 19.4 `pen_fakehardware_servo_node.py`

当前职责：

```text
虚拟笔 pose → tool0 target pose → Servo pose command
```

建议新增：

```text
虚拟笔 state → tool0 target pose + tool0 target twist
```

并逐步支持：

```text
tool0 velocity control
pen-tip priority velocity control
```

---

## 20. 输出接口建议

建议新增遥测话题。

```text
/pen_writing/virtual_pen/state
/pen_writing/virtual_pen/tip_pose
/pen_writing/virtual_pen/tip_twist
/pen_writing/virtual_pen/tool0_pose
/pen_writing/virtual_pen/tool0_twist
```

其中 twist 指线速度和角速度组成的速度消息。

### 20.1 `tip_twist`

```text
linear  = tip_velocity_world
angular = angular_velocity_world
```

### 20.2 `tool0_twist`

```text
linear  = tool0_linear_velocity_world
angular = tool0_angular_velocity_world
```

### 20.3 调试信息

建议额外输出：

```text
direction_confidence
tilt_rad
yaw_rad
motion_phase
planar_speed_mps
axis_angular_speed_radps
```

---

## 21. 与机械臂控制器的关系

新虚拟笔策略本身不直接控制机械臂关节。它输出更完整的运动学参考。

后续机械臂可以采用三种模式。

### 21.1 继续使用 Servo pose 模式

```text
tool0 target pose → Servo pose command
```

优点：

```text
改动小
可直接对比旧策略
```

缺点：

```text
速度前馈没有充分利用
动态姿态仍可能滞后
```

### 21.2 Servo twist 模式

```text
tool0 target twist + pose error feedback → Servo twist command
```

优点：

```text
开始利用速度前馈
实现量中等
```

缺点：

```text
仍可能出现 6 维 twist 被统一缩放
```

### 21.3 pen-tip priority 关节速度控制

```text
pen tip 主任务
+ pen axis 副任务
→ 速度级逆运动学
→ joint velocity command
```

优点：

```text
最符合写字任务
能避免姿态任务拖累笔尖位置
```

缺点：

```text
实现复杂
需要明确关节速度、加速度、安全限幅
```

推荐路线：

```text
先完成新虚拟笔状态输出
再尝试 tool0 twist 前馈
最后实现 pen-tip priority 关节速度控制
```

---

## 22. 验证方案

### 22.1 单元测试

需要测试：

```text
smoothstep 输出连续且范围为 [0, 1]
blend_angle 处理 ±π 环绕正确
平面速度不超过 max_speed
平面加速度不超过 max_accel / max_decel
平面加加速度不超过 max_jerk
倾角不超过 max_tilt
角速度不超过 max_axis_angular_speed
角加速度不超过 max_axis_angular_accel
tool0 速度公式与位姿差分一致
```

### 22.2 离线仿真测试

输入典型摇杆轨迹：

```text
直线启动
直线停止
90 度转向
圆形摇杆输入
短暂停顿后继续
快速反向
低速小抖动
```

检查输出：

```text
笔尖轨迹
笔尖速度曲线
倾角曲线
yaw 曲线
角速度曲线
tool0 速度曲线
```

### 22.3 URSim 测试

建议保留当前两组关键测试：

```text
固定竖直姿态 + 动态 XY
固定笔尖 XY + 动态姿态
```

再增加：

```text
动态 XY + 动态姿态
短暂停笔后继续
连续转向写字轨迹
```

### 22.4 指标

关键指标：

```text
target→commanded FK 位置 RMS
target→commanded FK 姿态 RMS
commanded→actual FK 位置 RMS
path length ratio
最大角速度
最大关节速度
最大关节加速度
笔尖位置误差
姿态误差折算笔尖误差
```

建议新增指标：

```text
orientation_induced_tip_error = pen_length × sin(orientation_error)
```

用于评估姿态误差通过笔长造成的位置误差。

---

## 23. 预期效果

### 23.1 虚拟笔输出更自然

预期：

```text
低速时 yaw 不抖
倾角不再突然开始
停止后不会立即回正
角速度曲线更平滑
```

### 23.2 机械臂跟踪更可预测

预期：

```text
tool0 速度可以提前计算
动态姿态造成的 tool0 补偿速度显式输出
Servo 或自定义控制器不再只靠 pose 误差追踪
```

### 23.3 位置误差更容易定位

因为新状态包含完整中间量，可以把误差拆成：

```text
虚拟笔目标生成误差
姿态规划引起的 tool0 速度需求
Servo 跟踪误差
UR 执行误差
```

---

## 24. 风险与注意事项

### 24.1 过度平滑会增加滞后

加入加加速度、角加速度限制后，运动会更自然，但如果限制太保守，会增加响应滞后。

因此应同时观察：

```text
自然性
机械臂可跟踪性
target→commanded 误差
```

### 24.2 姿态保持时间不宜过长

`HOLDING` 状态可以减少短暂停顿扰动，但保持时间太长会导致停笔后迟迟不回正。

建议初始使用：

```text
hold_time_sec = 0.30
```

### 24.3 tool0 速度公式必须统一坐标系

公式：

```text
v_tool = v_tip - omega × r
```

要求：

```text
v_tip、omega、r 必须在同一坐标系下
```

建议全部先使用世界坐标系。

### 24.4 四元数符号连续性

四元数 q 和 -q 表示同一个姿态。做差分前必须保持符号连续。

如果：

```text
dot(q_prev, q_curr) < 0
```

则：

```text
q_curr = -q_curr
```

避免角速度计算出现跳变。

---

## 25. 分阶段落地计划

### 阶段 1：只增加状态输出

目标：不改变行为，只增加遥测。

新增：

```text
tip_velocity
tip_acceleration
angular_velocity
angular_acceleration
tool0_linear_velocity
tool0_angular_velocity
motion_phase
```

目的：确认当前策略在动态姿态时产生了多大的 tool0 速度需求。

### 阶段 2：硬阈值改为连续混合

修改：

```text
yaw_update_threshold → direction_confidence smoothstep
tilt_start_threshold → tilt smoothstep
```

目的：减少低速和启动阶段突变。

### 阶段 3：增加 HOLDING 状态

修改：

```text
停止后先保持姿态
超过 hold_time_sec 后再回正
```

目的：减少短暂停笔时的姿态扰动。

### 阶段 4：加入角加速度限制

修改：

```text
姿态不仅限制角速度
还限制角加速度
```

目的：降低机械臂关节速度尖峰。

### 阶段 5：使用速度前馈

新增控制模式：

```text
tool0_twist_feedforward
```

输入：

```text
tool0_pose
tool0_twist
```

目的：减少 Servo 被动追 pose 的滞后。

### 阶段 6：pen-tip priority 控制

新增控制模式：

```text
pen_tip_priority_joint_velocity
```

目标：

```text
笔尖位置主任务
笔轴方向副任务
```

目的：从控制结构上避免姿态任务拖累笔尖位置。

---

## 26. 最终推荐方案

最终推荐把虚拟笔定义为：

```text
连续运动学参考生成器
```

它不模拟真实笔的质量、弹簧或阻尼，而是生成一组对机械臂友好的参考状态：

```text
笔尖位置
笔尖速度
笔尖加速度
笔杆姿态
笔杆角速度
笔杆角加速度
tool0 位姿
tool0 速度
```

核心规则是：

```text
平面运动：速度、加速度、加加速度受限
方向：由方向可信度连续跟随
倾角：由速度连续映射
姿态：四元数连续生成，角速度和角加速度受限
停止：短暂停笔保持，长停后回正
输出：显式提供 tool0 速度和笔尖速度
```

这套方案能保持当前虚拟笔最大动态速度 `0.03 m/s` 不变，同时让目标更平滑、更可预测，并为后续机械臂速度级控制提供必要的前馈信息。
