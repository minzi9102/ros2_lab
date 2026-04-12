# Task 5B：理解控制器体系

## 状态
- `[#] 进行中`
- Day 1 实验完成：2026-04-12

---

## Day 1 实验记录：speed scaling 与控制器切换

### 实验环境
- `ur_robot_driver` 版本：3.7.0
- 模式：mock hardware（`use_mock_hardware:=true`）

---

### 实验 1：speed scaling 话题的实际名称

**规划文档写的**：`/speed_scaling_state`
**实际话题名**：`/speed_scaling_state_broadcaster/speed_scaling`

v3.7.0 把话题放在节点命名空间下，旧文档/网络资料的名字已过时。查话题前先 `ros2 topic list | grep speed` 确认实际名称。

---

### 实验 2：QoS 不匹配导致的"A message was lost"

```
A message was lost!!!
    total count change:1
    total count: 1---
data: 100.0
```

**原因**：`speed_scaling_state_broadcaster` 以 `RELIABLE + TRANSIENT_LOCAL` 发布。
`ros2 topic echo --once` 默认用 `VOLATILE` 订阅，与 `TRANSIENT_LOCAL` **不兼容**。

| Durability | 行为 |
|---|---|
| `TRANSIENT_LOCAL` | 发布者缓存最近消息，新订阅者加入时补发 |
| `VOLATILE` | 不缓存，订阅者加入前的消息全部错过 |

QoS 匹配规则：订阅者的要求不能高于发布者的承诺。`VOLATILE` 订阅者收不到 `TRANSIENT_LOCAL` 的缓存，只能等下一帧实时消息——这期间计数器已经走了 1，于是报"lost"。

**正确 echo 方式**：
```bash
ros2 topic echo /speed_scaling_state_broadcaster/speed_scaling \
  --qos-durability transient_local --qos-reliability reliable --once
```

---

### 实验 3：set_speed_slider 调用成功但值不变

**操作**：
```bash
ros2 service call /io_and_status_controller/set_speed_slider \
  ur_msgs/srv/SetSpeedSliderFraction \
  "{speed_slider_fraction: 0.5}"
```

**响应**：`success=True`

**结果**：再次读取 broadcaster，值仍为 `100.0`，没有变化。

**根本原因：command/state 分离架构**

ros2_control 的 hardware interface 分两类：

| 接口类型 | 用途 | 谁写 | 谁读 |
|---|---|---|---|
| command interface | 发出指令 | 控制器/服务 | 驱动（发给硬件） |
| state interface | 反映实际状态 | 驱动（从硬件读回） | broadcaster / 控制器 |

`set_speed_slider` 写的是 **command interface**：
```
speed_scaling/target_speed_fraction_cmd        ← 服务写入这里（成功）
speed_scaling/target_speed_fraction_async_success
```

broadcaster 读的是 **state interface**：
```
speed_scaling/speed_scaling_factor             ← broadcaster 从这里读（没变）
```

**完整数据流（真机）**：
```
set_speed_slider 写 target_speed_fraction_cmd
        ↓
ur_robot_driver 通过 RTDE 发送给示教器
        ↓
示教器执行后，把实际速度比例通过 RTDE 回传
        ↓
驱动更新 speed_scaling_factor（state interface）
        ↓
speed_scaling_state_broadcaster 发布新值
```

**mock hardware 下**：RTDE 回传通道不存在，`speed_scaling_factor` 永远是 `1.0`（100%）。
`success=True` 是真实的——command 确实写进去了，只是没有硬件来响应并更新 state。

> **结论**：fake hardware 下 `set_speed_slider` 不会改变 speed scaling 的观测值，这是架构设计的必然结果，不是调用失败。

---

### 实验 4：控制器切换

```bash
ros2 control switch_controllers \
  --deactivate scaled_joint_trajectory_controller \
  --activate joint_trajectory_controller
```

**结果**：
```
Successfully switched controllers!
Deactivated: scaled_joint_trajectory_controller
Activated:   joint_trajectory_controller
```

两种控制器共享同一组 hardware interface，因此不能同时 active，必须先停一个再启另一个（或在同一命令里同时指定）。

---

## 待补充（Day 2）

- [ ] `scaled_joint_trajectory_controller` vs `joint_trajectory_controller` 行为对比表
- [ ] teach pendant 速度限制的完整传导路径
- [ ] 控制器 / 驱动 / MoveIt 三者分工结构图
- [ ] 三个核心问题的自答
