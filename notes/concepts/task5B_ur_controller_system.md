# Task 5B：理解控制器体系

## 状态
- `[x] 已完成`
- Day 1 实验完成：2026-04-12
- Day 2 概念内化完成：2026-04-12

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

---

## Day 2：概念内化

### 两种控制器行为对比

| 维度 | `joint_trajectory_controller` | `scaled_joint_trajectory_controller` |
|---|---|---|
| speed scaling 感知 | 否，忽略 `speed_scaling_factor` | 是，订阅并消费该值 |
| 降速机制 | 无 | 拉伸时间轴（路径形状不变，时间戳等比缩放） |
| teach pendant 50% 时 | 按原始时间戳执行，速度不变 | 用 2× 时间完成同样路径，速度减半 |
| 真机安全性 | 低（可能超出示教器限速，触发安全停机） | 高（遵守示教器限速） |
| fake hardware 行为 | 正常 | 正常（缩放因子默认 100%，等同于无缩放） |
| 适用场景 | 仿真调试、无速度限制需求 | 真机部署 |

**关键细节**：SJTC 降速是通过**拉伸时间轴**实现的，不是修改速度指令。原轨迹要求 2 秒从 A 到 B，50% 缩放下变为 4 秒——关节位置序列完全相同，只有时间戳被缩放。

---

### 控制器 / 驱动 / MoveIt 三者分工

```
┌─────────────────────────────────────────────────────┐
│  MoveIt（规划层）                                    │
│  职责：运动学规划，生成无碰撞轨迹                    │
│  输出：带时间戳的关节角度序列                        │
│  接口：通过 FollowJointTrajectory Action 委托给控制器 │
└───────────────────────┬─────────────────────────────┘
                        │ Action Goal（轨迹）
┌───────────────────────▼─────────────────────────────┐
│  ros2_control 控制器层                               │
│  职责：以 1kHz 控制周期实时跟踪轨迹                  │
│  读：state interface（关节当前位置/速度）             │
│  写：command interface（目标位置/速度）               │
└───────────────────────┬─────────────────────────────┘
                        │ hardware interface
┌───────────────────────▼─────────────────────────────┐
│  ur_robot_driver（驱动层）                           │
│  职责：与硬件通信，暴露 hardware interface           │
│  真机：通过 RTDE 协议与机器人控制器实时交换数据      │
│  mock：软件模拟 hardware interface，无实际通信       │
└─────────────────────────────────────────────────────┘
```

**为什么 MoveIt 不直接控制关节**：关注点分离。规划的时间尺度是毫秒到秒级，实时控制的时间尺度是 1ms 级。MoveIt 只负责"走哪条路"，控制器负责"每一步怎么走"，驱动负责"把指令翻译成硬件能懂的协议"。

---

### 三个核心问题自答

**Q1：teach pendant 速度设为 50% 时，两种控制器各会怎么表现？**

JTC 不感知速度缩放信号，按原始时间戳执行，机械臂以示教器不允许的速度运动，可能触发安全停机。SJTC 读取到 50% 的缩放因子，将轨迹时间轴拉伸为 2 倍，以原速度的一半完成同样路径，遵守示教器限速。

**Q2：为什么 MoveIt 不直接控制关节，而是通过控制器 Action？**

MoveIt 只负责规划路径（上层指令），不负责与 hardware interface 直接通信。与 hardware interface 的读写是控制器的职责。两者通过 `FollowJointTrajectory` Action 解耦，MoveIt 产出轨迹后委托给控制器，控制器负责实时跟踪。

**Q3：fake hardware 模式下 speed scaling 为何始终是 100%？**

`set_speed_slider` 服务调用成功，command interface（`target_speed_fraction_cmd`）确实被写入。但 mock hardware 没有 RTDE 回传通道，驱动无法从"硬件"读回实际速度比例来更新 state interface（`speed_scaling_factor`），broadcaster 读的是 state interface，所以始终发布 100%。这是 command/state 分离架构在 mock 模式下的必然结果，不是调用失败。

---

### 扩展研究：URSim Docker 能否测试真实 speed scaling？

**结论：能测试通信流，不能测试物理效果。**

URSim 是 Universal Robots 官方仿真器，可通过 Docker 运行。与 mock hardware 的核心区别：

| 对比项 | mock hardware | URSim Docker |
|---|---|---|
| RTDE 协议 | 不存在 | 完整实现 |
| speed_scaling_factor 回传 | 永远 100% | 虚拟示教器调节后真实回传 |
| ur_robot_driver 连接方式 | 内部 mock | 通过网络连接 URSim（与真机相同） |
| 动力学模拟 | 无 | 无（仅运动学） |

**URSim 下 speed scaling 的完整链路**：
```
URSim 虚拟示教器调节速度滑块
        ↓
URSim 内部 RTDE 服务更新速度缩放因子
        ↓
ur_robot_driver 通过 RTDE 读取到变化
        ↓
更新 speed_scaling_factor（state interface）
        ↓
speed_scaling_state_broadcaster 发布新值（非 100%）
```

**限制**：URSim 的 RTDE 是模拟实现，不模拟电机动力学、扭矩限制等物理特性。适合验证驱动层逻辑和控制器行为，不适合验证真实运动学效果。

启动方式参考（ur_robot_driver 官方文档）：
```bash
docker run --rm -it \
  -p 5900:5900 -p 6080:6080 \
  -e ROBOT_MODEL=UR3 \
  universalrobots/ursim_e-series
```
连接时将 `robot_ip` 指向 URSim 容器 IP，去掉 `use_mock_hardware:=true`。
