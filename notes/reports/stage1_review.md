# 阶段 1 验收复盘

## 1. 复盘目标

- 逐条核验阶段 1 的三项验收标准，确认能力已落地。
- 汇总"已掌握 / 待加强"清单，识别进入阶段 2 的风险点。
- 产出可追溯的能力证明（命令输出 + 日志片段）。

---

## 2. 验收范围

| 任务 | 主题 | 状态 |
|------|------|------|
| 4A | Service 最小闭环 | ✅ 已完成 |
| 4B | QoS 对比实验 | ✅ 已完成 |
| 4C | tf2 主动查询 | ✅ 已完成 |
| 4D | URDF 升级 xacro 参数化 | ✅ 已完成 |
| 4E | ros2_control 最小链路 | ✅ 已完成 |
| 4F | joint_trajectory_controller 联调 | ✅ 已完成 |
| 4G | forward_command_controller 对比 | ✅ 已完成 |
| 4H | 阶段 1 验收复盘（本文档） | 🔄 进行中 |

---

## 3. 完成情况总览

> 填写说明：用 1-2 句话描述每个任务的核心产出，不需要重复文档内容。

- **4A**：
- **4B**：
- **4C**：
- **4D**：
- **4E**：
- **4F**：
- **4G**：

---

## 4. 关键能力项验收

### 4.1 独立建包能力

> 验收标准：能独立创建并构建一个 ROS 2 package（Python 与 C++ 各至少一个）。

**验证命令：**
```bash
colcon build --packages-select ur3_stage1_review_py
ros2 run ur3_stage1_review_py demo_package_build
```

**实际输出（粘贴日志）：**
```
# 在此粘贴 demo_package_build 节点的终端输出
```

**结论：**
- [ ] Python 包：独立创建并构建通过
- [ ] C++ 包：独立创建并构建通过
- 备注：

---

### 4.2 Action 适配性理解

> 验收标准：能清楚解释为何轨迹执行通常使用 Action 而非 Topic/Service。

**验证命令：**
```bash
ros2 run ur3_stage1_review_py demo_action_rationale
```

**实际输出（粘贴日志）：**
```
# 在此粘贴 demo_action_rationale 节点的终端输出
```

**口述要点（用自己的话填写）：**

1. Topic 的局限：
2. Service 的局限：
3. Action 的优势：

**结论：**
- [ ] 能清晰区分三种通信机制的适用场景
- 备注：

---

### 4.3 机械臂 description 包结构理解

> 验收标准：能阅读并说明一套机械臂 description 包的结构与参数入口。

**验证命令：**
```bash
ros2 run ur3_stage1_review_py demo_description_reader
```

**实际输出（粘贴日志）：**
```
# 在此粘贴 demo_description_reader 节点的终端输出
```

**包结构说明（用自己的话填写）：**

| 文件 / 目录 | 职责 |
|-------------|------|
| `ur.urdf.xacro` | |
| `ur_macro.xacro` | |
| `inc/ur_common.xacro` | |
| `inc/ur_joint_control.xacro` | |
| `config/ur3/joint_limits.yaml` | |
| `config/ur3/default_kinematics.yaml` | |

**结论：**
- [ ] 能定位并解释 description 包的主要文件
- [ ] 能说明修改关节限位应改哪个文件
- 备注：

---

## 5. 问题与风险

> 填写在学习过程中遇到的卡点、理解偏差或尚未解决的疑问。

| # | 问题描述 | 影响范围 | 改进动作 |
|---|----------|----------|----------|
| 1 | | | |
| 2 | | | |
| 3 | | | |

---

## 6. 经验总结

> 填写 3 条最有价值的收获，要求具体、可复用。

1.
2.
3.

---

## 7. 下一阶段进入条件

> 以下条件全部满足后，方可进入阶段 2。

- [ ] 三项验收标准均已通过（4.1 / 4.2 / 4.3 全部打勾）
- [ ] 风险表中的高优先级改进动作已处理或已制定计划
- [ ] 阶段 2 首周目标已明确

---

## 8. 下一步计划

> 阶段 2 方向（初步）：

- 目标：
- 首个任务：
- 参考资料：
