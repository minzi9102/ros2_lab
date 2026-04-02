# Task4C tf2 主动查询学习记录模板（Day3 复盘版）

## 1. 本次任务目标（先写你自己的话）
- `TODO(human): 用 3-5 句话描述这次任务要解决什么问题，以及为什么它对机械臂控制有价值。`
- 目标清单（完成后勾选）：
  - [ ] 我能解释 `lookup_transform(target, source, time)` 的语义
  - [ ] 我能说清 `base_link <- tool0` 与 `tool0 <- base_link` 的差别
  - [ ] 我能复现 A/B/C 三组并解释现象
  - [ ] 我能通过 `query_time_offset_sec` 主动触发时间外推异常

## 2. 实验环境

| 项目 | 内容 |
|---|---|
| 日期 | `TODO(human)` |
| 机器/系统 | `TODO(human)` |
| ROS 2 版本 | `TODO(human)` |
| 工作区 | `/home/minzi/ros2_lab/workspaces/ws_tutorials` |
| 代码包 | `ur3_tf_lookup_py` |
| 基线链路 | `ur3_joint_state_publisher_py ur3_simplified_rviz.launch.py` |
| TF 话题 | `/tf_demo` + `/tf_static_demo` |

## 3. 坐标系与语义确认（关键）

### 3.1 目标表达
- 本任务常用目标：`末端在基座坐标系下位姿`
- 语义写法：`base_link <- tool0`

### 3.2 API 语义
- `lookup_transform(target, source, time)` 返回的是：`target <- source`

### 3.3 本次参数映射
- `target_frame=base_link`
- `source_frame=tool0`

`TODO(human): 用你自己的例子再写一条“target/source 搞反会导致什么后果”。`

## 4. 查询时间策略（可控查询时间）

| 场景 | `query_time_offset_sec` | 预期 |
|---|---:|---|
| 最新时刻查询 | `0.0` | 通常能稳定拿到 transform |
| 查询未来时刻 | `> 0`（如 `2.0`） | 容易触发 `ExtrapolationException`（future） |
| 查询过去时刻 | `< 0`（如 `-1.0`） | 可能触发 past extrapolation（取决于缓存） |

`TODO(human): 写下你选择 C 组偏移量的理由（为什么是这个值）。`

## 5. A/B/C 实验矩阵（填写版）

| 组别 | 参数 | 预期现象 | 实际现象 | 结论 |
|---|---|---|---|---|
| A 正常 | `source=tool0,target=base_link,offset=0.0` | 持续输出 `tf pose` | `TODO(human)` | `TODO(human)` |
| B frame 不存在 | `source=toolX,target=base_link,offset=0.0` | 持续 `frame missing` 告警 | `TODO(human)` | `TODO(human)` |
| C 时间外推 | `source=tool0,target=base_link,offset=2.0` | `extrapolation` 告警 | `TODO(human)` | `TODO(human)` |

## 6. 关键命令（可直接复现）

### 6.1 启动基线链路
```bash
cd /home/minzi/ros2_lab/workspaces/ws_tutorials
source install/setup.bash
ros2 launch ur3_joint_state_publisher_py ur3_simplified_rviz.launch.py use_rviz:=false
```

### 6.2 A 组（正常）
```bash
source /home/minzi/ros2_lab/workspaces/ws_tutorials/install/setup.bash
ros2 run ur3_tf_lookup_py tf_lookup_node --ros-args \
  -p source_frame:=tool0 \
  -p target_frame:=base_link \
  -p query_rate_hz:=5.0 \
  -p timeout_sec:=0.2 \
  -p query_time_offset_sec:=0.0 \
  -r /tf:=/tf_demo \
  -r /tf_static:=/tf_static_demo
```

### 6.3 B 组（frame 不存在）
```bash
source /home/minzi/ros2_lab/workspaces/ws_tutorials/install/setup.bash
ros2 run ur3_tf_lookup_py tf_lookup_node --ros-args \
  -p source_frame:=toolX \
  -p target_frame:=base_link \
  -p query_rate_hz:=5.0 \
  -p timeout_sec:=0.2 \
  -p query_time_offset_sec:=0.0 \
  -r /tf:=/tf_demo \
  -r /tf_static:=/tf_static_demo
```

### 6.4 C 组（时间外推）
```bash
source /home/minzi/ros2_lab/workspaces/ws_tutorials/install/setup.bash
ros2 run ur3_tf_lookup_py tf_lookup_node --ros-args \
  -p source_frame:=tool0 \
  -p target_frame:=base_link \
  -p query_rate_hz:=5.0 \
  -p timeout_sec:=0.2 \
  -p query_time_offset_sec:=2.0 \
  -r /tf:=/tf_demo \
  -r /tf_static:=/tf_static_demo
```

## 7. 运行现象与日志摘录（每组 3-5 行）

### 7.1 A 组日志
`TODO(human): 粘贴 3-5 行连续 tf pose 日志`

### 7.2 B 组日志
`TODO(human): 粘贴 3-5 行 frame missing 告警`

### 7.3 C 组日志
`TODO(human): 粘贴 3-5 行 extrapolation 告警（含 Requested time / latest data）`

## 8. 异常处理复盘卡

| 异常类型 | 本次触发条件 | 你看到的日志特征 | 你的处理策略 |
|---|---|---|---|
| `LookupException` | `TODO(human)` | `TODO(human)` | `TODO(human)` |
| `ConnectivityException` | `TODO(human)` | `TODO(human)` | `TODO(human)` |
| `ExtrapolationException` | `TODO(human)` | `TODO(human)` | `TODO(human)` |

## 9. 学习巩固（关键）

1. `TODO(human): 解释为什么“可控查询时间”是调试 tf2 的有效手段。`
2. `TODO(human): 解释为什么 C 组能触发 extrapolation，而 A 组不能。`
3. `TODO(human): 如果用于手术机器人，你会如何设置日志级别与重试节奏？`
4. `TODO(human): 你目前最不确定的点是什么？你准备如何验证？`

## 10. 自检问答（完成后可口述）

- [ ] 我能一句话解释 `target <- source`
- [ ] 我能独立写出 A/B/C 三组命令
- [ ] 我能从日志中区分 `frame missing` 与 `extrapolation`
- [ ] 我能说明 `query_time_offset_sec` 的正负含义

## 11. 下一步

- `TODO(human): 计划 1 个“过去时刻查询（offset<0）”扩展实验，并写出预期。`
- `TODO(human): 将本页结论同步到阶段复盘文档 notes/reports/stage1_review.md。`
