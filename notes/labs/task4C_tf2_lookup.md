# Task4C tf2 主动查询学习记录模板（Day3 复盘版）

## 1. 本次任务目标（先写你自己的话）
- `用 3-5 句话描述这次任务要解决什么问题，以及为什么它对机械臂控制有价值。本次人物解决了查询任意源frame在目标frame坐标系下的变换矩阵，可以帮助控制者知道如何计算源frame在目标frame坐标系下的位姿等信息的计算方式`
- 目标清单（完成后勾选）：
  - [x] 我能解释 `lookup_transform(target, source, time)` 的语义
  - [x] 我能说清 `base_link <- tool0` 与 `tool0 <- base_link` 的差别
  - [x] 我能复现 A/B/C 三组并解释现象
  - [x] 我能通过 `query_time_offset_sec` 主动触发时间外推异常

## 2. 实验环境

| 项目 | 内容 |
|---|---|
| 日期 | `2026.4.2` |
| 机器/系统 | `ubuntu24` |
| ROS 2 版本 | `jazzy` |
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

`lookup_transform(target, source, time)第一个参数是target,第二个参数是source,如果搞反，lookup_transform(tool0,base_link, time)语义上会变成基座在末端坐标系下的位姿`

## 4. 查询时间策略（可控查询时间）

| 场景 | `query_time_offset_sec` | 预期 |
|---|---:|---|
| 最新时刻查询 | `0.0` | 通常能稳定拿到 transform |
| 查询未来时刻 | `> 0`（如 `2.0`） | 容易触发 `ExtrapolationException`（future） |
| 查询过去时刻 | `< 0`（如 `-1.0`） | 可能触发 past extrapolation（取决于缓存） |

`写下你选择 C 组偏移量的理由（为什么是这个值）。选来10.0,因为参数定义类型为double,同时为了避免类型歧义，统一用浮点写法，必须输入小数，并且选择大于零的数值可以让lookup函数查询未来的时刻能稳定触发ExtrapolationException错误`

## 5. A/B/C 实验矩阵（填写版）

| 组别 | 参数 | 预期现象 | 实际现象 | 结论 |
|---|---|---|---|---|
| A 正常 | `source=tool0,target=base_link,offset=0.0` | 持续输出 `tf pose` | `[tf_lookup_node-1] [INFO] [1775118716.227794466] [ur3_tf_lookup_node]: tf pose source=tool0 target=base_link xyz=(0.1335, -0.4081, 0.9506) quat=(0.3587, 0.2110, -0.4411, 0.7952)` | `能正常输出` |
| B frame 不存在 | `source=toolX,target=base_link,offset=0.0` | 持续 `frame missing` 告警 | `[tf_lookup_node-1] [WARN] [1775118728.789467464] [ur3_tf_lookup_node]: tf lookup frame missing (toolX <- base_link): "toolX" passed to lookupTransform argument source_frame does not exist.` | `不能正常输出` |
| C 时间外推 | `source=tool0,target=base_link,offset=10.0` | `extrapolation` 告警 | `[tf_lookup_node-1] [WARN] [1775118779.946577265] [ur3_tf_lookup_node]: tf lookup extrapolation (tool0 <- base_link, query_time_offset_sec=10.000): Lookup would require extrapolation into the future.  Requested time 1775118789.743473 but the latest data is at time 1775118777.297939, when looking up transform from frame [tool0] to frame [base_link]` | `不能正常输出` |

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
  -p query_time_offset_sec:=10.0 \
  -r /tf:=/tf_demo \
  -r /tf_static:=/tf_static_demo
```

## 7. 运行现象与日志摘录（每组 3-5 行）

### 7.1 A 组日志
[tf_lookup_node-1] [INFO] [1775118715.628235157] [ur3_tf_lookup_node]: tf pose source=tool0 target=base_link xyz=(-0.0306, -0.0158, 1.1454) quat=(0.2947, 0.0408, -0.2630, 0.9178)
[tf_lookup_node-1] [INFO] [1775118715.828199840] [ur3_tf_lookup_node]: tf pose source=tool0 target=base_link xyz=(0.0283, -0.1775, 1.0957) quat=(0.3430, 0.0877, -0.3458, 0.8689)
[tf_lookup_node-1] [INFO] [1775118716.027851461] [ur3_tf_lookup_node]: tf pose source=tool0 target=base_link xyz=(0.0864, -0.3189, 1.0202) quat=(0.3622, 0.1529, -0.4095, 0.8233)
[tf_lookup_node-1] [INFO] [1775118716.227794466] [ur3_tf_lookup_node]: tf pose source=tool0 target=base_link xyz=(0.1335, -0.4081, 0.9506) quat=(0.3587, 0.2110, -0.4411, 0.7952)

### 7.2 B 组日志
[tf_lookup_node-1] [WARN] [1775118722.917015419] [ur3_tf_lookup_node]: tf lookup frame missing (toolX <- base_link): "toolX" passed to lookupTransform argument source_frame does not exist. 
[tf_lookup_node-1] [WARN] [1775118724.741166344] [ur3_tf_lookup_node]: tf lookup frame missing (toolX <- base_link): "toolX" passed to lookupTransform argument source_frame does not exist. 
[tf_lookup_node-1] [WARN] [1775118726.765209494] [ur3_tf_lookup_node]: tf lookup frame missing (toolX <- base_link): "toolX" passed to lookupTransform argument source_frame does not exist. 
[tf_lookup_node-1] [WARN] [1775118728.789467464] [ur3_tf_lookup_node]: tf lookup frame missing (toolX <- base_link): "toolX" passed to lookupTransform argument source_frame does not exist. 

### 7.3 C 组日志
[tf_lookup_node-1] [WARN] [1775118778.928086527] [ur3_tf_lookup_node]: tf lookup extrapolation (tool0 <- base_link, query_time_offset_sec=10.000): Lookup would require extrapolation into the future.  Requested time 1775118788.725664 but the latest data is at time 1775118776.847908, when looking up transform from frame [tool0] to frame [base_link]
[tf_lookup_node-1] [WARN] [1775118779.131029879] [ur3_tf_lookup_node]: tf lookup extrapolation (tool0 <- base_link, query_time_offset_sec=10.000): Lookup would require extrapolation into the future.  Requested time 1775118788.928796 but the latest data is at time 1775118776.947920, when looking up transform from frame [tool0] to frame [base_link]
[tf_lookup_node-1] [WARN] [1775118779.334430413] [ur3_tf_lookup_node]: tf lookup extrapolation (tool0 <- base_link, query_time_offset_sec=10.000): Lookup would require extrapolation into the future.  Requested time 1775118789.131738 but the latest data is at time 1775118777.047925, when looking up transform from frame [tool0] to frame [base_link]

## 8. 异常处理复盘卡

| 异常类型 | 本次触发条件 | 你看到的日志特征 | 你的处理策略 |
|---|---|---|---|
| `LookupException` | `传入来错误的frame名` | `日志提示frame不存在` | `修改frame名` |
| `ConnectivityException` | `启动一个独立的终端发布一个独立静态树，可使用以下指令：ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 world isolated_root \  --ros-args -r /tf:=/tf_demo -r /tf_static:=/tf_static_demo ，再查询原TF树和新TF树之间的联系` | `日志提示Could not find a connection between 'base_link' and 'isolated_root' because they are not part of the same tree.` | `修改查询的frame名，确保他们在同一个tf树内` |
| `ExtrapolationException` | `手动设置超前查询时间戳，查询未来的数据` | `Lookup would require extrapolation into the future. ` | `修正查询时间参数` |

## 9. 学习巩固（关键）

1. `解释为什么“可控查询时间”是调试 tf2 的有效手段。tf2 不是在查一张“静态坐标表”，而是在查一棵“随时间缓存的坐标树”。 你传给 lookupTransform 的 time，本质上是在改变查询条件；一旦你能主动控制这个时间，就能把“到底是坐标关系错了，还是时间没对上”这两类问题拆开。tf2 官方文档也明确说明，它维护的是“coordinate frames over time”，并允许在“any desired point in time”查询两帧之间的变换。`
2. `解释为什么 C 组能触发 extrapolation，而 A 组不能。因为C组真正实现主动控制了查询时间参数，主动实现了查询未来的结果，直接可以触发extrapolation；而A组因为写了time 0，所以程序默认使用最新的已被缓存的结果来返回，不能触发extrapolation`
3. `如果用于手术机器人，你会如何设置日志级别与重试节奏？不知道`
4. `你目前最不确定的点是什么？你准备如何验证？lookup transform函数相关的工作流程,在本文档中写一个小节，把该函数的相关分析记录下来`

## 10. 自检问答（完成后可口述）

- [x] 我能一句话解释 `target <- source`
- [x] 我能独立写出 A/B/C 三组命令
- [x] 我能从日志中区分 `frame missing` 与 `extrapolation`
- [x] 我能说明 `query_time_offset_sec` 的正负含义

## 11. 下一步

- `将本页结论同步到阶段复盘文档 notes/reports/stage1_review.md。`
