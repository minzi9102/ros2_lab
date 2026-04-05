# Task4D URDF 与 xacro 参数化学习记录（实践版）

## 1. 本次学习目标（用自己的话）
- `TODO(human): 用 3-5 句话描述为什么要从 URDF 升级到 xacro。`
- 目标清单（完成后勾选）：
  - [ ] 我能解释 xacro 相比单体 URDF 的收益与代价
  - [ ] 我能说清参数从 launch 传递到模型生效的路径
  - [ ] 我能复现 A/B/C 三组参数覆盖实验
  - [ ] 我能指出当前参数设计中最需要改进的 1-2 点

## 2. 实验环境

| 项目 | 内容 |
|---|---|
| 日期 | `TODO(human)` |
| 机器/系统 | `TODO(human)` |
| ROS 2 版本 | `jazzy` |
| 工作区 | `/home/minzi/ros2_lab/workspaces/ws_tutorials` |
| 代码包 | `ur3_joint_state_publisher_py` |
| 启动链路 | `ur3_simplified_rviz.launch.py` |

## 3. 文件结构梳理（4D 脚手架）
- xacro 入口：`workspaces/ws_tutorials/src/ur3_joint_state_publisher_py/urdf/ur3_simplified.urdf.xacro`
- xacro 宏：`workspaces/ws_tutorials/src/ur3_joint_state_publisher_py/urdf/ur3_simplified_macro.xacro`
- launch 接入：`workspaces/ws_tutorials/src/ur3_joint_state_publisher_py/launch/ur3_simplified_rviz.launch.py`
- 安装与依赖：
  - `workspaces/ws_tutorials/src/ur3_joint_state_publisher_py/setup.py`
  - `workspaces/ws_tutorials/src/ur3_joint_state_publisher_py/package.xml`

## 4. 参数入口与传递关系

| 参数名 | 入口位置 | 当前默认值 | 影响位置 | 备注 |
|---|---|---:|---|---|
| `link_2_length` | launch arg -> xacro arg | `0.34` | `link_2` 几何长度与 `elbow_joint` 原点 | 几何参数 |
| `tool0_offset_z` | launch arg -> xacro arg | `0.12` | `tool0_fixed_joint` 的 `origin.z` | 末端偏移 |
| `joint_limit_scale` | launch arg -> xacro arg | `1.0` | 各 revolute joint 的 `lower/upper` | 限位缩放 |

`TODO(human): 判断这 3 个参数是否是你想保留的最小集合；若不是，写出替换方案。`

## 5. 关键宏与映射
- 入口文件负责：
  - 声明 xacro 参数；
  - include 宏文件；
  - 调用 `ur3_simplified_chain`。
- 宏文件负责：
  - 定义 link/joint 结构；
  - 将参数映射到几何和限位。

`TODO(human): 说明你是否接受“单宏承载整条机械臂”的方案，或计划拆成多宏（例如 base/arm/wrist/tool）。`

## 6. A/B/C 实验矩阵（填写版）

| 组别 | 启动参数 | 预期现象 | 实际现象 | 结论 |
|---|---|---|---|---|
| A 默认参数 | `link_2_length=0.34 tool0_offset_z=0.12 joint_limit_scale=1.0` | 与旧 URDF 一致 | `TODO(human)` | `TODO(human)` |
| B 几何覆盖 | `link_2_length=0.45` | link_2 与 elbow 位置变化 | `TODO(human)` | `TODO(human)` |
| C 限位覆盖 | `joint_limit_scale=0.7` | 关节 limit 区间缩小 | `TODO(human)` | `TODO(human)` |

## 7. 关键命令

### 7.1 构建
```bash
cd /home/minzi/ros2_lab/workspaces/ws_tutorials
colcon build --packages-select ur3_joint_state_publisher_py
source install/setup.bash
```

### 7.2 A 组（默认）
```bash
ros2 launch ur3_joint_state_publisher_py ur3_simplified_rviz.launch.py \
  use_rviz:=false
```

### 7.3 B 组（几何覆盖）
```bash
ros2 launch ur3_joint_state_publisher_py ur3_simplified_rviz.launch.py \
  use_rviz:=false \
  link_2_length:=0.45
```

### 7.4 C 组（限位覆盖）
```bash
ros2 launch ur3_joint_state_publisher_py ur3_simplified_rviz.launch.py \
  use_rviz:=false \
  joint_limit_scale:=0.7
```

### 7.5 导出 xacro 结果做比对
```bash
ros2 run xacro xacro \
  /home/minzi/ros2_lab/workspaces/ws_tutorials/src/ur3_joint_state_publisher_py/urdf/ur3_simplified.urdf.xacro \
  joint_limit_scale:=0.7 \
  > /tmp/ur3_simplified_scaled.urdf
```

## 8. 风险与注意事项
- 参数过多会让调试路径变长，建议先固定最小参数集。
- `joint_limit_scale` 仅用于学习演示；工程上常见做法是按关节独立配置上下限。
- 如果 launch 报 xacro 相关错误，先单独跑 `ros2 run xacro xacro ...` 验证源文件。

## 9. 复盘与下一步
- 本次最关键的设计判断：`TODO(human)`
- 当前实现的优点：`TODO(human)`
- 当前实现的风险：`TODO(human)`
- 下一步准备衔接到 4E 的内容：`TODO(human)`
