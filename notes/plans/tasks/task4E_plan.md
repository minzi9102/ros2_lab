# 任务 4E 规划文档：ros2_control 最小链路（实践版）

## 1. 任务目标
- 新建一个独立应用包 `ur3_ros2_control_lab_py`，集中存放 4E 任务代码。
- 跑通最小控制链：`robot_description -> controller_manager -> joint_state_broadcaster -> /joint_states`。
- 能解释 `read / update / write` 循环与 `controller_manager`、`hardware interface` 的职责边界。

## 2. 当前基线（来自仓库现状）
- 阶段任务进度：4A/4B/4C/4D 已完成，4E 尚未开始。
- 现有包均为单功能练习包，`ws_tutorials/src` 下暂无专用于 `ros2_control` 的应用包。
- 已具备 xacro 入口：`ur3_joint_state_publisher_py/urdf/ur3_simplified.urdf.xacro`，可作为 4E 的模型来源。
- 4F/4G 尚未开始，本任务只推进 4E，不提前实现轨迹联调与控制器对比。

## 3. 任务范围（单功能约束）
- 包含：
  - 新建应用包骨架与安装配置；
  - 最小 `ros2_control` 配置（先 `joint_state_broadcaster`）；
  - 启动链路与控制器状态验证命令；
  - 4E 学习文档沉淀。
- 不包含：
  - `joint_trajectory_controller` 联调（归属 4F）；
  - `forward_command_controller` 行为对比（归属 4G）；
  - 真机驱动接入、实时内核优化、MoveIt 集成。

## 4. 新建应用包设计（本任务硬性要求）
- 包名：`ur3_ros2_control_lab_py`
- 位置：`workspaces/ws_tutorials/src/ur3_ros2_control_lab_py`
- 目标目录骨架（计划）：
  - `launch/ur3_ros2_control_minimal.launch.py`
  - `config/ur3_controllers_minimal.yaml`
  - `urdf/ur3_simplified_ros2_control.urdf.xacro`
  - `package.xml` / `setup.py` / `setup.cfg`
  - `resource/ur3_ros2_control_lab_py`
  - `ur3_ros2_control_lab_py/__init__.py`

## 5. learn mode 分工
- 智能体负责（低学习价值，可直接完成）：
  - 新包脚手架、依赖声明、安装规则；
  - launch 与 controller 配置接线；
  - 命令清单、日志模板、验收口径。
- 人类负责（关键学习点，执行时需你亲自决策）：
  - 选择硬件插件方案：`mock_components/GenericSystem` 或后续自定义硬件接口；
  - 定义 state/command interface 集合与关节映射策略；
  - 失败场景处置策略（加载失败时是否自动降级为仅 broadcaster）。

## 6. 实施步骤（3 天节奏）
1. Day 1：建包与最小配置落盘
- 新建 `ur3_ros2_control_lab_py`；
- 在 xacro 中注入最小 `ros2_control` 标签（6 关节 state interface 必选）；
- 编写 `ur3_controllers_minimal.yaml`（仅 `joint_state_broadcaster`）。

2. Day 2：启动链路闭环
- 在 launch 中同时启动 `robot_state_publisher` 与 `ros2_control_node`；
- 使用 `spawner` 拉起 `joint_state_broadcaster`；
- 跑通 `ros2 control list_controllers` 与 `/joint_states` 观测。

3. Day 3：验证与文档沉淀
- 完成 A/B/C 验证矩阵并记录日志；
- 归纳最小可用配置、常见报错、排障顺序；
- 更新 `notes/concepts/ros2_control_minimal_chain.md`。

## 7. 实验矩阵（4E 最小可复现）

| 组别 | 输入条件 | 预期现象 | 证据 |
|---|---|---|---|
| A 基线 | 仅启动 `ros2_control_node`，不 spawner 控制器 | `controller_manager` 正常运行 | `ros2 node list` + 启动日志 |
| B 闭环 | 启动并激活 `joint_state_broadcaster` | 可见 `active` 状态，`/joint_states` 持续发布 | `ros2 control list_controllers` + `ros2 topic echo` |
| C 异常 | 人为制造接口名不匹配或关节映射错误 | 控制器加载失败但日志可定位根因 | 报错链路记录 + 恢复步骤 |

## 8. 关键命令（执行阶段使用）
```bash
cd /home/minzi/ros2_lab/workspaces/ws_tutorials
ros2 pkg create --build-type ament_python ur3_ros2_control_lab_py
colcon build --packages-select ur3_ros2_control_lab_py
source install/setup.bash
ros2 launch ur3_ros2_control_lab_py ur3_ros2_control_minimal.launch.py
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 topic echo /joint_states --once
```

## 9. 交付物
- 代码：`ur3_ros2_control_lab_py` 最小应用包（仅 4E 所需文件）。
- 文档：`notes/concepts/ros2_control_minimal_chain.md`（含步骤、日志、结论、风险）。

## 10. 验收标准
- `controller_manager` 正常运行，`joint_state_broadcaster` 能稳定激活；
- 能复述状态链路与控制链路最小闭环；
- 能独立说明 1 个成功案例与 1 个失败案例的排障过程；
- 不引入 4F/4G 范围内的额外功能。

## 11. 风险与回退
- 风险 1：`ros2_control` 标签与控制器 YAML 关节名不一致。
  - 回退：先只保留 `joint_state_broadcaster`，确认状态链路再加命令接口。
- 风险 2：依赖或安装项遗漏导致 launch 找不到配置文件。
  - 回退：先本地直跑 YAML/xacro 路径，确认后再修安装规则。
- 风险 3：一次性引入多个控制器导致排障面扩大。
  - 回退：严格单控制器闭环，通过后再进入 4F。

## 12. 完成记录
- 状态：`[ ] 未开始` `[#] 进行中` `[x] 已完成`
- 日期：`2026-04-07`
- 备注：已更新为实践版计划，明确新建应用包与最小闭环路径。
