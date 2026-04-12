# 任务 6B 规划文档：URSim speed scaling 实证（实践版）

## 1. 任务目标
- 在 `ws_stage2` 新建一个独立 Python 应用包 `ur3_ursim_speed_scaling_lab_py`，集中存放 6B 任务代码。
- 将阶段 2 已跑通的 UR3 控制链路从 mock hardware 切换到 URSim。
- 用新包完成两个最小实验：
  - 观测并记录 URSim 回传的真实 `speed_scaling`；
  - 在不同速度滑块条件下发送同一条保守轨迹，比较执行节奏变化。
- 为未来手术机器人系统开发积累“状态真实性验证、实验隔离、最小风险联调”的基本方法。

## 2. 当前基线（来自仓库现状）
- 5A 已完成：`ur_robot_driver` 已在 mock hardware 下跑通。
- 5B 已完成：已确认真实观测话题为 `/speed_scaling_state_broadcaster/speed_scaling`，并已搞清 QoS 兼容性与 command/state 分离原因。
- 5C 已完成：`ur3_minimal_control_lab_py` 已验证最小控制闭环。
- 6A 已完成：`ur3_minimal_control_lab_cpp` 已验证 C++ `FollowJointTrajectory` 发送链路。
- `workspaces/ws_stage2/src` 当前已有两个阶段 2 应用包：
  - `ur3_minimal_control_lab_py`
  - `ur3_minimal_control_lab_cpp`
- 当前仓库里还没有专门承载“URSim speed scaling 实证”的独立应用包。

## 3. 任务范围（单功能约束）
- 包含：
  - 新建一个独立 Python 应用包；
  - 包内提供 1 个 speed scaling 观测节点；
  - 包内提供 1 个最小轨迹发送节点，用于对比 `100%` 与降速条件下的执行耗时；
  - 提供 1 个 launch 文件和 1 份实验记录文档；
  - 完成一次 URSim 下的真实 speed scaling 闭环验证。
- 不包含：
  - C++ 复刻版本；
  - MoveIt 规划；
  - 真机接入；
  - 新的轨迹算法、复杂避障或安全策略设计；
  - 对 5C / 6A 既有包做跨任务重构。

> 说明：6B 的单一功能是“验证 URSim 下真实 speed scaling 对执行节奏的影响”。即使新包里有观测节点和发送节点，它们也都服务于这一个实验目标。

## 4. 新建应用包设计（本任务硬性要求）
- 包名：`ur3_ursim_speed_scaling_lab_py`
- 位置：`workspaces/ws_stage2/src/ur3_ursim_speed_scaling_lab_py`
- 目录骨架：
  ```text
  ur3_ursim_speed_scaling_lab_py/
  ├── package.xml
  ├── setup.py
  ├── setup.cfg
  ├── resource/
  │   └── ur3_ursim_speed_scaling_lab_py
  ├── config/
  │   └── task6B_experiment.yaml
  ├── launch/
  │   └── task6B_ursim_speed_scaling.launch.py
  └── ur3_ursim_speed_scaling_lab_py/
      ├── __init__.py
      ├── speed_scaling_monitor.py
      └── scaled_trajectory_runner.py
  ```

## 5. 为什么 6B 要新建包，而不是继续塞进 5C / 6A
- 5C 的重点是“最小控制闭环”，6A 的重点是“C++ Action Client 迁移”，它们都不是 URSim speed scaling 实证任务本身。
- 6B 需要新增的节点会带入 URSim 专属假设，例如：
  - `robot_ip` 指向容器；
  - `speed_scaling` 订阅必须使用特定 QoS；
  - 轨迹幅度要更保守，便于在不同滑块条件下比较执行时间。
- 新建独立包可以把 6B 的实验参数、日志和代码边界单独封装，避免把 5C 已完成闭环污染成“多目标试验场”。
- 复用的应该是思路和模式，而不是继续把新实验堆进旧包。

## 6. 与现有包的关系
- `ur3_minimal_control_lab_py`：
  - 复用其“先等 `/joint_states`、再发 Action goal、再看 result”的控制链路经验；
  - 6B 不再把新代码直接写进这个包中。
- `ur3_minimal_control_lab_cpp`：
  - 保留为 6A 的独立成果；
  - 本任务不把 C++ 复验并入 6B。
- 6B 新包中若需要借鉴 5C 的轨迹发送逻辑，应优先复用接口形状、日志风格与参数命名，不在本任务中做跨包公共库抽取。

## 7. learn mode 分工
- 智能体负责（低学习价值，可直接完成）：
  - 包脚手架、依赖声明、安装规则；
  - `speed_scaling_monitor` 的 QoS 配置与日志外壳；
  - `scaled_trajectory_runner` 的参数、校验、基础发送链路与 launch 文件；
  - 命令清单、观察矩阵、实验记录模板。
- 人类负责（关键学习点，执行时需你亲自决策）：
  - 判断 6B 演示轨迹的幅度是否足够保守；
  - 在 URSim 界面中完成 ready / brake release / 速度滑块调节等实际操作；
  - 解释为什么 `speed_scaling` 变了、而轨迹 result 仍可能成功；
  - 判断当前证据是否足以说明“这是真实 speed scaling，而不是 mock 假象”。

## 8. 核心学习问题
1. 为什么 mock hardware 下 `speed_scaling` 永远是 `100%`，而 URSim 下会随速度滑块变化？
2. 为什么 6B 最值得亲手写的不是“大段 Action 胶水”，而是“如何设计一个安全、可比较、可解释的实验”？
3. 为什么 `scaled_joint_trajectory_controller` 在降速后常常仍能成功完成 goal，但耗时会变长？
4. 为什么做机器人实验时，把不同任务拆进独立应用包会提高排障效率？

## 9. 实施步骤（推荐 4 次练习）

### 练习 1：先把 6B 应用包建出来
1. 在 `ws_stage2/src` 新建 `ur3_ursim_speed_scaling_lab_py`。
2. 先只完成包骨架、入口、安装规则与 launch 文件。
3. 构建通过后再进入 URSim 联调，避免把“建包失败”和“驱动连接失败”混在一起。

### 练习 2：只验证 URSim 真实 speed scaling 可观测
1. 启动 URSim Docker，并在图形界面中完成必要确认。
2. 获取 URSim 容器 IP。
3. 启动 `ur_robot_driver` 连接 URSim，确保不是 `use_mock_hardware:=true`。
4. 运行 `speed_scaling_monitor`，先在 `100%` 速度滑块下读到基线值。
5. 把 URSim 速度滑块调到 `50%`、再调到更低，观察节点输出是否随之变化。

### 练习 3：在新包里做一次最小对比实验
1. 在 `scaled_trajectory_runner.py` 中准备一条比 5C 更保守的轨迹。
2. 先在 `100%` 下运行一次，记录：
   - goal 是否 accepted；
   - result 是否成功；
   - 总耗时大约多少秒。
3. 再在 `50%` 下运行同一条轨迹，记录同样信息。
4. 比较两次差异，重点看“时间轴是否被拉伸”，而不是只盯着 success / failure。

### 练习 4：用自己的话完成解释闭环
1. 在实验记录里回答：
   - 这次为什么算“真实 speed scaling 实证”？
   - 为什么 result 仍成功，不代表速度滑块没有生效？
   - 如果把控制器换成 `joint_trajectory_controller`，你预计会有什么差异？
2. 用自己的语言说明：
   - command interface 与 state interface 在这次实验中分别扮演什么角色；
   - 这次实验对未来手术机器人系统的控制链路设计有什么启发。

## 10. 关键命令（执行阶段使用）
```bash
# 构建 6B 新应用包
cd /home/minzi/ros2_lab/workspaces/ws_stage2
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ur3_ursim_speed_scaling_lab_py
source install/setup.bash

# 启动 URSim（镜像标签以执行当日可用版本为准）
docker run --rm -it \
  -p 5900:5900 \
  -p 6080:6080 \
  -e ROBOT_MODEL=UR3 \
  universalrobots/ursim_e-series

# 查询容器 IP
docker ps
docker inspect -f '{{range.NetworkSettings.Networks}}{{.IPAddress}}{{end}}' <容器名或容器ID>

# 启动 ur_robot_driver 连接 URSim
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur3 \
  robot_ip:=<ursim_container_ip> \
  launch_rviz:=true

# 启动 6B 观测链路
ros2 launch ur3_ursim_speed_scaling_lab_py task6B_ursim_speed_scaling.launch.py

# 单独运行 speed scaling 观测节点
ros2 run ur3_ursim_speed_scaling_lab_py speed_scaling_monitor

# 单独运行最小轨迹发送节点
ros2 run ur3_ursim_speed_scaling_lab_py scaled_trajectory_runner

# 备用：命令行直接验证话题
ros2 topic echo /speed_scaling_state_broadcaster/speed_scaling \
  --qos-durability transient_local \
  --qos-reliability reliable \
  --once
```

## 11. 观察矩阵

| 组别 | 条件 | 预期现象 | 证据 |
|---|---|---|---|
| A 基线 | URSim 速度滑块 `100%` | `speed_scaling` 接近 `100` | monitor 日志 |
| B 降速 | URSim 速度滑块 `50%` | `speed_scaling` 明显下降 | monitor 日志 |
| C 低速 | URSim 速度滑块更低 | `speed_scaling` 继续下降 | monitor 日志 |
| D 执行对比 | 同一条轨迹，`100%` vs `50%` | 两次都可能成功，但低速那次耗时更长 | runner 日志 |

## 12. 交付物
- 代码：`workspaces/ws_stage2/src/ur3_ursim_speed_scaling_lab_py`
- 文档：`notes/labs/task6B_ursim_speed_scaling.md`
- 结果：你能清楚演示一次“URSim 回传真实 speed scaling，并影响 `scaled_joint_trajectory_controller` 执行节奏”的最小实验。

## 13. 验收标准
- 6B 新包能独立构建通过。
- `speed_scaling_monitor` 能在 URSim 下观测到非 `100%` 的 speed scaling。
- 用同一条保守轨迹在两档速度滑块下运行时，能看到执行耗时差异。
- 你能用自己的语言解释：
  - 为什么这次是“真实 speed scaling 验证”；
  - 为什么低速下 result 仍可能成功；
  - 为什么 6B 值得新建独立应用包，而不是继续堆在 5C / 6A 里。

## 14. 风险与回退
- 风险 1：URSim 容器已启动，但界面尚未真正 ready。
  - 回退：先完成虚拟示教器内的人机确认，再连接驱动。
- 风险 2：`speed_scaling_monitor` 没收到消息，看起来像功能失效。
  - 回退：先检查 QoS 是否为 `transient_local + reliable`，再用命令行 echo 交叉验证。
- 风险 3：第一次设计的轨迹幅度过大，不利于保守实验。
  - 回退：先只动 1 到 2 个关节的小幅角度，并延长总执行时间。
- 风险 4：把“URSim 接入”“speed scaling 观测”“轨迹发送”同时一起调，排障面过大。
  - 回退：严格按“先连上 -> 再观测 -> 最后发送轨迹”的顺序推进。

## 15. 完成记录
- 状态：`[x] 已完成`
- 日期：`2026-04-12`
- 备注：已按“6B 新建独立应用包”方案完成实证。URSim 下已观测到 `speed_scaling` 从 `100.0` 变为 `50.0`，同一条保守轨迹在 `100%` 下耗时约 `5.00s`、在 `50%` 下耗时约 `10.00s`，两次均返回 `status=4 error_code=0`。
