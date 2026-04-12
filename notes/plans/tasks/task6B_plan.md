# 任务 6B 规划文档：URSim 接入与真实 speed scaling 验证

## 1. 任务目标
- 将阶段 2 已跑通的 UR3 控制链路从 mock hardware 切换到 URSim。
- 验证真实 RTDE 回传存在时，`speed_scaling_state_broadcaster` 会发布非 100% 的速度缩放值。
- 复用现有最小控制客户端，完成一次“调低速度滑块 -> 发送轨迹 -> 观察 speed scaling 与执行结果”的闭环验证。

## 2. 当前基线（来自仓库现状）
- 5A 已完成：已在 mock hardware 下跑通 `ur_robot_driver`。
- 5B 已完成：已理解 `scaled_joint_trajectory_controller`、`joint_trajectory_controller` 与 speed scaling 机制。
- 5C 已完成：`ur3_minimal_control_lab_py` 已验证最小控制闭环。
- 6A 已完成：`ur3_minimal_control_lab_cpp` 已验证 C++ Action Client 可用。
- 当前仓库已有关于 URSim 的概念准备：
  - `notes/labs/task5B_ur_controller_system.md` 已整理 URSim 与 mock hardware 的差异；
  - 已确认真实 speed scaling 观测话题应以 `/speed_scaling_state_broadcaster/speed_scaling` 为准。

## 3. 任务范围（单功能约束）
- 包含：
  - 启动 URSim Docker，并让 `ur_robot_driver` 连接到 URSim；
  - 验证 speed scaling 话题在 URSim 下可随虚拟示教器速度滑块变化；
  - 复用已有最小 Action Client 发送轨迹，并记录 speed scaling 对执行结果的影响；
  - 沉淀一篇实验记录文档。
- 不包含：
  - 新建控制应用包；
  - 真机接入；
  - MoveIt 规划；
  - 新的轨迹算法或安全策略设计；
  - 与 6A 无关的 C++ 重构。

> 说明：本任务的核心只有一个功能点：验证“URSim 提供真实 speed scaling 回传”。因此优先复用现有 `ur3_minimal_control_lab_py` 或 `ur3_minimal_control_lab_cpp`，不重复造轮子。

## 4. 复用策略
- 优先复用的现有包：
  - Python：`workspaces/ws_stage2/src/ur3_minimal_control_lab_py`
  - C++：`workspaces/ws_stage2/src/ur3_minimal_control_lab_cpp`
- 推荐主验证客户端：
  - 首选 Python 版 `joint_trajectory_sender`，因为排障成本更低；
  - C++ 版可作为复验项，不作为 6B 必做项。
- 文档交付路径：
  - `notes/labs/task6B_ursim_speed_scaling.md`

## 5. learn mode 分工
- 智能体负责（低学习价值，可直接完成）：
  - 任务计划、命令清单、日志模板、观察矩阵；
  - 帮你解读 speed scaling 话题、QoS 和控制器日志；
  - 在实验后 review 结论并补文档。
- 人类负责（关键学习点，执行时需你亲自决策）：
  - 启动并操作 URSim 界面；
  - 确认 URSim 是否真正“就绪可控”；
  - 调整虚拟示教器速度滑块并观察结果；
  - 解释“为什么 topic 变化了 / 为什么轨迹耗时变了”。

## 6. 核心学习问题
1. 为什么 URSim 能验证真实 speed scaling，而 mock hardware 不行？
2. 为什么读取 `/speed_scaling_state_broadcaster/speed_scaling` 时要显式带上 QoS 参数？
3. 当速度滑块从 `100%` 降到 `50%` 时，`scaled_joint_trajectory_controller` 的执行行为会怎么变化？

## 7. 实施步骤（推荐 3 次练习）

### 练习 1：让 URSim 与驱动成功连上
1. 启动 URSim Docker。
2. 等待 URSim 完全启动，并在图形界面中完成必要的人机确认。
3. 获取 URSim 容器 IP。
4. 启动 `ur_robot_driver`，将 `robot_ip` 指向 URSim 容器 IP，且不要启用 `use_mock_hardware:=true`。
5. 验证：
   - `ros2 control list_controllers`
   - `ros2 topic echo /joint_states --once`
   - `ros2 action list`

### 练习 2：验证 speed scaling 真实回传
1. 使用正确 QoS 读取：
   ```bash
   ros2 topic echo /speed_scaling_state_broadcaster/speed_scaling \
     --qos-durability transient_local \
     --qos-reliability reliable \
     --once
   ```
2. 在 URSim 虚拟示教器中把速度滑块从 `100%` 调到 `50%`、再调到更低。
3. 每次调整后重新读取一次 topic，记录数值变化。
4. 若需要，再尝试对照：
   - `/io_and_status_controller/set_speed_slider`
   - broadcaster 最终观测值

### 练习 3：做一次“缩放影响轨迹执行”验证
1. 保持 `scaled_joint_trajectory_controller` active。
2. 用现有 Python 版发送节点跑一条已知轨迹。
3. 在 `100%` 和 `50%` speed scaling 下各运行一次，比较：
   - Action 完成耗时；
   - feedback 持续时间；
   - result 是否成功。
4. 用自己的话解释：
   - 轨迹形状有没有变；
   - 为什么最终 result 仍可能成功；
   - 时间轴为什么会被拉伸。

## 8. 关键命令（执行阶段使用）
```bash
# 1) 启动 URSim（镜像名 / 标签以执行当日官方文档为准）
docker run --rm -it \
  -p 5900:5900 \
  -p 6080:6080 \
  -e ROBOT_MODEL=UR3 \
  <官方_URSim_镜像>

# 2) 查询容器 IP（示例）
docker ps
docker inspect -f '{{range.NetworkSettings.Networks}}{{.IPAddress}}{{end}}' <容器名或容器ID>

# 3) 启动 ur_robot_driver 连接 URSim
cd /home/minzi/ros2_lab/workspaces/ws_stage2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur3 \
  robot_ip:=<ursim_container_ip> \
  launch_rviz:=true

# 4) 验证 speed scaling 话题
ros2 topic echo /speed_scaling_state_broadcaster/speed_scaling \
  --qos-durability transient_local \
  --qos-reliability reliable \
  --once

# 5) 发送最小轨迹（推荐复用 Python 版）
ros2 run ur3_minimal_control_lab_py joint_trajectory_sender
```

## 9. 观察矩阵

| 组别 | 条件 | 预期现象 | 证据 |
|---|---|---|---|
| A 基线 | URSim 速度滑块 `100%` | speed scaling topic 接近 `100` | topic echo 日志 |
| B 降速 | URSim 速度滑块 `50%` | speed scaling topic 明显下降 | topic echo 日志 |
| C 执行 | `scaled_joint_trajectory_controller` + `50%` | Action 仍成功，但反馈持续更久 | 发送节点日志 |

## 10. 验收标准
- `ur_robot_driver` 能连接 URSim，而不是 mock hardware。
- `/speed_scaling_state_broadcaster/speed_scaling` 在 URSim 下不再固定为 `100%`。
- 用现有最小客户端发送轨迹时，能观察到 speed scaling 对执行节奏的影响。
- 你能清楚解释：为什么这次是“真实 speed scaling 验证”，而 5B 里的 mock hardware 不是。

## 11. 风险与回退
- 风险 1：URSim 容器已启动，但控制器尚未真正就绪。
  - 回退：先在 URSim 界面中完成必要确认，再重启驱动连接。
- 风险 2：`/speed_scaling_state_broadcaster/speed_scaling` 读取仍出现 `A message was lost!!!`。
  - 回退：继续使用 `transient_local + reliable` 组合，并记录这是 QoS 兼容问题而不是功能失效。
- 风险 3：把“URSim 接入”和“控制器比较实验”同时做太多，导致定位困难。
  - 回退：先只验证 topic 变化，再做轨迹发送实验。
- 风险 4：直接复用 5C 当前较大幅度轨迹，现象虽然明显，但不利于后续更保守的控制习惯。
  - 回退：若联调不稳定，可先把轨迹幅度调小后再复验。

## 12. 完成记录
- 状态：`[ ] 未开始` `[#] 进行中` `[x] 已完成`
- 日期：`2026-04-12`
- 备注：本轮仅创建计划文档；执行时建议优先复用 Python 版最小客户端，完成 URSim speed scaling 实证后再考虑是否扩展到 C++ 复验。
