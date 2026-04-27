# 任务 8D 规划文档：低速小范围 home / ready 关节动作

## 1. 任务目标
- 在 8A-8C 全部通过后，执行第一组低速、小范围、人工确认过的 UR3 真机关节空间动作。
- 只做 home / ready 点，不做复杂规划、不做笛卡尔路径、不做 Servo。
- 为每次动作记录当前状态、目标、关节 delta、速度限制、执行结果和人工确认。

## 2. 当前基线（来自仓库现状）
- 阶段 2 已完成 FollowJointTrajectory 和 URSim speed scaling。
- 阶段 3 已完成 MoveIt 规划执行，但真机首轮不直接复用复杂规划。
- 8C 将提供执行前状态门闩。

## 3. 任务范围（单功能约束）
- 包含：
  - 受限 home / ready 关节目标；
  - joint range 与 delta range 检查；
  - 速度、加速度、持续时间保守约束；
  - 人工确认后发送单次 FollowJointTrajectory；
  - 执行结果日志。
- 不包含：
  - MoveIt pose goal；
  - Cartesian Path；
  - Servo / Twist 控制；
  - 循环执行；
  - 自动恢复 protective stop；
  - 非人工确认的无人值守动作。

## 4. 包设计建议
- 包名：`ur3_real_guarded_motion_lab_cpp`
- 位置：`workspaces/ws_stage4/src/ur3_real_guarded_motion_lab_cpp`
- 目录骨架建议：
  ```text
  ur3_real_guarded_motion_lab_cpp/
  ├── CMakeLists.txt
  ├── package.xml
  ├── config/
  │   └── task8D_guarded_targets.yaml
  ├── launch/
  │   └── task8D_guarded_home_ready.launch.py
  └── src/
      └── guarded_joint_motion_node.cpp
  ```
- 首轮节点默认 dry-run 或 require-confirm，必须显式参数打开执行。

## 5. learn mode 分工
- 智能体负责：
  - 包骨架；
  - 参数读取、日志结构、Action Client 外壳；
  - joint range / delta range 检查脚手架；
  - 默认拒绝执行路径。
- 人类负责：
  - 选择 home / ready 点；
  - 判断目标是否处于现场安全工作空间；
  - 确认速度、加速度、持续时间是否足够保守；
  - 每次动作前进行现场确认。

## 6. 核心学习问题
1. 为什么真机第一组动作应选择关节空间 home / ready，而不是 pose goal？
2. 为什么目标范围检查还不够，必须检查从当前状态到目标的 delta？
3. 为什么默认 dry-run / require-confirm 是真机实验入口的底线？
4. 为什么 Action result 成功也不等于这段逻辑已经是控制系统？

## 7. 实施步骤

### 练习 1：准备目标点但不执行
1. 人类给出 home / ready 候选关节值。
2. 记录每个关节目标、单位、来源、现场审核人。
3. 先运行 dry-run，输出目标与当前状态差异。

### 练习 2：实现执行前检查
1. 检查 8C 状态门闩是否通过。
2. 检查当前 joint state 是否新鲜、关节名是否匹配。
3. 检查目标是否在允许范围内。
4. 检查每个关节 delta 是否小于本任务上限。
5. 检查速度、加速度、执行时长是否保守。

### 练习 3：人工确认后执行单次小动作
1. 只发送一个目标。
2. 保持现场急停可达。
3. 观察执行中状态与 driver 日志。
4. 执行结束后记录 result、最终 joint state、误差。

### 练习 4：复盘并决定是否允许第二个目标
1. 如果第一目标异常，停止任务并进入 8E。
2. 如果第一目标正常，人工确认是否继续第二个 ready 点。
3. 不进行循环动作。

## 8. 关键命令（执行阶段使用）
```bash
cd /home/minzi/ros2_lab/workspaces/ws_stage4
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ur3_real_guarded_motion_lab_cpp
source install/setup.bash

# dry-run：只检查，不发送 goal
ros2 launch ur3_real_guarded_motion_lab_cpp task8D_guarded_home_ready.launch.py \
  execute:=false \
  target_name:=ready

# 真实执行：必须经过人类现场确认后才允许
ros2 launch ur3_real_guarded_motion_lab_cpp task8D_guarded_home_ready.launch.py \
  execute:=true \
  require_confirmation:=true \
  target_name:=ready
```

## 9. 执行前记录模板

| 项目 | 内容 |
|---|---|
| 时间 | |
| 操作者 | |
| 旁站确认 | |
| 目标名 | home / ready |
| 当前 joint state | |
| 目标 joint state | |
| 每关节 delta | |
| controller 状态 | |
| robot / safety / program 状态 | |
| 速度 / 加速度 / 时长 | |
| 人工确认 | yes / no |

## 10. 交付物
- 代码：`workspaces/ws_stage4/src/ur3_real_guarded_motion_lab_cpp`
- 文档：`notes/labs/task8D_guarded_home_ready_motion.md`
- 结果：一次或少量几次可追踪的低速小范围真机动作。

## 11. 验收标准
- dry-run 能输出当前状态、目标、delta 和检查结果。
- 任何检查失败时默认拒绝执行。
- 真实执行前有人类确认记录。
- 真机能以低速完成一个 home / ready 点动作。
- 每次执行都有目标、状态、结果和异常记录。

## 12. 风险与回退
- 风险 1：目标点未经现场审核。
  - 回退：只保留 dry-run，不允许 `execute:=true`。
- 风险 2：当前状态离目标太远。
  - 回退：拒绝执行，重新定义更近的中间点。
- 风险 3：执行中出现 protective stop 或 controller error。
  - 回退：停止发送目标，进入 8E 异常处理记录。
