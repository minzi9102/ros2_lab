# Task 7D：点击目标位姿 -> 自动规划执行 Demo

## 状态
- `[ ] 未开始`
- `[#] 进行中`
- `[x] 已完成`
- 当前状态：`[x] 已在 fake hardware 与 URSim 下完成 /goal_pose 点击自动规划与执行验证`

## 1. 目标
- 在 `workspaces/ws_stage3/src/ur3_moveit_goal_pose_lab_cpp` 中实现一个“点击 -> 自动规划”的最小 demo。
- 输入来自 RViz 的 `/goal_pose`。
- 默认映射策略：
  - 保留 `x / y / yaw`
  - 固定 `z`
  - 固定末端朝下
  - 超出工作区直接拒绝

## 2. 应用包位置
- 包路径：`/home/minzi/ros2_lab/workspaces/ws_stage3/src/ur3_moveit_goal_pose_lab_cpp`
- 关键文件：
  - `src/goal_pose_executor_node.cpp`
  - `launch/task7D_goal_pose_auto_plan.launch.py`

## 3. 当前准备情况
- 已完成：
  - `/goal_pose` 订阅骨架；
  - 工作区范围检查；
  - `x / y / yaw -> 目标抓取位姿` 的默认映射；
  - 自动 `plan()` 与可选 `execute()`；
  - 7D 实验记录模板；
  - fake hardware 下点击自动规划与执行验证；
  - URSim 下点击自动规划与执行验证。
- 待你完成：
  - 无，本任务已完成主路径验收。

## 4. 练习 1：构建 7D C++ 包

### 执行命令
```bash
cd /home/minzi/ros2_lab/workspaces/ws_stage3
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ur3_moveit_goal_pose_lab_cpp
source install/setup.bash
```

### 结果记录
- 是否构建通过：是
- 若失败，错误集中在哪个 tf2 / MoveIt 头文件：无

## 5. 练习 2：检查默认映射策略

### 背景
这里最值得你思考的，不是“怎么订阅一个 PoseStamped”，而是“二维点击输入为什么不能直接等于末端最终位姿”。你需要亲自判断：桌面任务的语义应该如何映射成机器人末端姿态。

### 位置
- 文件：`workspaces/ws_stage3/src/ur3_moveit_goal_pose_lab_cpp/src/goal_pose_executor_node.cpp`
- 重点参数：
  - `target_height`
  - `workspace_min_x`
  - `workspace_max_x`
  - `workspace_min_y`
  - `workspace_max_y`
  - `downward_roll_rad`
  - `downward_pitch_rad`

### 已准备内容
- `/goal_pose` 订阅；
- 工作区范围判断；
- yaw 提取；
- 目标姿态构造；
- 自动规划与可选执行。

### 你需要完成的内容
- 审查并修改工作区边界。
- 审查并修改安全高度。
- 判断是否保留输入 yaw。
- 判断“朝下抓取”的固定姿态是否足够合理。

### 权衡点
- 工作区太小会误拒绝可用点位。
- 工作区太大又会把明显不可达点送进规划器。
- 固定高度太低会更容易碰桌，太高又会偏离抓取语义。
- 保留 yaw 很自然，但未必总是与你希望的末端朝向一致。

### 完成标准
- 你能解释当前映射策略背后的理由。
- 你能说清哪些目标应该在规划前就拒绝。
- 你能完成至少一次点击后自动规划。

### 参数填写区
- 当前选择的 `target_height`：0.1519
- 当前选择的工作区边界：
  - `workspace_min_x`：0.0
  - `workspace_max_x`：0.6
  - `workspace_min_y`：-0.3
  - `workspace_max_y`：0.3
- 当前选择的朝下姿态参数：
  - `downward_roll_rad`：M_PI
  - `downward_pitch_rad`：0
- 当前是否保留输入 `yaw`：是
- 你对“固定朝下 + 保留输入 yaw”的解释：2d goal pose的输入信息只包含x,y,yaw，所以必须要补充roll和pitch的数值，固定roll和pitch的数值为竖直指向下方，并添加固定z轴数值，整理成完整的pose类型消息
- 你认为哪些点击目标应在规划前直接拒绝：超过工作区域的；与场景碰撞或不可达的目标当前主要在规划阶段失败，而不是在本地预先拒绝
- 选择这些参数与策略的原因：工作区足够覆盖桌面点击测试范围，固定高度和朝下姿态能形成稳定的最小点击抓取语义

### 恢复方式
- 你调整完参数或逻辑后告诉我“请 review task7D 实现”。
- 我会继续帮你：
  - 检查边界条件；
  - review 可达性与姿态逻辑；
  - 修复构建 / 运行问题。

## 6. 练习 3：运行记录

### 执行命令
```bash
ros2 launch ur3_moveit_goal_pose_lab_cpp task7D_goal_pose_auto_plan.launch.py \
  execute_plan:=true
```

### 成功轮次记录
- 运行日期：2026.4.16 17：23
- 运行环境：fake hardware 
- 是否使用 `execute_plan=true`：是
- 本轮 RViz 的 `Fixed Frame`：base_link
- 本轮使用的参数：
  - `target_height`：0.1519
  - `workspace_min_x`：0.0
  - `workspace_max_x`：0.6
  - `workspace_min_y`：-0.3
  - `workspace_max_y`：0.3
  - `downward_roll_rad`：M_PI
  - `downward_pitch_rad`：0.0
- 本轮成功点击的目标：
  - click A：`x=0.448` `y=0.050` `yaw=0.000`
  - click B：`x=0.227` `y=-0.007` `yaw=0.000`
  - click C：`x=0.340` `y=0.056` `yaw=0.000`
- RViz 点击后是否收到 `/goal_pose`：是
- 工作区内目标是否成功规划： 是
- 工作区内目标是否成功执行：是
- 终端中是否出现 `Mapped goal_pose -> target pose`：是
- 终端中是否出现 `Execute request success!`：是
- 你对本轮映射结果的观察：能成功执行运动
- 本轮最终结论：能成功执行运动

### 失败轮次总结
- 是否观察到失败轮次：是
- 观察到的失败类型：工作区越界
- 本轮失败点击的目标：
  - fail A：`x=-0.504` `y=-0.037` `yaw=未看见`
  - fail B：`x=0.862` `y=-0.030` `yaw=未看见`
- 失败时的关键日志：[goal_pose_executor_node-1] [WARN] [1776331558.871402420] [ur3_goal_pose_executor]: Rejected goal outside workspace: x=-0.504 y=-0.037
- 失败时的主要原因：超出工作区界限
- 你准备如何调整：重新设置目标，在工作区内运动

### URSim 冒烟记录
- 运行日期：2026.4.16
- 是否已在 URSim 下验证：是
- URSim 下是否收到 `/goal_pose`：是；终端已多次进入 `on_goal_pose` 回调，并打印 `Mapped goal_pose -> target pose`、`Rejected goal outside workspace` 和 `Unsupported goal frame` 等日志。
- URSim 下工作区内目标是否成功规划：是；例如 `(x=0.257, y=0.011, yaw=0.000)` 和 `(x=0.370, y=0.132, yaw=0.000)` 均出现 `planned=true`。
- URSim 下工作区内目标是否成功执行：是；两次成功规划后都出现了 `Execute request success!`，说明机械臂在 URSim 控制链路下完成了真实执行。
- URSim 下工作区外目标是否被直接拒绝：是；例如 `x=-0.057, y=0.372` 和 `x=-0.221, y=0.010` 被直接判定为 `Rejected goal outside workspace`。
- URSim 下额外遇到的执行链路问题：本轮未观察到新的执行链路故障。`scaled_joint_trajectory_controller` 与 `speed_scaling_state_broadcaster` 均为 `active`，`/speed_scaling_state_broadcaster/speed_scaling` 读到 `100.0`，说明 `External Control` 与执行控制器链路正常。
- fake hardware / URSim 下的差异：fake hardware 下只要规划成功通常就能直接执行；URSim 下除了规划成功，还必须保证 `External Control` 已运行、`scaled_joint_trajectory_controller` 为 `active`，并且 `speed_scaling` 不为 `0.0`。本轮说明 7D 在真实 URSim 执行链路下也能完成“点击 -> 自动规划 -> 执行”。
- 本轮最终结论：7D 已在 URSim 下完成一次主路径验收。程序能够接收 RViz 的 `/goal_pose` 输入，对非法 frame 或越界目标做前置拒绝，并对工作区内的合法目标完成自动规划与执行。


## 7. 完成记录
- 日期：2026.4.17
- 最终采用的参数：
  - `target_height`：0.1519
  - `workspace_min_x`：0.0
  - `workspace_max_x`：0.6
  - `workspace_min_y`：-0.3
  - `workspace_max_y`：0.3
  - `downward_roll_rad`：M_PI
  - `downward_pitch_rad`：0.0
- 最终是否保留输入 `yaw`：是
- 最终观察到的关键现象：程序能够接收 RViz 的 `/goal_pose` 输入，对非法 frame 或越界目标做前置拒绝，并对工作区内的合法目标完成自动规划与执行
- 我对 7D 的一句话总结：把 RViz 的二维人类输入翻译成机械臂三维动作语义，并通过 MoveIt 自动完成规划与执行的最小中间层
- 备注：URSim 验证中额外确认了 `External Control`、`scaled_joint_trajectory_controller` 和 `speed_scaling` 状态对真实执行链路的重要性
