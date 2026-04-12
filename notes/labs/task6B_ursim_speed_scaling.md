# Task 6B：URSim speed scaling 实验记录

## 状态
- `[ ] 未开始`
- `[#] 进行中`
- `[x] 已完成`
- 当前状态：`[x] 已完成 URSim speed scaling 实证与 100% / 50% 轨迹对比`

## 1. 目标
- 在 `workspaces/ws_stage2/src/ur3_ursim_speed_scaling_lab_py` 中完成一个独立的 6B 实验包。
- 验证 `/speed_scaling_state_broadcaster/speed_scaling` 在 URSim 下会随速度滑块变化。
- 用同一条保守轨迹比较 `100%` 与降速条件下的执行节奏差异。

## 2. 应用包位置
- 包路径：`/home/minzi/ros2_lab/workspaces/ws_stage2/src/ur3_ursim_speed_scaling_lab_py`
- 关键节点：
  - `speed_scaling_monitor`
  - `scaled_trajectory_runner`

## 3. 当前准备情况
- 已完成：
  - 新建 6B 独立 Python 应用包骨架；
  - 提供 speed scaling 观测节点；
  - 提供轨迹发送节点骨架、launch 文件与参数 YAML；
  - 已在 `scaled_trajectory_runner.py` 中补完保守轨迹；
  - 已完成 URSim 下的 speed scaling 观测与轨迹对比实验。
- 待你完成：
  - 将本轮观察转写成自己的解释；
  - 若需要，可继续做更低速度档位的扩展复验。

## 4. 练习 1：构建 6B 新包

### 执行命令
```bash
cd /home/minzi/ros2_lab/workspaces/ws_stage2
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ur3_ursim_speed_scaling_lab_py
source install/setup.bash
```

### 结果记录
- 是否构建通过：是
- 如失败，报错关键信息：无

## 5. 练习 2：只验证 speed scaling 观测链路

### 执行命令
```bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur3 \
  robot_ip:=<ursim_container_ip> \
  launch_rviz:=true

ros2 launch ur3_ursim_speed_scaling_lab_py task6B_ursim_speed_scaling.launch.py
```

### 观察记录
- `100%` 速度滑块时的日志：
  - `Update: speed_scaling=100.0% fraction=1.000`
- `50%` 速度滑块时的日志：
  - `Update: speed_scaling=50.0% fraction=0.500`
- 更低速度滑块时的日志：
  - 本轮未继续降低到 `50%` 以下。
- 我观察到的变化：
  - 速度滑块从 `100%` 调到 `50%` 后，`/speed_scaling_state_broadcaster/speed_scaling` 稳定变为 `50.0`。
  - 中途一度出现 `0.0%`，对应的是外部控制程序停止，而不是“50% 没生效”。
  - 通过 `/io_and_status_controller/resend_robot_program` 恢复后，speed scaling 从 `0.0 -> 50.0` 再次稳定。

## 6. 练习 3：补写保守轨迹并做执行对比

### 位置
- 文件：`workspaces/ws_stage2/src/ur3_ursim_speed_scaling_lab_py/ur3_ursim_speed_scaling_lab_py/scaled_trajectory_runner.py`
- 函数：`plan_demo_points()`

### TODO(human) 完成后记录
- 我选择移动哪些关节：
  - `shoulder_pan_joint`
  - `shoulder_lift_joint`
- 每个关节变化幅度大约多少：
  - `shoulder_pan_joint += 0.5 rad`
  - `shoulder_lift_joint -= 0.5 rad`
- 为什么认为这条轨迹足够保守：
  - 只动了 2 个关节；
  - 幅度比 5C 的 `1.0 rad` 更小；
  - 轨迹为“起点 -> 中间点 -> 起点”，便于解释发送前后状态。
- `100%` speed scaling 下的执行结果：
  - `speed_scaling_before_send=100.0%`
  - `status=4 error_code=0`
  - `elapsed_sec=5.00`
- `50%` speed scaling 下的执行结果：
  - `speed_scaling_before_send=50.0%`
  - `status=4 error_code=0`
  - `elapsed_sec=10.00`
- 两次耗时差异：
  - `50%` 比 `100%` 多约 `5s`；
  - 总耗时约为 `2x`，符合“speed scaling 拉伸时间轴”的预期。

## 7. 解释闭环
- 为什么这次可以算“真实 speed scaling 实证”：
  - 在 mock hardware 下，speed scaling 始终固定为 `100%`；
  - 这次在 URSim 下，速度滑块改变后，ROS 话题真实变为 `50.0`；
  - 同一条轨迹在 `50%` 与 `100%` 下表现出明显不同的执行耗时。
- 为什么 result 成功不等于“速度滑块没生效”：
  - `scaled_joint_trajectory_controller` 会在低速条件下拉伸时间轴；
  - 路径仍能被完整执行，所以 Action 仍可能返回成功；
  - 真正的差异体现在执行节奏和 feedback 持续时间上，而不是只看 success / failure。
- 如果换成 `joint_trajectory_controller`，我预计会有什么差异：
  - 它不会消费 speed scaling；
  - 在真机或 URSim 外部控制语义下，不如 `scaled_joint_trajectory_controller` 适合作为默认入口。
- 这次实验对未来手术机器人控制链路设计的启发：
  - 状态观测必须和控制结果一起验证，不能只看命令是否发出；
  - “程序停止导致 speed scaling=0” 与 “速度滑块变为 50%” 是两种不同状态，必须分开解释；
  - 做真实控制系统时，验证时间轴变化与状态回传同样重要。

## 8. 完成记录
- 日期：2026-04-12
- 备注：
  - 已完成 6B 核心验收：
    - `speed_scaling=100.0%` 时轨迹耗时约 `5.00s`
    - `speed_scaling=50.0%` 时轨迹耗时约 `10.00s`
    - 两次均返回 `status=4 error_code=0`
  - 中途定位到一次 `reverse interface dropped -> scaled_joint_trajectory_controller inactive -> speed_scaling=0.0` 的现象，并通过重发 robot program 恢复。

## 9. 概念内化总结

### 这次 6B 真正验证了什么

这次实验真正验证的，不只是“URSim 里有一个会变化的话题”，而是下面这条完整因果链：

```text
URSim 速度滑块变化
        ↓
URSim / RTDE 回传真实速度比例
        ↓
ur_robot_driver 更新 speed_scaling state interface
        ↓
speed_scaling_state_broadcaster 发布新值
        ↓
scaled_joint_trajectory_controller 读取该值并拉伸轨迹时间轴
        ↓
同一条轨迹在不同 speed scaling 下表现出不同执行耗时
```

换句话说，6B 不是只验证“topic 变了”，而是验证“真实状态回传会改变控制器行为”。

### 四个最重要的概念

**概念 1：`speed_scaling` 是 state，不是我想象出来的命令回显**

- 在 mock hardware 下，`set_speed_slider` 虽然能调用成功，但 broadcaster 仍固定发 `100%`。
- 到了 URSim，下调速度滑块后，话题真实变为 `50.0`。
- 这说明 broadcaster 发的是“驱动从外部控制器读回来的状态”，不是“我刚刚写进去的命令值”。

**概念 2：`scaled_joint_trajectory_controller` 的核心不是“变慢”，而是“时间轴缩放”**

- 我这次发的是同一条轨迹。
- 在 `100%` 下耗时约 `5s`，在 `50%` 下耗时约 `10s`。
- 轨迹路径没有换，goal 也没有失败，只是执行时间被大约拉成了 2 倍。
- 所以 speed scaling 的控制语义不是“换一条轨迹”，而是“让同一条轨迹按更慢的节奏完成”。

**概念 3：`speed_scaling=0.0` 和 `speed_scaling=50.0` 是两种完全不同的系统状态**

- `50.0` 表示外部控制链路仍然活着，只是当前允许的执行速度是原来的 50%。
- `0.0` 这次对应的是 `reverse interface dropped`，随后 `scaled_joint_trajectory_controller` 被自动降为 inactive。
- 这意味着系统已经从“降速运行”切换成了“控制程序停止 / 无法继续执行”的状态。
- 所以看到 `0.0` 时，第一反应不该是“这是更低的 speed scaling”，而该先检查程序状态、控制器状态和 reverse interface。

**概念 4：机器人实验里，证据要分层看，不能只盯一个信号**

- 只看 topic：你能知道状态变了，但不知道控制器是否还可执行。
- 只看 action result：你能知道轨迹成功了，但不知道速度滑块是否真的影响了时间轴。
- 只看 URSim 界面：你能知道自己拖了滑块，但不知道 ROS 侧有没有收到真实状态。
- 这次 6B 让我更清楚：至少要把“UI 操作 + topic 变化 + controller 状态 + action 耗时”四层证据连起来看。

### 四个核心问题自答

**Q1：为什么同一条轨迹在 `50%` 下大约用了 `10s`，在 `100%` 下用了 `5s`？**

因为 `scaled_joint_trajectory_controller` 会消费 `speed_scaling`，并把轨迹时间轴按比例拉伸。`50%` 并不是“只完成一半轨迹”，而是“用两倍时间完成同一条轨迹”。

**Q2：为什么这次 Action 结果都成功，却仍能证明速度滑块生效了？**

因为 speed scaling 的影响主要落在“执行节奏”上，不一定落在“是否成功”上。只要轨迹仍可执行，控制器就可能返回成功；真正说明滑块生效的证据，是 `speed_scaling` 从 `100.0` 变成了 `50.0`，同时总耗时从 `5s` 变成了 `10s`。

**Q3：为什么我这次看到过 `speed_scaling=0.0`，但最后又能恢复到 `50.0`？**

因为那次 `0.0` 不是“我把滑块调到了 0%”，而是外部控制程序停了，driver 日志里也对应出现了 `reverse interface dropped`。恢复 robot program 后，控制链路回来了，`speed_scaling` 才重新回到 `50.0`。

**Q4：为什么 6B 值得新建独立应用包，而不是继续写进 5C？**

因为 6B 的重点已经从“最小控制闭环”升级成了“真实状态回传如何改变控制器行为”。它需要自己的 monitor、自己的 runner、自己的实验记录和自己的异常解释。把它单独放进 `ur3_ursim_speed_scaling_lab_py`，可以让实验目的、代码边界和排障路径都更清楚。

### 对未来手术机器人开发的迁移理解

如果把 6B 压缩成对未来最有用的一句话，那就是：

> 在真实控制系统里，"命令发出去了" 不等于 "系统就按我想的方式执行了"；必须同时验证状态回传、控制器行为和时间语义。

这次 6B 练到的，不只是 UR 的 speed scaling，而是一种以后会不断重复用到的工程习惯：

- 先分清 command 和 state；
- 再确认状态是否真实来自外部系统；
- 再验证控制器有没有消费这个状态；
- 最后用执行结果和时间特征证明“这个状态真的改变了系统行为”。
