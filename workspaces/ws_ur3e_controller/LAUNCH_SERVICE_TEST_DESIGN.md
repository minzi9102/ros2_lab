# ws_ur3e_controller launch + service 自动测试设计

本文档定义仿真 v1 下一步的 launch + service 级自动测试方案。目标是用尽量接近人工验收的方式启动完整 MoveIt 仿真，并通过 service 调用验证命名点控制器主流程。

## 1. 测试目标

自动测试应覆盖当前人工已经验证过的核心路径：

- 启动完整 `sim_named_motion_bringup.launch.py`。
- 等待 `/ur3e_named_motion_controller/execute_named_target` service 可用。
- 请求 `home` plan-only，期望返回 `status='planned'`。
- 请求 `ready` plan-only，期望返回 `status='planned'`。
- 在允许执行的 launch 配置下请求 `ready` execute，期望返回 `status='executed'`。
- execute 后验证 final-target gate 已由 service message 确认通过。

## 2. 第一版范围

第一版只做成功路径和一个关键拒绝路径：

- `execute=false` bringup 下，`ready` plan-only 成功。
- `execute=true` bringup 下，`ready` execute 成功。
- `execute=false` bringup 下，请求 `execute=true` 返回 `rejected_execution_disabled`。

状态：已完成。对应测试文件为 `src/ur3e_named_motion_controller_cpp/test/test_sim_launch_service.py`。

暂不做：

- Dashboard mock。
- speed scaling mock。
- Remote Control mock。
- 真机 safety mode 分支。
- joint state 缺失、关节名不完整、delta 超限等状态注入测试。

这些拒绝路径留到第二版，避免第一版测试夹带过多测试专用设施。

## 3. 建议文件结构

优先在现有包内新增测试文件：

```text
src/ur3e_named_motion_controller_cpp/test/
  test_sim_launch_service.py
```

如果需要 pytest 标记或 launch testing 配置，再修改：

```text
src/ur3e_named_motion_controller_cpp/CMakeLists.txt
```

第一版不新增生产代码接口，不新增测试专用 launch 文件。

## 4. 启动策略

测试入口优先复用：

```text
ur3e_named_motion_controller_cpp/launch/sim_named_motion_bringup.launch.py
```

建议分成两个测试用例或两个 launch fixture：

- plan-only fixture：`runtime_mode:=sim execute:=false launch_rviz:=false`
- execute fixture：`runtime_mode:=sim execute:=true launch_rviz:=false`

每个测试都应设置充足超时，因为完整 MoveIt 仿真启动比普通单元测试慢。

## 5. Service 调用检查

测试客户端逻辑：

1. 初始化 `rclpy`。
2. 创建临时测试节点。
3. 等待 `/ur3e_named_motion_controller/execute_named_target` service。
4. 发送 `ExecuteNamedTarget` 请求。
5. 等待 future 完成。
6. 断言响应字段。

第一版断言建议：

```text
accepted == True
planned == True
status == 'planned' 或 'executed'
```

execute 成功路径额外断言：

```text
executed == True
message 包含 'final-target gate passed'
```

execution-disabled 拒绝路径断言：

```text
accepted == False
planned == False
executed == False
status == 'rejected_execution_disabled'
```

## 6. 设计取舍

使用 launch + service 测试的价值：

- 覆盖真实 launch 参数传递。
- 覆盖 MoveIt、fake hardware、controller 节点之间的集成。
- 与人工验收流程一致，失败时更接近真实问题。

代价：

- 测试耗时更长。
- 对 ROS 图启动时序更敏感。
- 失败日志更长，需要在测试中打印 service response 和关键 launch 事件。

第一版接受这些代价，因为本阶段目标就是验证完整 MoveIt 仿真主链路。

## 7. 下一步实现顺序

已完成：

1. `execute=false` 下的 `ready` plan-only 自动测试。
2. `execute=false` 下的 `execute=true` 拒绝测试。
3. `execute=true` 下的 `ready` execute 自动测试。

最近一次通过验证：

```bash
cd /home/minzi/ros2_lab/workspaces/ws_ur3e_controller
source /opt/ros/jazzy/setup.bash
colcon test --packages-select ur3e_named_motion_controller_cpp --event-handlers console_direct+
```

当前 `test_sim_launch_service` 包含三个完整仿真用例，最近一次 pytest 部分耗时约 `18.43s`。

下一步转入第二版拒绝路径设计。优先选择不需要复杂 ROS 状态注入的路径，例如 `rejected_unknown_target` 或 disabled target。
