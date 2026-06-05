# ws_ur3e_controller 真机 HOME/READY 图形界面开发计划

本文档定义 `ws_ur3e_controller` 的第一版真机图形界面计划。目标是新增一个最小可用的独立 GUI 小面板，让操作者点击 `HOME` 或 `READY` 按钮后，通过现有 service 控制真实 UR3e 运动到已复核的命名点位。

本阶段只写计划，不实施代码。

## 1. 阶段目标

第一版 GUI 只追求一个小而完整的真机操作闭环：

- 启动一个独立 GUI 小窗口。
- GUI 连接现有 service：`/ur3e_named_motion_controller/execute_named_target`。
- 点击 `HOME` 按钮后发送 `home` 的真实执行请求。
- 点击 `READY` 按钮后发送 `ready` 的真实执行请求。
- 界面显示 service 连接状态、请求状态和最近一次响应。
- 后端返回失败时，界面直接显示 `status` 和 `message`，不自动重试、不自动恢复。

## 2. YAGNI / KISS 原则

本阶段遵守以下约束：

- 不新增控制协议。
- 不修改 `ExecuteNamedTarget.srv`。
- 不修改 `named_motion_controller_node.cpp` 的执行门闩逻辑。
- 不修改 `ur3e_named_targets.yaml` 中 `home` / `ready` 的点位。
- 不新增第三个命名目标。
- 不做目标编辑、目标管理或 catalog 可视化。
- 不做 waypoint 队列、连续控制、自动重试、自动恢复或 Action cancel。
- 不引入 Web 服务、数据库、rqt 插件或复杂前端框架。
- GUI 只做薄客户端，真实安全检查继续由现有 C++ 控制器负责。

## 3. 技术方案

新增一个独立 Python ROS 2 GUI 包，建议命名为：

```text
ur3e_named_motion_gui_py
```

推荐技术栈：

- `rclpy`
- `python_qt_binding`
- Qt Widgets
- `ur3e_controller_msgs/srv/ExecuteNamedTarget`

选择独立 Qt 小面板的原因：

- 本机已有 `rclpy`、`PyQt5` 和 `python_qt_binding` 可用。
- 相比 rqt 插件，独立小窗口脚手架更少，调试路径更短。
- 相比 Web 面板，不需要引入额外 bridge、server 或浏览器通信层。

## 4. GUI 行为

界面只包含第一版必需控件：

- `HOME` 按钮
- `READY` 按钮
- service 状态显示
- 请求状态显示
- 最近一次 service 响应显示

按钮请求内容固定为：

```text
HOME:
target_name: home
execute: true
human_confirmation: <confirmation_token>

READY:
target_name: ready
execute: true
human_confirmation: <confirmation_token>
```

确认 token 默认使用现有控制器默认值：

```text
I_CONFIRM_REAL_ROBOT_MOTION
```

第一版按“纯一键按钮”设计：

- GUI 自动携带确认 token。
- 点击 `HOME` 或 `READY` 后直接发送真实 execute 请求。
- 不额外弹出确认框。
- 不在 GUI 中复制 Dashboard、controller、speed scaling 等安全检查。

请求期间：

- 两个运动按钮临时禁用。
- 状态显示为请求中。
- service 返回后恢复按钮状态。

service 不可用时：

- 界面显示 service 未连接。
- 按钮禁用，或点击后显示 service 不可用。
- 不启动控制器、不启动 MoveIt、不尝试自动修复环境。

响应处理：

- `status == "executed"` 且 `executed == true` 时显示成功。
- 其他状态均显示失败或拒绝，并展示后端返回的 `status` 和 `message`。
- 不根据失败原因自动重试或执行恢复命令。

## 5. 运行前置条件

使用 GUI 前必须由操作者先完成真实机器人运行环境准备：

- 真机 bringup 已启动。
- Dashboard client 和 External Control 已正常运行。
- MoveIt 语义层已启动。
- `ur3e_named_motion_controller` 已启动，并使用：

```text
runtime_mode:=real
execute:=true
```

- real catalog 中 `home` 和 `ready` 已经现场复核并保持 `enabled=true`。
- 机器人周围环境已完成现场安全确认。

GUI 不负责完成以上前置步骤，只负责显示 service 是否可用，并发送命名目标执行请求。

## 6. 推荐文件与接口

预计新增内容：

```text
src/ur3e_named_motion_gui_py/
  package.xml
  setup.py
  setup.cfg
  resource/ur3e_named_motion_gui_py
  ur3e_named_motion_gui_py/
    __init__.py
    main.py
  launch/
    real_named_motion_gui.launch.py
  test/
    test_request_builder.py
```

推荐 ROS 参数：

- `service_name`
  - 默认：`/ur3e_named_motion_controller/execute_named_target`
- `human_confirmation`
  - 默认：`I_CONFIRM_REAL_ROBOT_MOTION`
- `service_wait_timeout_sec`
  - 默认：`1.0`

第一版不增加 runtime mode 参数。该 GUI 文档和命名都明确面向真实机器人 HOME / READY 执行。

## 7. 测试计划

轻量测试优先覆盖请求构造和响应解释：

- `HOME` 请求字段正确：
  - `target_name == "home"`
  - `execute == true`
  - `human_confirmation == token`
- `READY` 请求字段正确：
  - `target_name == "ready"`
  - `execute == true`
  - `human_confirmation == token`
- service 返回 `executed` 时，GUI 状态解释为成功。
- service 返回 `rejected_real_gate` 时，GUI 状态解释为失败并保留后端 message。
- service 返回 `planning_failed` 时，GUI 状态解释为失败并保留后端 message。
- service 返回 `execution_failed` 时，GUI 状态解释为失败并保留后端 message。

手动替代验证：

- 用 mock service 返回固定成功响应，验证 `HOME` / `READY` 按钮可触发请求。
- 用 mock service 返回固定失败响应，验证界面显示后端 `status` 和 `message`。
- 在 service 不存在时启动 GUI，验证按钮不可用或清晰显示 service 未连接。

真机验收：

1. 启动完整真机链路。
2. 使用命令行 service 先执行 plan-only 验证当前目标可规划。
3. 启动 GUI。
4. 点击 `HOME`，确认返回 `status=executed`。
5. 点击 `READY`，确认返回 `status=executed`。
6. 若失败，只记录 GUI 显示的 `status` 和 `message`，不在 GUI 内自动恢复。

## 8. Learn Mode 分工

适合智能体直接完成：

- Python 包脚手架。
- launch 文件。
- service client 胶水代码。
- 请求构造函数。
- 响应状态解释函数。
- 轻量测试骨架和机械测试。
- 使用说明文档补充。

适合人类重点参与：

- 现场确认纯一键按钮是否符合当前安全流程。
- 确认 token 是否允许默认写入 launch 参数。
- 确认 GUI 文案是否足够提醒“真实机器人将运动”。
- 真机 HOME / READY 验收。

如果后续进入实现阶段，在写 GUI 执行按钮核心逻辑前，应再次确认真实机器人现场安全策略。

## 9. 暂不做的事项

第一版明确不做：

- 仿真 / 真机模式切换。
- 点位新增、删除、编辑。
- catalog 文件读取和可视化。
- Dashboard 恢复命令。
- controller 启停控制。
- MoveIt 或真机 bringup 自动启动。
- 操作日志持久化。
- 权限系统。
- 多机器人支持。
- 多目标队列。
- 急停功能。

急停和物理安全链路应继续依赖机器人现场已有安全机制，而不是由这个 GUI 第一版承担。

## 10. 推荐下一步

下一步可以按一个独立小任务实现：

1. 新建 `ur3e_named_motion_gui_py` 包。
2. 先实现请求构造和响应解释的可测试纯逻辑。
3. 再接入 Qt 小窗口和 ROS service client。
4. 使用 mock service 做手动验证。
5. 最后在真实机器人环境中低速验收 `HOME` 和 `READY`。
